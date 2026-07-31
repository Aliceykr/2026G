#!/usr/bin/env python3
"""Automated amplitude/phase calibration for the G measurement chain."""

from __future__ import annotations

import argparse
import csv
import json
import math
import select
import socket
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from siglent_vxi11 import Vxi11Instrument


class RigolScpiInstrument:
    """Persistent raw-SCPI connection used by the RIGOL DG1022Z."""

    def __init__(self, host: str, timeout: float = 3.0, port: int = 5555) -> None:
        self.host = host
        self.timeout = timeout
        self.port = port
        self.sock: socket.socket | None = None

    def open(self) -> None:
        self.sock = socket.create_connection((self.host, self.port), self.timeout)
        self.sock.settimeout(self.timeout)

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def write(self, command: str) -> None:
        if self.sock is None:
            raise ConnectionError("RIGOL SCPI connection is not open")
        self.sock.sendall((command.rstrip() + "\n").encode("ascii"))
        # DG1022Z firmware 00.01.12 drops rapid back-to-back state changes:
        # queries show the new values while the physical waveform can remain
        # unchanged.  A 50 ms command spacing was verified on the instrument.
        time.sleep(0.05)

    def query(self, command: str) -> str:
        if not command.rstrip().endswith("?"):
            raise ValueError("SCPI query must end in '?'")
        self.write(command)
        if self.sock is None:
            raise ConnectionError("RIGOL SCPI connection is not open")
        response = bytearray()
        while not response.endswith(b"\n"):
            block = self.sock.recv(4096)
            if not block:
                raise ConnectionError("RIGOL SCPI socket closed unexpectedly")
            response.extend(block)
        return response.decode("ascii", errors="replace").strip()


def parse_series(text: str) -> list[float]:
    values: list[float] = []
    for part in text.split(","):
        fields = part.strip().split(":")
        if len(fields) == 1:
            values.append(float(fields[0]))
            continue
        if len(fields) != 3:
            raise ValueError(f"invalid series: {part}")
        start, stop, step = map(float, fields)
        if step <= 0 or stop < start:
            raise ValueError(f"invalid range: {part}")
        value = start
        while value <= stop + 0.25 * step:
            values.append(value)
            value += step
    return values


def wrap_degrees(value: float) -> float:
    while value <= -180.0:
        value += 360.0
    while value > 180.0:
        value -= 360.0
    return value


def circular_mean_degrees(values: list[float]) -> tuple[float, float, float]:
    radians = [math.radians(value) for value in values]
    cosine = statistics.fmean(math.cos(value) for value in radians)
    sine = statistics.fmean(math.sin(value) for value in radians)
    resultant = math.hypot(cosine, sine)
    mean = math.degrees(math.atan2(sine, cosine))
    stddev = math.degrees(math.sqrt(max(0.0, -2.0 * math.log(max(resultant, 1e-12)))))
    return wrap_degrees(mean), resultant, stddev


class Generator:
    def __init__(self, host: str, kind: str = "auto") -> None:
        self.host = host
        self.kind = kind
        self.instrument = None
        self.is_rigol = False

    def __enter__(self) -> "Generator":
        if self.kind in ("auto", "rigol"):
            try:
                instrument = RigolScpiInstrument(self.host, 3.0)
                instrument.open()
                identity = instrument.query("*IDN?")
                if "RIGOL" in identity.upper():
                    self.instrument = instrument
                    self.is_rigol = True
                    print(f"generator: {identity}")
                    return self
                instrument.close()
                if self.kind == "rigol":
                    raise RuntimeError(f"expected RIGOL generator, received: {identity}")
            except (ConnectionError, OSError, RuntimeError):
                if self.kind == "rigol":
                    raise

        self.instrument = Vxi11Instrument(self.host, 3.0)
        self.instrument.open()
        identity = self.instrument.query("*IDN?")
        if self.kind == "siglent" and "SIGLENT" not in identity.upper():
            raise RuntimeError(f"expected SIGLENT generator, received: {identity}")
        print(f"generator: {identity}")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.instrument.close()

    def _verify_rigol_sine(self, frequency_hz: float, vpp_mv: float) -> None:
        response = self.instrument.query(":SOUR1:APPL?").strip('"')
        fields = response.split(",")
        if len(fields) < 3 or fields[0].upper() != "SIN":
            raise RuntimeError(f"unexpected RIGOL waveform readback: {response}")
        actual_frequency_hz = float(fields[1])
        actual_vpp_mv = float(fields[2]) * 1000.0
        if abs(actual_frequency_hz - frequency_hz) > max(0.5, frequency_hz * 1e-6):
            raise RuntimeError(
                f"RIGOL frequency readback mismatch: {actual_frequency_hz} Hz"
            )
        if abs(actual_vpp_mv - vpp_mv) > max(0.001, vpp_mv * 1e-5):
            raise RuntimeError(
                f"RIGOL amplitude readback mismatch: {actual_vpp_mv} mVpp"
            )

    def _reconnect_rigol(self) -> None:
        # DG1022Z firmware 00.01.12 physically applies the first parameter
        # batch on a raw-SCPI connection, but later batches may only update
        # query readback.  Reconnect for every calibration point.
        self.instrument.close()
        self.instrument.open()

    def _apply_rigol_commands_isolated(self, commands: list[str]) -> None:
        # The raw-SCPI service can acknowledge a batch while physically
        # applying only some setters.  Make every setter the first command of
        # a fresh TCP session; this is slower but deterministic for calibration.
        for command in commands:
            self.instrument.close()
            self.instrument.open()
            self.instrument.write(command)
        self.instrument.close()
        self.instrument.open()

    def sine(self, frequency_hz: float, vpp_mv: float, phase_deg: float = 0.0) -> None:
        if self.instrument is None:
            raise ConnectionError("generator is not connected")
        if self.is_rigol:
            # DG1022Z firmware 00.01.12 can update APPL? readback without
            # applying APPL:SIN to the waveform engine.  Independent setters
            # have been verified at the physical output and must be used here.
            self._apply_rigol_commands_isolated([
                ":OUTP1 OFF",
                ":SOUR1:FUNC SIN",
                # FREQ:FIX changes readback but can leave output unchanged.
                f":SOUR1:FREQ {frequency_hz:.9g}",
                ":SOUR1:VOLT:UNIT VPP",
                f":SOUR1:VOLT {vpp_mv / 1000.0:.9g}",
                ":SOUR1:VOLT:OFFS 0",
                f":SOUR1:PHAS {phase_deg:.9g}",
                ":OUTP1:LOAD 50",
                ":OUTP1 ON",
            ])
            if self.instrument.query("*OPC?") != "1":
                raise RuntimeError("RIGOL did not complete the sine update")
            error = self.instrument.query(":SYST:ERR?")
            if not error.startswith("0,"):
                raise RuntimeError(f"RIGOL SCPI error: {error}")
            self._verify_rigol_sine(frequency_hz, vpp_mv)
            return

        self.instrument.write("C1:OUTP OFF")
        self.instrument.write("C1:HARM HARMSTATE,OFF")
        self.instrument.write("C1:OUTP LOAD,50")
        self.instrument.write(
            "C1:BSWV WVTP,SINE,FRQ,{:.9g}HZ,AMP,{:.9g}V,OFST,0V,PHSE,{:.9g}".format(
                frequency_hz, vpp_mv / 1000.0, phase_deg
            )
        )
        self.instrument.write("C1:OUTP ON")

    def harmonics(
        self,
        fundamental_hz: float,
        fundamental_vpp_mv: float,
        fundamental_phase_deg: float,
        items: list[tuple[int, float, float]],
    ) -> dict[int, float]:
        if any(order < 2 or order > 10 for order, _vpp, _phase in items):
            raise ValueError("harmonic order must be 2..10")
        if self.instrument is None:
            raise ConnectionError("generator is not connected")
        if self.is_rigol:
            highest_order = max([order for order, _vpp, _phase in items], default=2)
            selected_orders = {order for order, _vpp, _phase in items}
            user_phase_supported = highest_order <= 8
            commands = [
                ":OUTP1 OFF",
                ":OUTP1:LOAD 50",
                ":SOUR1:APPL:HARM {:.9g},{:.9g},0,{:.9g}".format(
                    fundamental_hz,
                    fundamental_vpp_mv / 1000.0,
                    fundamental_phase_deg,
                ),
                f":SOUR1:HARM:ORDER {highest_order}",
            ]
            if user_phase_supported:
                user_mask = "X" + "".join(
                    "1" if order in selected_orders else "0"
                    for order in range(2, 9)
                )
                commands.extend([
                    ":SOUR1:HARM:TYPE USER",
                    f":SOUR1:HARM:USER {user_mask}",
                ])
            else:
                # DG1022Z的USER选择位只覆盖H2~H8。H9/H10使用ALL，
                # 其余阶次幅值清零，并统一使用仪器实际生效的0度相位。
                commands.append(":SOUR1:HARM:TYPE ALL")
            for order in range(2, highest_order + 1):
                commands.append(f":SOUR1:HARM:AMPL {order},0")
                commands.append(f":SOUR1:HARM:PHAS {order},0")
            for order, vpp_mv, phase_deg in items:
                commands.append(
                    f":SOUR1:HARM:AMPL {order},{vpp_mv / 1000.0:.9g}"
                )
                commands.append(
                    ":SOUR1:HARM:PHAS {},{:.9g}".format(
                        order,
                        phase_deg if user_phase_supported else 0.0,
                    )
                )
            commands.append(":OUTP1 ON")

            # DG1022Z firmware 00.01.12 has the same stale-waveform behavior
            # in harmonic mode as in sine mode: setters after the first one on
            # a raw-SCPI connection may update query readback without reaching
            # the waveform engine.  Isolate every setter in a fresh session.
            self._apply_rigol_commands_isolated(commands)
            error = self.instrument.query(":SYST:ERR?")
            if not error.startswith("0,"):
                raise RuntimeError(f"RIGOL SCPI error: {error}")
            if self.instrument.query("*OPC?") != "1":
                raise RuntimeError("RIGOL did not complete the harmonic update")
            return {
                order: (phase_deg if user_phase_supported else 0.0)
                for order, _vpp, phase_deg in items
            }

        self.instrument.write("C1:OUTP OFF")
        self.instrument.write("C1:HARM HARMSTATE,OFF")
        self.instrument.write("C1:OUTP LOAD,50")
        self.instrument.write(
            "C1:BSWV WVTP,SINE,FRQ,{:.9g}HZ,AMP,{:.9g}V,OFST,0V,PHSE,{:.9g}".format(
                fundamental_hz,
                fundamental_vpp_mv / 1000.0,
                fundamental_phase_deg,
            )
        )
        self.instrument.write("C1:HARM HARMTYPE,ALL")
        for order in range(2, 11):
            self.instrument.write(f"C1:HARM HARMORDER,{order},HARMAMP,0V,HARMPHASE,0")
        for order, vpp_mv, phase_deg in items:
            self.instrument.write(
                "C1:HARM HARMORDER,{},HARMAMP,{:.9g}V,HARMPHASE,{:.9g}".format(
                    order, vpp_mv / 1000.0, phase_deg
                )
            )
        self.instrument.write("C1:HARM HARMSTATE,ON")
        self.instrument.write("C1:OUTP ON")
        return {order: phase_deg for order, _vpp, phase_deg in items}

    def restore(self) -> None:
        self.sine(100000.0, 100.0, 0.0)


class OmniProbe:
    def __init__(self, port: int) -> None:
        self.port = port
        self.sock: socket.socket | None = None
        self.stream = None

    def __enter__(self) -> "OmniProbe":
        self.sock = socket.create_connection(("127.0.0.1", self.port), timeout=3.0)
        self.stream = self.sock.makefile("rb", buffering=0)
        hello = json.loads(self.stream.readline())
        if hello.get("schema") != "ek.telemetry/v1" or hello.get("type") != "hello":
            raise RuntimeError("EK-OmniProbe bridge handshake failed")
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self.stream is not None:
            self.stream.close()
        if self.sock is not None:
            self.sock.close()

    def _messages(self, seconds: float):
        assert self.sock is not None and self.stream is not None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            timeout = min(0.25, max(0.0, deadline - time.monotonic()))
            if not select.select([self.sock], [], [], timeout)[0]:
                continue
            line = self.stream.readline()
            if not line:
                raise ConnectionError("EK-OmniProbe bridge closed")
            yield json.loads(line)

    def drain(self, seconds: float) -> None:
        for _message in self._messages(seconds):
            pass

    def collect(
        self,
        fundamental_hz: float,
        harmonics: list[int],
        frame_count: int,
        timeout: float,
    ) -> list[dict[int, dict[str, float]]]:
        pending: dict[int, dict[int, dict[str, float]]] = {}
        completed: list[dict[int, dict[str, float]]] = []
        for message in self._messages(timeout):
            if message.get("type") != "samples":
                continue
            for sample in message.get("samples", []):
                values = sample.get("values", {})
                required = ("seq", "h", "f", "codes", "phase_sin")
                if not all(key in values for key in required):
                    continue
                seq = int(round(values["seq"]))
                harmonic = int(round(values["h"]))
                if harmonic not in harmonics:
                    continue
                expected_hz = fundamental_hz * harmonic
                if abs(values["f"] - expected_hz) > max(1000.0, expected_hz * 0.002):
                    continue
                pending.setdefault(seq, {})[harmonic] = {
                    key: float(values[key]) for key in required
                }
                for optional_key in ("amp", "k", "rel"):
                    if optional_key in values:
                        pending[seq][harmonic][optional_key] = float(
                            values[optional_key]
                        )
                if all(item in pending[seq] for item in harmonics):
                    completed.append(pending.pop(seq))
                    if len(completed) >= frame_count:
                        return completed
            if len(pending) > 100:
                for old_seq in sorted(pending)[:-50]:
                    pending.pop(old_seq, None)
        raise RuntimeError(
            f"only received {len(completed)}/{frame_count} complete frames for {harmonics}"
        )


def write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def fit_amplitude(rows: list[dict], c_path: Path) -> None:
    grouped: dict[float, list[dict]] = {}
    for row in rows:
        grouped.setdefault(float(row["frequency_hz"]), []).append(row)

    lines = [
        "static const GMeasurementAmplitudeCalibrationRow s_AmplitudeCalibrationRows[] =",
        "{",
    ]
    worst_equivalent_noise = 0.0
    for frequency_hz in sorted(grouped):
        samples = sorted(grouped[frequency_hz], key=lambda row: float(row["vpp_mv"]))
        if len(samples) > 5:
            raise RuntimeError("firmware supports at most five amplitude levels per frequency")
        lines.append(f"    {{ {frequency_hz:9.1f}f, {len(samples)}U, {{")
        for row in samples:
            codes = float(row["codes_median"])
            peak_mv = 0.5 * float(row["vpp_mv"])
            code_noise = float(row["codes_stddev"])
            equivalent_noise = peak_mv * code_noise / codes
            worst_equivalent_noise = max(worst_equivalent_noise, equivalent_noise)
            lines.append(
                f"        {{ {codes:11.4f}f, {peak_mv:8.4f}f }},"
                f"  // sigma ~= {equivalent_noise:.4f} mV"
            )
        lines.append("    } },")
    lines.extend(
        [
            "};",
            "",
            f"/* worst one-frame equivalent peak noise: {worst_equivalent_noise:.6f} mV */",
        ]
    )
    c_path.parent.mkdir(parents=True, exist_ok=True)
    c_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"amplitude table: {c_path}")
    print(f"worst one-frame equivalent peak noise: {worst_equivalent_noise:.6f} mV")


def amplitude_sweep(args) -> int:
    frequencies = parse_series(args.frequencies)
    amplitudes = parse_series(args.vpps)
    rows: list[dict] = []
    if args.resume and Path(args.output).exists():
        with Path(args.output).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    completed = {
        (round(float(row["frequency_hz"]), 3), round(float(row["vpp_mv"]), 3))
        for row in rows
    }
    with Generator(args.generator, args.generator_kind) as generator, OmniProbe(args.port) as probe:
        try:
            total = len(frequencies) * len(amplitudes)
            progress = 0
            for frequency_hz in frequencies:
                for vpp_mv in amplitudes:
                    if (round(frequency_hz, 3), round(vpp_mv, 3)) in completed:
                        continue
                    progress += 1
                    print(
                        f"[{progress}/{total}] sine {frequency_hz:.0f} Hz, {vpp_mv:.3f} mVpp",
                        flush=True,
                    )
                    generator.sine(frequency_hz, vpp_mv)
                    probe.drain(args.settle)
                    frames = probe.collect(
                        frequency_hz, [1], args.frames, args.timeout
                    )
                    codes = [frame[1]["codes"] for frame in frames]
                    measured_frequencies = [frame[1]["f"] for frame in frames]
                    rows.append(
                        {
                            "frequency_hz": frequency_hz,
                            "vpp_mv": vpp_mv,
                            "frames": len(frames),
                            "codes_mean": statistics.fmean(codes),
                            "codes_median": statistics.median(codes),
                            "codes_stddev": statistics.pstdev(codes),
                            "measured_frequency_mean_hz": statistics.fmean(measured_frequencies),
                        }
                    )
                    completed.add((round(frequency_hz, 3), round(vpp_mv, 3)))
                    write_csv(
                        Path(args.output),
                        rows,
                        [
                            "frequency_hz", "vpp_mv", "frames",
                            "codes_mean", "codes_median", "codes_stddev",
                            "measured_frequency_mean_hz",
                        ],
                    )
            write_csv(
                Path(args.output),
                rows,
                [
                    "frequency_hz",
                    "vpp_mv",
                    "frames",
                    "codes_mean",
                    "codes_median",
                    "codes_stddev",
                    "measured_frequency_mean_hz",
                ],
            )
            fit_amplitude(rows, Path(args.c_output))
        finally:
            if not args.no_restore:
                generator.restore()
    return 0


def build_amplitude_table(args) -> int:
    with Path(args.input).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError("amplitude CSV is empty")
    fit_amplitude(rows, Path(args.output))
    return 0


def phase_sweep(args) -> int:
    fundamentals = parse_series(args.fundamentals)
    requested_orders = [int(round(value)) for value in parse_series(args.orders)]
    rows: list[dict] = []
    if args.resume and Path(args.output).exists():
        with Path(args.output).open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
    completed = {
        (round(float(row["fundamental_hz"]), 3), int(row["harmonic"]))
        for row in rows
    }
    with Generator(args.generator, args.generator_kind) as generator, OmniProbe(args.port) as probe:
        try:
            settings: list[tuple[float, list[int]]] = []
            for fundamental_hz in fundamentals:
                valid = [
                    order for order in requested_orders
                    if 2 <= order <= 10 and order * fundamental_hz <= 500000.0
                    and (round(fundamental_hz, 3), order) not in completed
                ]
                for index in range(0, len(valid), 2):
                    settings.append((fundamental_hz, valid[index:index + 2]))

            for position, (fundamental_hz, orders) in enumerate(settings, 1):
                phase_items = []
                phase_by_order: dict[int, float] = {}
                for order in orders:
                    phase = float((37 * order + 13) % 360)
                    phase_by_order[order] = phase
                    phase_items.append((order, args.harmonic_vpp, phase))
                print(
                    f"[{position}/{len(settings)}] f0={fundamental_hz:.0f} Hz, orders={orders}",
                    flush=True,
                )
                phase_by_order = generator.harmonics(
                    fundamental_hz,
                    args.fundamental_vpp,
                    args.fundamental_phase,
                    phase_items,
                )
                probe.drain(args.settle)
                frames = probe.collect(
                    fundamental_hz,
                    [1] + orders,
                    args.frames,
                    args.timeout,
                )
                for order in orders:
                    # 两台信号源的谐波相位参数均表示该谐波的相对相位；
                    # 基波PHSE移动完整波形，不能再次按n*PHSE重复扣除。
                    true_relative = wrap_degrees(phase_by_order[order])
                    errors = []
                    for frame in frames:
                        measured_relative = wrap_degrees(
                            frame[order]["phase_sin"] -
                            order * frame[1]["phase_sin"]
                        )
                        errors.append(wrap_degrees(measured_relative - true_relative))
                    mean_error, resultant, circular_stddev = circular_mean_degrees(errors)
                    rows.append(
                        {
                            "fundamental_hz": fundamental_hz,
                            "harmonic": order,
                            "target_frequency_hz": fundamental_hz * order,
                            "frames": len(frames),
                            "true_relative_deg": true_relative,
                            "measured_error_deg": mean_error,
                            "circular_stddev_deg": circular_stddev,
                            "resultant": resultant,
                        }
                    )
                    completed.add((round(fundamental_hz, 3), order))
                write_csv(
                    Path(args.output),
                    rows,
                    [
                        "fundamental_hz", "harmonic", "target_frequency_hz",
                        "frames", "true_relative_deg", "measured_error_deg",
                        "circular_stddev_deg", "resultant",
                    ],
                )
            write_csv(
                Path(args.output),
                rows,
                [
                    "fundamental_hz",
                    "harmonic",
                    "target_frequency_hz",
                    "frames",
                    "true_relative_deg",
                    "measured_error_deg",
                    "circular_stddev_deg",
                    "resultant",
                ],
            )
            print(f"phase observations: {args.output}")
        finally:
            if not args.no_restore:
                generator.restore()
    return 0


def solve_linear(matrix: list[list[float]], vector: list[float]) -> list[float]:
    size = len(vector)
    augmented = [matrix[row][:] + [vector[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise RuntimeError("phase calibration matrix is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        for item in range(column, size + 1):
            augmented[column][item] /= scale
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor == 0.0:
                continue
            for item in range(column, size + 1):
                augmented[row][item] -= factor * augmented[column][item]
    return [augmented[row][size] for row in range(size)]


def build_phase_table(args) -> int:
    frequencies = parse_series(args.grid)
    index_by_frequency = {round(value): index for index, value in enumerate(frequencies)}
    size = len(frequencies)
    normal = [[0.0] * size for _ in range(size)]
    rhs = [0.0] * size
    observations: list[tuple[int, int, int, float]] = []

    def add_equation(coefficients: dict[int, float], value: float, weight: float) -> None:
        for row, row_value in coefficients.items():
            rhs[row] += weight * row_value * value
            for column, column_value in coefficients.items():
                normal[row][column] += weight * row_value * column_value

    with Path(args.input).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            fundamental_hz = round(float(row["fundamental_hz"]))
            target_hz = round(float(row["target_frequency_hz"]))
            harmonic = int(row["harmonic"])
            if fundamental_hz not in index_by_frequency or target_hz not in index_by_frequency:
                continue
            fundamental_index = index_by_frequency[fundamental_hz]
            target_index = index_by_frequency[target_hz]
            error_deg = float(row["measured_error_deg"])
            resultant = max(0.1, float(row.get("resultant", 1.0)))
            coefficients = {target_index: 1.0, fundamental_index: -float(harmonic)}
            add_equation(coefficients, error_deg, resultant * resultant)
            observations.append((fundamental_index, target_index, harmonic, error_deg))

    if not observations:
        raise RuntimeError("no phase observations match the requested grid")

    add_equation({0: 1.0}, 0.0, args.anchor_weight)
    for index in range(1, size - 1):
        add_equation(
            {index - 1: 1.0, index: -2.0, index + 1: 1.0},
            0.0,
            args.smooth_weight,
        )

    phase_degrees = solve_linear(normal, rhs)
    residuals = [
        wrap_degrees(
            phase_degrees[target] -
            harmonic * phase_degrees[fundamental] -
            measured
        )
        for fundamental, target, harmonic, measured in observations
    ]
    rms = math.sqrt(statistics.fmean(value * value for value in residuals))
    maximum = max(abs(value) for value in residuals)

    lines = [
        "static const GMeasurementPhaseCalibrationPoint s_PhaseCalibrationPoints[] =",
        "{",
    ]
    for frequency_hz, phase_deg in zip(frequencies, phase_degrees):
        lines.append(
            f"    {{ {frequency_hz:9.1f}f, {math.radians(phase_deg): .9f}f }},"
            f"  // {phase_deg:+.5f} deg"
        )
    lines.extend(
        [
            "};",
            "",
            f"/* observation residual: RMS={rms:.6f} deg, max={maximum:.6f} deg */",
        ]
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"phase table: {output}")
    print(f"observation residual: RMS={rms:.6f} deg, max={maximum:.6f} deg")
    if maximum > args.max_residual:
        ranked = sorted(
            zip(residuals, observations),
            key=lambda item: abs(item[0]),
            reverse=True,
        )
        for residual, (fundamental, target, harmonic, _measured) in ranked[:10]:
            print(
                "residual f0={:.0f}Hz h={} target={:.0f}Hz: {:+.6f} deg".format(
                    frequencies[fundamental],
                    harmonic,
                    frequencies[target],
                    residual,
                ),
                file=sys.stderr,
            )
        print(
            f"FAIL: max phase residual exceeds {args.max_residual:.3f} deg",
            file=sys.stderr,
        )
        return 2
    return 0


def self_test(_args) -> int:
    assert parse_series("10:30:10,45") == [10.0, 20.0, 30.0, 45.0]
    solution = solve_linear([[2.0, 1.0], [1.0, 3.0]], [5.0, 6.0])
    assert abs(solution[0] - 1.8) < 1e-9
    assert abs(solution[1] - 1.4) < 1e-9
    print("ok")
    return 0


def add_common_sweep_arguments(parser) -> None:
    parser.add_argument("--generator", default="169.254.239.82")
    parser.add_argument(
        "--generator-kind",
        choices=("auto", "rigol", "siglent"),
        default="auto",
    )
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--frames", type=int, default=25)
    parser.add_argument("--settle", type=float, default=0.8)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--no-restore", action="store_true")
    parser.add_argument("--resume", action="store_true")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    amplitude = subparsers.add_parser("amplitude-sweep")
    add_common_sweep_arguments(amplitude)
    amplitude.add_argument("--frequencies", default="10000:500000:10000")
    amplitude.add_argument("--vpps", default="50,100,150,200,250")
    amplitude.add_argument("--output", default="CalibrationData/amplitude_raw.csv")
    amplitude.add_argument("--c-output", default="CalibrationData/amplitude_table.inc")
    amplitude.set_defaults(func=amplitude_sweep)

    build_amplitude = subparsers.add_parser("build-amplitude-table")
    build_amplitude.add_argument("--input", default="CalibrationData/amplitude_raw.csv")
    build_amplitude.add_argument("--output", default="CalibrationData/amplitude_table.inc")
    build_amplitude.set_defaults(func=build_amplitude_table)

    phase = subparsers.add_parser("phase-sweep")
    add_common_sweep_arguments(phase)
    phase.add_argument("--fundamentals", default="10000:250000:10000")
    phase.add_argument("--orders", default="2:10:1")
    phase.add_argument("--fundamental-vpp", type=float, default=80.0)
    phase.add_argument("--harmonic-vpp", type=float, default=50.0)
    phase.add_argument("--fundamental-phase", type=float, default=10.0)
    phase.add_argument("--output", default="CalibrationData/phase_observations.csv")
    phase.set_defaults(func=phase_sweep)

    build_phase = subparsers.add_parser("build-phase-table")
    build_phase.add_argument("--input", default="CalibrationData/phase_observations.csv")
    build_phase.add_argument("--grid", default="10000:500000:10000")
    build_phase.add_argument("--output", default="CalibrationData/phase_table.inc")
    build_phase.add_argument("--smooth-weight", type=float, default=0.05)
    build_phase.add_argument("--anchor-weight", type=float, default=1000.0)
    build_phase.add_argument("--max-residual", type=float, default=0.2)
    build_phase.set_defaults(func=build_phase_table)

    test = subparsers.add_parser("self-test")
    test.set_defaults(func=self_test)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ConnectionError, OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1)
