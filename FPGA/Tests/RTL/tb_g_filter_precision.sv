`timescale 1ns/1ps

module tb_g_filter_precision;

localparam real PI = 3.14159265358979323846;
localparam real INPUT_SAMPLE_RATE_HZ = 50000000.0;

reg clk = 1'b0;
reg rst_n = 1'b0;
reg signed [12:0] sample_in = 13'sd0;
integer failures = 0;

wire signed [17:0] cic_sample;
wire cic_valid;
wire signed [17:0] cic_sample_q4 = cic_sample <<< 4;
wire signed [17:0] filtered_sample;
wire filtered_valid;

always #10 clk = ~clk;

g_cic3_decimator cic_inst(
    .clk(clk),
    .rst_n(rst_n),
    .sample_in(sample_in),
    .sample_out(cic_sample),
    .sample_valid(cic_valid)
);

g_sos3_lowpass lowpass_inst(
    .clk(clk),
    .rst_n(rst_n),
    .sample_in(cic_sample_q4),
    .sample_valid(cic_valid),
    .sample_out(filtered_sample),
    .output_valid(filtered_valid)
);

task automatic run_tone;
    input real frequency_hz;
    input integer amplitude_codes;
    integer input_index;
    integer output_count;
    integer collect_count;
    integer minimum;
    integer maximum;
    real sum_square;
    real phase;
    real rms;
    begin
        rst_n = 1'b0;
        sample_in = 13'sd0;
        repeat (12) @(negedge clk);
        rst_n = 1'b1;

        input_index = 0;
        output_count = 0;
        collect_count = 0;
        minimum = 131071;
        maximum = -131072;
        sum_square = 0.0;

        while (collect_count < 4096) begin
            @(negedge clk);
            phase = 2.0 * PI * frequency_hz *
                    input_index / INPUT_SAMPLE_RATE_HZ;
            sample_in = $rtoi(amplitude_codes * $sin(phase));
            input_index = input_index + 1;

            if (filtered_valid) begin
                output_count = output_count + 1;
                if (output_count > 512) begin
                    if ($signed(filtered_sample) < minimum)
                        minimum = $signed(filtered_sample);
                    if ($signed(filtered_sample) > maximum)
                        maximum = $signed(filtered_sample);
                    sum_square = sum_square +
                                 $itor($signed(filtered_sample)) *
                                 $itor($signed(filtered_sample));
                    collect_count = collect_count + 1;
                end
            end
        end

        rms = $sqrt(sum_square / collect_count);
        $display("RTL_TONE f=%0.0fHz in_peak=%0d out_min=%0d out_max=%0d out_vpp=%0d out_rms=%0.4f",
                 frequency_hz,
                 amplitude_codes,
                 minimum,
                 maximum,
                 maximum - minimum,
                 rms);

        if ((frequency_hz == 100000.0) &&
            (amplitude_codes == 1000) &&
            ((rms < 10000.0) || (rms > 12000.0))) begin
            $display("FAIL: 100kHz passband gain out of range");
            failures = failures + 1;
        end

        if ((frequency_hz == 500000.0) &&
            (amplitude_codes == 1000) &&
            ((rms < 9500.0) || (rms > 11500.0))) begin
            $display("FAIL: 500kHz passband gain out of range");
            failures = failures + 1;
        end

        if ((frequency_hz == 1000000.0) && (rms > 20.0)) begin
            $display("FAIL: 1MHz residual exceeds Q4 target");
            failures = failures + 1;
        end

        if ((amplitude_codes == 2047) &&
            ((minimum < -32768) || (maximum > 32767))) begin
            $display("FAIL: full-scale passband tone exceeds RAM range");
            failures = failures + 1;
        end
    end
endtask

initial begin
    run_tone(100000.0, 1000);
    run_tone(500000.0, 1000);
    run_tone(1000000.0, 1000);
    run_tone(100000.0, 2047);
    run_tone(500000.0, 2047);

    if (failures != 0)
        $fatal(1, "g_filter_precision failures=%0d", failures);

    $display("PASS: g_filter_precision");
    $finish;
end

endmodule
