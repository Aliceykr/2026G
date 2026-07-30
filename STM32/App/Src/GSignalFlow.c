#include "GSignalFlow.h"

#include "FpgaLink.h"
#include "GMeasurementCalibration.h"
#include "SpectrumAnalyzer.h"
#include "GSerial.h"
#include "tjc_usart_hmi.h"
#include "stm32f4xx_hal.h"

#include <stdio.h>
#include <string.h>

#define G_FLOW_STATUS_POLL_MS       2U
#define G_FLOW_UART_TIMEOUT_MS      50U
#define G_FLOW_RESPONSE_LIMIT_MS    2000U
#define G_FLOW_ASCII_DEBUG_ENABLED   1U
#define G_FLOW_HMI_IDLE_REFRESH_MS  2000U
#define G_FLOW_HMI_MEASUREMENT_FIELDS 9U
#define G_FLOW_HMI_BUSY_MIN_MS        300U
#define G_FLOW_CAPTURE_TIMEOUT_MS     2000U
#define G_FLOW_WAVE_HEIGHT_DIVISOR    3U
#define G_FLOW_WAVE_VERTICAL_OFFSET   20U
#define G_FLOW_WAVE_FALLBACK_POINTS   512U
#define G_FLOW_WAVE_MAX_POINTS        1024U
#define G_FLOW_SPECTRUM_HEIGHT_DIVISOR 3U
#define G_FLOW_SPECTRUM_FALLBACK_POINTS 512U
#define G_FLOW_SPECTRUM_MAX_POINTS    1024U
#define G_FLOW_BUTTON_DEBOUNCE_MS     120U

typedef enum
{
    G_FLOW_STATE_IDLE = 0,
    G_FLOW_STATE_WAIT_ANALYSIS,
    G_FLOW_STATE_WAIT_WAVEFORM,
    G_FLOW_STATE_WAIT_RESTART,
    G_FLOW_STATE_HOLD
} GFlowState;

static int16_t s_CaptureFrame[SPECTRUM_FRAME_LENGTH];
static SpectrumResult s_Result;
static GMeasurementResult s_Measurement;
static GMeasurementWaveform s_Waveform;
static GFlowState s_State;
static uint32_t s_NextActionTick;
static uint32_t s_ReportSequence;
static uint32_t s_ActiveSequence;
static uint32_t s_CycleStartTick;
static uint32_t s_AnalysisElapsedMs;
static uint32_t s_FpgaId;
static uint32_t s_NextIdleHmiTextTick;
static uint8_t s_FpgaOnline;
static uint8_t s_SelectedWaveformCycles;
static uint8_t s_ActiveWaveformCycles;
static uint8_t s_CycleChangePending;
static uint8_t s_WaveFrameValid;
static uint8_t s_MeasurementEnabled;
static uint8_t s_HmiMeasurementField;
static uint8_t s_HmiMeasurementPending;
static uint8_t s_HmiReadyPending;
static uint32_t s_HmiBusyUntilTick;
static uint32_t s_LastStartEventTick;
static uint32_t s_LastCycleEventTick;
static uint16_t s_WaveDisplayPoints;
static uint8_t s_WaveDisplay[G_FLOW_WAVE_MAX_POINTS];
static uint16_t s_SpectrumDisplayPoints;
static uint8_t s_SpectrumDisplay[G_FLOW_SPECTRUM_MAX_POINTS];
static uint8_t s_SpectrumScreenInitialized;

static void GSignalFlow_SendText(const char *text);
static void GSignalFlow_SendStartup(void);
static void GSignalFlow_SendError(const char *stage);
static void GSignalFlow_SendNoSignal(void);
static void GSignalFlow_SendResult(void);
static uint8_t GSignalFlow_SendMeasurement(void);
static uint32_t GSignalFlow_RoundPositive(float value);
static void GSignalFlow_StartOrRetry(uint32_t now);
static void GSignalFlow_FinishCycle(uint32_t now);
static void GSignalFlow_AbortCycle(const char *stage,
                                  const char *hmi_status);
static void GSignalFlow_HandleCommand(void);
static void GSignalFlow_UpdateHmiPlaceholders(void);
static void GSignalFlow_QueueHmiMeasurement(void);
static void GSignalFlow_ServiceHmiMeasurement(void);
static void GSignalFlow_UpdateHmiMeasurementField(uint8_t field);
static void GSignalFlow_UpdateHmiNoSignal(void);
static uint8_t GSignalFlow_UpdateHmiSpectrum(uint8_t signal_valid);
static void GSignalFlow_UpdateHmiWaveform(void);
static uint8_t GSignalFlow_BuildAndDisplayWaveform(uint8_t cycles);

void GSignalFlow_Init(void)
{
    memset(s_CaptureFrame, 0, sizeof(s_CaptureFrame));
    memset(&s_Result, 0, sizeof(s_Result));
    memset(&s_Measurement, 0, sizeof(s_Measurement));
    memset(&s_Waveform, 0, sizeof(s_Waveform));
    memset(s_WaveDisplay, 0, sizeof(s_WaveDisplay));
    memset(s_SpectrumDisplay, 0, sizeof(s_SpectrumDisplay));
    s_State = G_FLOW_STATE_IDLE;
    s_NextActionTick = HAL_GetTick();
    s_ReportSequence = 0UL;
    s_ActiveSequence = 0UL;
    s_CycleStartTick = HAL_GetTick();
    s_AnalysisElapsedMs = 0UL;
    s_FpgaId = 0UL;
    s_NextIdleHmiTextTick = HAL_GetTick();
    s_SelectedWaveformCycles = 1U;
    s_ActiveWaveformCycles = 1U;
    s_CycleChangePending = 0U;
    s_WaveFrameValid = 0U;
    s_MeasurementEnabled = 0U;
    s_HmiMeasurementField = 0U;
    s_HmiMeasurementPending = 0U;
    s_HmiReadyPending = 0U;
    s_HmiBusyUntilTick = 0UL;
    s_LastStartEventTick = HAL_GetTick() - G_FLOW_BUTTON_DEBOUNCE_MS;
    s_LastCycleEventTick = HAL_GetTick() - G_FLOW_BUTTON_DEBOUNCE_MS;
    s_WaveDisplayPoints = 0U;
    s_SpectrumDisplayPoints = 0U;
    s_SpectrumScreenInitialized = 0U;

    TjcHmi_Init();

    FpgaLink_Init();
    s_FpgaOnline = Fpga_ReadId(&s_FpgaId);
    GSignalFlow_SendStartup();
}

void GSignalFlow_Process(void)
{
    uint32_t now = HAL_GetTick();

    GSignalFlow_HandleCommand();
    GSignalFlow_ServiceHmiMeasurement();

    if (s_State == G_FLOW_STATE_IDLE)
    {
        /*
         * 屏幕上电通常慢于MCU。开始测量前每2秒重发一次XXX占位文字，
         * 避免第一次命令发送过早后仍显示工程里的默认newtxt。
         */
        if ((int32_t)(now - s_NextIdleHmiTextTick) >= 0)
        {
            GSignalFlow_UpdateHmiPlaceholders();
            TjcHmi_SetComputeBusy(0U);
            if (s_SpectrumScreenInitialized == 0U)
            {
                (void)GSignalFlow_UpdateHmiSpectrum(0U);
            }
            s_NextIdleHmiTextTick = now + G_FLOW_HMI_IDLE_REFRESH_MS;
        }
        return;
    }

    /* 单次测量完成后保持本轮结果，直到再次按下“开始测量”。 */
    if (s_State == G_FLOW_STATE_HOLD)
    {
        return;
    }

    if ((int32_t)(now - s_NextActionTick) < 0)
    {
        return;
    }

    if (s_State == G_FLOW_STATE_WAIT_RESTART)
    {
        GSignalFlow_StartOrRetry(now);
        return;
    }

    {
        uint32_t status = 0UL;

        if (Fpga_ReadCaptureStatus(&status) == 0U)
        {
            GSignalFlow_AbortCycle("status", "STAT ERR");
            return;
        }

        if ((status & FPGA_CAPTURE_STATUS_DONE) == 0UL)
        {
            if ((uint32_t)(now - s_CycleStartTick) >=
                G_FLOW_CAPTURE_TIMEOUT_MS)
            {
                GSignalFlow_AbortCycle("timeout", "TIMEOUT");
                return;
            }
            s_NextActionTick = now + G_FLOW_STATUS_POLL_MS;
            return;
        }

        if ((uint16_t)(status >> 16U) != FPGA_CAPTURE_FRAME_LENGTH)
        {
            GSignalFlow_AbortCycle("length", "LEN ERR");
            return;
        }
    }

    if (Fpga_ReadCaptureFrame(s_CaptureFrame, SPECTRUM_FRAME_LENGTH) == 0U)
    {
        GSignalFlow_AbortCycle("frame", "FRAME ERR");
        return;
    }

    if (s_State == G_FLOW_STATE_WAIT_ANALYSIS)
    {
        if (SpectrumAnalyzer_Run(s_CaptureFrame, &s_Result) == 0U)
        {
            if (s_Result.status == SPECTRUM_STATUS_NO_SIGNAL)
            {
                s_ActiveSequence = s_ReportSequence;
                s_ReportSequence++;
                s_AnalysisElapsedMs = HAL_GetTick() - s_CycleStartTick;
                GSignalFlow_SendNoSignal();
                GSignalFlow_UpdateHmiNoSignal();
            }
            else
            {
                GSignalFlow_AbortCycle("spectrum", "CALC ERR");
                return;
            }
            GSignalFlow_FinishCycle(HAL_GetTick());
            return;
        }

        s_ActiveSequence = s_ReportSequence;
        s_ReportSequence++;
        s_AnalysisElapsedMs = HAL_GetTick() - s_CycleStartTick;
        (void)GSignalFlow_UpdateHmiSpectrum(1U);
        GSignalFlow_SendResult();
        if (GSignalFlow_SendMeasurement() == 0U)
        {
            GSignalFlow_AbortCycle("measurement", "CALC ERR");
            return;
        }

        if (Fpga_StartCapture(1U) == 0U)
        {
            GSignalFlow_AbortCycle("wave_start", "WAVE ERR");
            return;
        }

        s_State = G_FLOW_STATE_WAIT_WAVEFORM;
        s_NextActionTick = HAL_GetTick() + G_FLOW_STATUS_POLL_MS;
        return;
    }

    if (s_State == G_FLOW_STATE_WAIT_WAVEFORM)
    {
        s_WaveFrameValid = 1U;
        if (s_CycleChangePending != 0U)
        {
            s_ActiveWaveformCycles = s_SelectedWaveformCycles;
            s_CycleChangePending = 0U;
        }

        if (GSignalFlow_BuildAndDisplayWaveform(
                s_ActiveWaveformCycles) == 0U)
        {
            GSignalFlow_AbortCycle("waveform", "WAVE ERR");
            return;
        }

        GSignalFlow_UpdateHmiWaveform();
        GSignalFlow_FinishCycle(HAL_GetTick());
        return;
    }
}

static void GSignalFlow_StartOrRetry(uint32_t now)
{
    /* 没有收到明确的开始按键事件时，任何路径都不得启动FPGA采集。 */
    if (s_MeasurementEnabled == 0U)
    {
        s_State = G_FLOW_STATE_IDLE;
        return;
    }

    if (s_FpgaOnline == 0U)
    {
        s_FpgaOnline = Fpga_ReadId(&s_FpgaId);
        if (s_FpgaOnline == 0U)
        {
            GSignalFlow_AbortCycle("id", "ID ERR");
            return;
        }
    }

    if (Fpga_StartCapture(0U) == 0U)
    {
        GSignalFlow_AbortCycle("start", "START ERR");
        return;
    }

    memset(&s_Result, 0, sizeof(s_Result));
    memset(&s_Measurement, 0, sizeof(s_Measurement));
    memset(&s_Waveform, 0, sizeof(s_Waveform));
    s_WaveFrameValid = 0U;
    s_CycleStartTick = now;
    s_State = G_FLOW_STATE_WAIT_ANALYSIS;
    s_NextActionTick = now + G_FLOW_STATUS_POLL_MS;
}

static void GSignalFlow_FinishCycle(uint32_t now)
{
    (void)now;
    s_MeasurementEnabled = 0U;
    s_State = G_FLOW_STATE_HOLD;
    s_HmiReadyPending = 1U;
}

static void GSignalFlow_AbortCycle(const char *stage,
                                  const char *hmi_status)
{
    GSignalFlow_SendError(stage);
    s_MeasurementEnabled = 0U;
    s_HmiMeasurementPending = 0U;
    s_HmiReadyPending = 0U;
    s_State = G_FLOW_STATE_HOLD;
    TjcHmi_SetStatusText(hmi_status);
}

static void GSignalFlow_HandleCommand(void)
{
    TjcHmiEvent event;
    uint8_t requested_cycles = 0U;
    uint32_t now;

    if (TjcHmi_ReadEvent(&event, &requested_cycles) == 0U)
    {
        return;
    }

    now = HAL_GetTick();

    if (event == TJC_HMI_EVENT_START_MEASUREMENT)
    {
        if ((uint32_t)(now - s_LastStartEventTick) <
            G_FLOW_BUTTON_DEBOUNCE_MS)
        {
            return;
        }
        s_LastStartEventTick = now;

        /* 测量中重复开始直接忽略，不重置状态、不重启FPGA采集。 */
        if ((s_State != G_FLOW_STATE_IDLE) &&
            (s_State != G_FLOW_STATE_HOLD))
        {
            GSignalFlow_SendText("G_HMI,event=start,ignored=busy\r\n");
            return;
        }

        GSignalFlow_SendText("G_HMI,event=start\r\n");
        TjcHmi_SetComputeBusy(1U);
        s_HmiBusyUntilTick = now + G_FLOW_HMI_BUSY_MIN_MS;
        s_HmiReadyPending = 1U;
        s_MeasurementEnabled = 1U;
        s_HmiMeasurementPending = 0U;
        s_ActiveWaveformCycles = s_SelectedWaveformCycles;
        s_CycleChangePending = 0U;
        s_WaveFrameValid = 0U;
        s_State = G_FLOW_STATE_WAIT_RESTART;
        s_NextActionTick = now;
        return;
    }

    if (event == TJC_HMI_EVENT_TOGGLE_CYCLES)
    {
        requested_cycles =
            (s_SelectedWaveformCycles == 1U) ? 3U : 1U;
    }

    if ((event == TJC_HMI_EVENT_TOGGLE_CYCLES) ||
        (event == TJC_HMI_EVENT_SET_CYCLES))
    {
        char buffer[64];
        const char *apply_mode;

        if ((uint32_t)(now - s_LastCycleEventTick) <
            G_FLOW_BUTTON_DEBOUNCE_MS)
        {
            return;
        }
        s_LastCycleEventTick = now;

        if (GSignalFlow_SetWaveformCycles(requested_cycles) == 0U)
        {
            return;
        }

        if ((s_State == G_FLOW_STATE_WAIT_RESTART) ||
            (s_State == G_FLOW_STATE_WAIT_ANALYSIS) ||
            (s_State == G_FLOW_STATE_WAIT_WAVEFORM))
        {
            /* 本轮采集不中断，在波形生成前一次性提交最新周期选择。 */
            s_CycleChangePending = 1U;
            apply_mode = "queued";
        }
        else if ((s_State == G_FLOW_STATE_HOLD) &&
                 (s_WaveFrameValid != 0U))
        {
            /* 使用最近一次5MSPS帧重画，不重新启动FPGA。 */
            s_ActiveWaveformCycles = s_SelectedWaveformCycles;
            TjcHmi_SetComputeBusy(1U);
            if (GSignalFlow_BuildAndDisplayWaveform(
                    s_ActiveWaveformCycles) != 0U)
            {
                TjcHmi_SetComputeBusy(0U);
                apply_mode = "redraw";
            }
            else
            {
                TjcHmi_SetStatusText("WAVE ERR");
                apply_mode = "error";
            }
        }
        else
        {
            s_ActiveWaveformCycles = s_SelectedWaveformCycles;
            apply_mode = "next";
        }

        (void)snprintf(buffer,
                       sizeof(buffer),
                       "G_HMI,event=cycles,value=%u,apply=%s\r\n",
                       (unsigned int)s_SelectedWaveformCycles,
                       apply_mode);
        GSignalFlow_SendText(buffer);
    }
}

static void GSignalFlow_SendText(const char *text)
{
#if G_FLOW_ASCII_DEBUG_ENABLED != 0U
    size_t length;

    if (text == NULL)
    {
        return;
    }

    length = strlen(text);
    if (length > 0xFFFFU)
    {
        length = 0xFFFFU;
    }

    (void)GSerial_Transmit((const uint8_t *)text,
                           (uint16_t)length,
                           G_FLOW_UART_TIMEOUT_MS);
#else
    (void)text;
#endif
}

static void GSignalFlow_SendStartup(void)
{
    char buffer[96];

    (void)snprintf(buffer,
                   sizeof(buffer),
                   "G_FLOW,boot,id=0x%08lX,online=%u,fs=1250000,n=4096\r\n",
                   (unsigned long)s_FpgaId,
                   (unsigned int)s_FpgaOnline);
    GSignalFlow_SendText(buffer);
}

static void GSignalFlow_SendError(const char *stage)
{
    char buffer[96];

    (void)snprintf(buffer,
                   sizeof(buffer),
                   "G_FLOW,error=%s,seq=%lu\r\n",
                   stage,
                   (unsigned long)s_ReportSequence);
    GSignalFlow_SendText(buffer);
}

static void GSignalFlow_SendNoSignal(void)
{
    char buffer[128];

    (void)snprintf(
        buffer,
        sizeof(buffer),
        "G_FLOW,no_signal,seq=%lu,t_ms=%lu,under2s=%u,threshold=%u\r\n",
        (unsigned long)s_ActiveSequence,
        (unsigned long)s_AnalysisElapsedMs,
        (unsigned int)((s_AnalysisElapsedMs <=
                        G_FLOW_RESPONSE_LIMIT_MS) ? 1U : 0U),
        (unsigned int)SPECTRUM_MIN_VALID_AMPLITUDE_CODES);
    GSignalFlow_SendText(buffer);
}

static void GSignalFlow_SendResult(void)
{
    char buffer[256];
    int written;
    uint8_t component;

    written = snprintf(buffer,
                       sizeof(buffer),
                       "G_FLOW,seq=%lu,f0=%luHz,n=%u,rms=%lu,vpp=%lu,t_ms=%lu,under2s=%u",
                       (unsigned long)s_ActiveSequence,
                       (unsigned long)GSignalFlow_RoundPositive(s_Result.fundamental_hz),
                       (unsigned int)s_Result.component_count,
                       (unsigned long)GSignalFlow_RoundPositive(s_Result.rms_codes),
                       (unsigned long)GSignalFlow_RoundPositive(s_Result.vpp_codes),
                       (unsigned long)s_AnalysisElapsedMs,
                       (unsigned int)((s_AnalysisElapsedMs <=
                                       G_FLOW_RESPONSE_LIMIT_MS) ? 1U : 0U));

    if (written < 0)
    {
        return;
    }

    for (component = 0U;
         (component < s_Result.component_count) &&
         ((size_t)written < sizeof(buffer));
         component++)
    {
        const SpectrumComponent *item = &s_Result.components[component];
        int appended = snprintf(&buffer[written],
                                sizeof(buffer) - (size_t)written,
                                ",h%u=%luHz:%lu",
                                (unsigned int)item->harmonic,
                                (unsigned long)GSignalFlow_RoundPositive(item->frequency_hz),
                                (unsigned long)GSignalFlow_RoundPositive(item->amplitude_codes));

        if (appended < 0)
        {
            break;
        }

        if ((size_t)appended >= sizeof(buffer) - (size_t)written)
        {
            written = (int)(sizeof(buffer) - 1U);
            break;
        }

        written += appended;
    }

    if ((size_t)written < sizeof(buffer))
    {
        int appended;

        if (s_Result.spur_valid != 0U)
        {
            appended = snprintf(&buffer[written],
                                sizeof(buffer) - (size_t)written,
                                ",spur=%luHz:%lu",
                                (unsigned long)GSignalFlow_RoundPositive(
                                    s_Result.spur_frequency_hz),
                                (unsigned long)GSignalFlow_RoundPositive(
                                    s_Result.spur_amplitude_codes));
        }
        else
        {
            appended = snprintf(&buffer[written],
                                sizeof(buffer) - (size_t)written,
                                ",spur=none");
        }

        if (appended >= 0)
        {
            if ((size_t)appended >=
                sizeof(buffer) - (size_t)written)
            {
                written = (int)(sizeof(buffer) - 1U);
            }
            else
            {
                written += appended;
            }
        }
    }

    if ((size_t)written + 2U < sizeof(buffer))
    {
        buffer[written++] = '\r';
        buffer[written++] = '\n';
        buffer[written] = '\0';
    }
    else
    {
        buffer[sizeof(buffer) - 3U] = '\r';
        buffer[sizeof(buffer) - 2U] = '\n';
        buffer[sizeof(buffer) - 1U] = '\0';
    }

    GSignalFlow_SendText(buffer);
}

static uint8_t GSignalFlow_SendMeasurement(void)
{
    char buffer[256];
    int written;
    uint8_t component;

    if (GMeasurement_Convert(&s_Result,
                             GMeasurementCalibration_Get(),
                             &s_Measurement) == 0U)
    {
        (void)snprintf(buffer,
                       sizeof(buffer),
                       "G_MEAS,seq=%lu,cal=0,reason=no_mv_table\r\n",
                       (unsigned long)s_ActiveSequence);
        GSignalFlow_SendText(buffer);
        return 0U;
    }

    GSignalFlow_QueueHmiMeasurement();

    written = snprintf(
        buffer,
        sizeof(buffer),
        "G_MEAS,seq=%lu,cal=1,f0=%luHz,upp=%lumV,urms=%lumV,n=%u",
        (unsigned long)s_ActiveSequence,
        (unsigned long)GSignalFlow_RoundPositive(
            s_Measurement.fundamental_hz),
        (unsigned long)GSignalFlow_RoundPositive(s_Measurement.upp_mv),
        (unsigned long)GSignalFlow_RoundPositive(s_Measurement.urms_mv),
        (unsigned int)s_Measurement.component_count);

    if (written < 0)
    {
        return 0U;
    }

    for (component = 0U;
         (component < s_Measurement.component_count) &&
         ((size_t)written < sizeof(buffer));
         component++)
    {
        const GMeasurementComponent *item =
            &s_Measurement.components[component];
        int appended = snprintf(
            &buffer[written],
            sizeof(buffer) - (size_t)written,
            ",h%u=%luHz:%lumV",
            (unsigned int)item->harmonic,
            (unsigned long)GSignalFlow_RoundPositive(item->frequency_hz),
            (unsigned long)GSignalFlow_RoundPositive(item->amplitude_mv));

        if (appended < 0)
        {
            break;
        }
        if ((size_t)appended >= sizeof(buffer) - (size_t)written)
        {
            written = (int)(sizeof(buffer) - 1U);
            break;
        }
        written += appended;
    }

    if ((size_t)written + 2U < sizeof(buffer))
    {
        buffer[written++] = '\r';
        buffer[written++] = '\n';
        buffer[written] = '\0';
    }
    else
    {
        buffer[sizeof(buffer) - 3U] = '\r';
        buffer[sizeof(buffer) - 2U] = '\n';
        buffer[sizeof(buffer) - 1U] = '\0';
    }

    GSignalFlow_SendText(buffer);
    return 1U;
}

static void GSignalFlow_QueueHmiMeasurement(void)
{
    /* 三个主要参数先立即显示，确保后续波形采集不会拖住Urms。 */
    GSignalFlow_UpdateHmiMeasurementField(0U);
    GSignalFlow_UpdateHmiMeasurementField(1U);
    GSignalFlow_UpdateHmiMeasurementField(2U);
    s_HmiMeasurementField = 3U;
    s_HmiMeasurementPending = 1U;
}

/* 每次主循环最多更新一个文本控件，避免参数集中成批发送。 */
static void GSignalFlow_ServiceHmiMeasurement(void)
{
    if (s_HmiMeasurementPending != 0U)
    {
        GSignalFlow_UpdateHmiMeasurementField(s_HmiMeasurementField);
        s_HmiMeasurementField++;

        if (s_HmiMeasurementField >= G_FLOW_HMI_MEASUREMENT_FIELDS)
        {
            s_HmiMeasurementPending = 0U;
        }
        return;
    }

    if ((s_HmiReadyPending != 0U) &&
        (s_State == G_FLOW_STATE_HOLD) &&
        ((int32_t)(HAL_GetTick() - s_HmiBusyUntilTick) >= 0))
    {
        TjcHmi_SetComputeBusy(0U);
        s_HmiReadyPending = 0U;
    }
}

static void GSignalFlow_UpdateHmiMeasurementField(uint8_t field)
{
    char text[40];
    uint8_t component;
    const char *object_name;

    if (field == 0U)
    {
        (void)snprintf(text,
                       sizeof(text),
                       "f0=%lu Hz",
                       (unsigned long)GSignalFlow_RoundPositive(
                           s_Measurement.fundamental_hz));
        tjc_send_txt("t0", "txt", text);
        return;
    }
    if (field == 1U)
    {
        (void)snprintf(text,
                       sizeof(text),
                       "Vpp=%lu mV",
                       (unsigned long)GSignalFlow_RoundPositive(
                           s_Measurement.upp_mv));
        tjc_send_txt("t1", "txt", text);
        return;
    }
    if (field == 2U)
    {
        (void)snprintf(text,
                       sizeof(text),
                       "Urms=%lu mV",
                       (unsigned long)GSignalFlow_RoundPositive(
                           s_Measurement.urms_mv));
        tjc_send_txt("t2", "txt", text);
        return;
    }

    if (field <= 5U)
    {
        component = (uint8_t)(field - 3U);
        object_name = (field == 3U) ? "t3" :
                      ((field == 4U) ? "t4" : "t5");

        if (component < s_Measurement.component_count)
        {
            const GMeasurementComponent *item =
                &s_Measurement.components[component];
            (void)snprintf(text,
                           sizeof(text),
                           "C%u=H%u,%lu Hz,%lu mV",
                           (unsigned int)(component + 1U),
                           (unsigned int)item->harmonic,
                           (unsigned long)GSignalFlow_RoundPositive(
                               item->frequency_hz),
                           (unsigned long)GSignalFlow_RoundPositive(
                               item->amplitude_mv));
        }
        else
        {
            (void)snprintf(text,
                           sizeof(text),
                           "C%u=XXX",
                           (unsigned int)(component + 1U));
        }
        tjc_send_txt(object_name, "txt", text);
        return;
    }

    if (field < G_FLOW_HMI_MEASUREMENT_FIELDS)
    {
        uint8_t harmonic = (uint8_t)(field - 5U);
        const GMeasurementComponent *matched = NULL;
        char harmonic_object[3] =
            {'t', (char)('5' + harmonic), '\0'};

        for (component = 0U;
             component < s_Measurement.component_count;
             component++)
        {
            if (s_Measurement.components[component].harmonic == harmonic)
            {
                matched = &s_Measurement.components[component];
                break;
            }
        }

        if (matched != NULL)
        {
            (void)snprintf(text,
                           sizeof(text),
                           "H%u=%lu mV",
                           (unsigned int)harmonic,
                           (unsigned long)GSignalFlow_RoundPositive(
                               matched->amplitude_mv));
        }
        else
        {
            (void)snprintf(text,
                           sizeof(text),
                           "H%u=XXX mV",
                           (unsigned int)harmonic);
        }
        tjc_send_txt(harmonic_object, "txt", text);
    }
}

static void GSignalFlow_UpdateHmiPlaceholders(void)
{
    uint8_t field;
    uint8_t harmonic;

    tjc_send_txt("t0", "txt", "f0=XXX Hz");
    tjc_send_txt("t1", "txt", "Vpp=XXX mV");
    tjc_send_txt("t2", "txt", "Urms=XXX mV");
    for (field = 3U; field <= 5U; field++)
    {
        char object_name[3] = {'t', (char)('0' + field), '\0'};
        char text[12];

        (void)snprintf(text,
                       sizeof(text),
                       "C%u=XXX",
                       (unsigned int)(field - 2U));
        tjc_send_txt(object_name, "txt", text);
    }
    for (harmonic = 1U; harmonic <= 3U; harmonic++)
    {
        char object_name[3] =
            {'t', (char)('5' + harmonic), '\0'};
        char text[20];

        (void)snprintf(text,
                       sizeof(text),
                       "H%u=XXX mV",
                       (unsigned int)harmonic);
        tjc_send_txt(object_name, "txt", text);
    }
}

static void GSignalFlow_UpdateHmiNoSignal(void)
{
    s_HmiMeasurementPending = 0U;
    GSignalFlow_UpdateHmiPlaceholders();
    (void)GSignalFlow_UpdateHmiSpectrum(0U);
}

static uint8_t GSignalFlow_UpdateHmiSpectrum(uint8_t signal_valid)
{
    uint16_t point;
    uint16_t display_points = s_SpectrumDisplayPoints;
    uint8_t width_ready = 1U;
    uint8_t axis_ready;

    if (display_points == 0U)
    {
        if (TjcHmi_GetComponentWidth("s1", &display_points) != 0U)
        {
            s_SpectrumDisplayPoints = display_points;
        }
        else
        {
            display_points = G_FLOW_SPECTRUM_FALLBACK_POINTS;
            width_ready = 0U;
        }
    }

    if (display_points > G_FLOW_SPECTRUM_MAX_POINTS)
    {
        display_points = G_FLOW_SPECTRUM_MAX_POINTS;
    }

    if ((signal_valid == 0U) ||
        (SpectrumAnalyzer_BuildDisplay(s_SpectrumDisplay,
                                       display_points) == 0U))
    {
        memset(s_SpectrumDisplay, 0, display_points);
    }
    else
    {
        /*
         * BuildDisplay按10kHz->500kHz的FFT bin递增顺序输出。
         * 仅压缩纵坐标上限，0下限保持不变。
         */
        for (point = 0U; point < display_points; point++)
        {
            s_SpectrumDisplay[point] = (uint8_t)(
                s_SpectrumDisplay[point] /
                G_FLOW_SPECTRUM_HEIGHT_DIVISOR);
        }
    }

    tjc_clear_wave("s1.id", 0);
    tjc_send_wave("s1.id",
                  0,
                  s_SpectrumDisplay,
                  display_points);

    /* 频谱为空时全零谱线本身也保留底部横轴。 */
    axis_ready = TjcHmi_DrawSpectrumXAxis();
    if ((width_ready != 0U) && (axis_ready != 0U))
    {
        s_SpectrumScreenInitialized = 1U;
        return 1U;
    }

    s_SpectrumScreenInitialized = 0U;
    return 0U;
}

static uint8_t GSignalFlow_BuildAndDisplayWaveform(uint8_t cycles)
{
    if (GMeasurement_BuildWaveform(
            s_CaptureFrame,
            SPECTRUM_FRAME_LENGTH,
            G_MEASUREMENT_WAVEFORM_SAMPLE_RATE_HZ,
            s_Result.fundamental_hz,
            cycles,
            &s_Waveform) == 0U)
    {
        return 0U;
    }

    GSignalFlow_UpdateHmiWaveform();
    return 1U;
}

static void GSignalFlow_UpdateHmiWaveform(void)
{
    int32_t minimum;
    int32_t range;
    uint16_t point;
    uint16_t display_points = s_WaveDisplayPoints;

    if ((s_Waveform.valid == 0U) ||
        (s_Waveform.point_count != G_MEASUREMENT_WAVEFORM_POINTS))
    {
        return;
    }

    minimum = (int32_t)s_Waveform.minimum_code;
    range = (int32_t)s_Waveform.maximum_code - minimum;

    if (display_points == 0U)
    {
        if (TjcHmi_GetComponentWidth("s0", &display_points) != 0U)
        {
            s_WaveDisplayPoints = display_points;
        }
        else
        {
            display_points = G_FLOW_WAVE_FALLBACK_POINTS;
        }
    }
    if (display_points > G_FLOW_WAVE_MAX_POINTS)
    {
        display_points = G_FLOW_WAVE_MAX_POINTS;
    }

    for (point = 0U; point < display_points; point++)
    {
        uint32_t source_numerator =
            (uint32_t)point *
            (uint32_t)(s_Waveform.point_count - 1U);
        uint16_t source_index = (uint16_t)(
            source_numerator / (display_points - 1U));
        uint32_t source_remainder =
            source_numerator % (display_points - 1U);
        uint16_t next_index =
            (source_index + 1U < s_Waveform.point_count)
            ? (uint16_t)(source_index + 1U)
            : source_index;
        int32_t interpolated =
            ((int32_t)s_Waveform.points[source_index] *
             (int32_t)((display_points - 1U) -
                       source_remainder) +
             (int32_t)s_Waveform.points[next_index] *
             (int32_t)source_remainder) /
            (int32_t)(display_points - 1U);

        if (range <= 0)
        {
            s_WaveDisplay[point] = G_FLOW_WAVE_VERTICAL_OFFSET;
        }
        else
        {
            int32_t centered = interpolated - minimum;
            int32_t original_height = (centered * 254) / range;
            int32_t scaled = original_height /
                             (int32_t)G_FLOW_WAVE_HEIGHT_DIVISOR +
                             (int32_t)G_FLOW_WAVE_VERTICAL_OFFSET;

            if (scaled < 0)
            {
                scaled = 0;
            }
            if (scaled > 254)
            {
                scaled = 254;
            }
            s_WaveDisplay[point] = (uint8_t)scaled;
        }
    }

    tjc_clear_wave("s0.id", 0);
    tjc_send_wave("s0.id",
                  0,
                  s_WaveDisplay,
                  display_points);
}

static uint32_t GSignalFlow_RoundPositive(float value)
{
    if (value <= 0.0f)
    {
        return 0UL;
    }

    if (value >= 4294967040.0f)
    {
        return 0xFFFFFFFFUL;
    }

    return (uint32_t)(value + 0.5f);
}

const GMeasurementResult *GSignalFlow_GetLatestMeasurement(void)
{
    return &s_Measurement;
}

uint8_t GSignalFlow_SetWaveformCycles(uint8_t cycles)
{
    if ((cycles != 1U) && (cycles != 3U))
    {
        return 0U;
    }

    s_SelectedWaveformCycles = cycles;
    return 1U;
}

const GMeasurementWaveform *GSignalFlow_GetLatestWaveform(void)
{
    return &s_Waveform;
}
