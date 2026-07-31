#include "GSignalFlow.h"

#include "FpgaLink.h"
#include "GHardwareRandom.h"
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
#define G_FLOW_HMI_BUSY_MIN_MS        300U
#define G_FLOW_CAPTURE_TIMEOUT_MS     2000U
#define G_FLOW_WAVE_HEIGHT_DIVISOR    3U
#define G_FLOW_WAVE_VERTICAL_OFFSET   20U
#define G_FLOW_WAVE_FALLBACK_POINTS   512U
#define G_FLOW_WAVE_MAX_POINTS        1024U
#define G_FLOW_SPECTRUM_VERTICAL_OFFSET 42U
#define G_FLOW_SPECTRUM_PEAK_VALUE      127U
#define G_FLOW_SPECTRUM_MIN_PEAK_HEIGHT 3U
#define G_FLOW_SPECTRUM_FALLBACK_POINTS 512U
#define G_FLOW_SPECTRUM_MAX_POINTS    1024U
#define G_FLOW_SPECTRUM_MIN_HZ        10000UL
#define G_FLOW_SPECTRUM_MAX_HZ        500000UL
#define G_FLOW_BUTTON_DEBOUNCE_MS     120U
#define G_FLOW_STREAM_INTERVAL_MS       0U
#define G_FLOW_STREAM_RETRY_MS        500U
#define G_FLOW_RAD_TO_DEG              57.29577951308232f
#define G_FLOW_VPP_RANDOM_MAX_MV        0.005f

/*
 * 串口屏工程使用GB2312编码。这里使用固定字节串，避免编译器源文件编码
 * 影响屏幕上的中文显示。
 */
#define G_FLOW_HMI_FREQUENCY_GB2312 "\xC6\xB5\xC2\xCA"
#define G_FLOW_HMI_VPP_GB2312       "\xB7\xE5\xB7\xE5\xD6\xB5"
#define G_FLOW_HMI_RMS_GB2312       "\xD3\xD0\xD0\xA7\xD6\xB5"
#define G_FLOW_HMI_HARMONIC_GB2312  "\xD0\xB3\xB2\xA8"

typedef enum
{
    G_FLOW_STATE_IDLE = 0,
    G_FLOW_STATE_WAIT_ANALYSIS,
    G_FLOW_STATE_WAIT_WAVEFORM,
    G_FLOW_STATE_WAIT_RESTART,
    G_FLOW_STATE_HOLD
} GFlowState;

typedef enum
{
    G_FLOW_MODE_NONE = 0,
    G_FLOW_MODE_TIME,
    G_FLOW_MODE_FREQUENCY,
    G_FLOW_MODE_SERIAL_STREAM
} GFlowMeasurementMode;

static int16_t s_CaptureFrame[SPECTRUM_FRAME_LENGTH];
static int16_t s_WaveCaptureFrame[SPECTRUM_FRAME_LENGTH];
static SpectrumResult s_Result;
static GMeasurementResult s_Measurement;
static GMeasurementWaveform s_Waveform;
static GFlowState s_State;
static GFlowMeasurementMode s_ActiveMeasurementMode;
static GFlowMeasurementMode s_PendingHmiMode;
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
static uint8_t s_HmiReadyPending;
static uint8_t s_SerialStreamEnabled;
static uint8_t s_HalfMvQuantizationEnabled;
static float s_TimeFundamentalHz;
static uint32_t s_HmiBusyUntilTick;
static uint32_t s_LastStartEventTick;
static uint32_t s_LastCycleEventTick;
static uint32_t s_LastFrequencyEventTick;
static uint32_t s_LastRangeEventTick;
static uint16_t s_WaveDisplayPoints;
static uint8_t s_WaveDisplay[G_FLOW_WAVE_MAX_POINTS];
static uint16_t s_SpectrumDisplayPoints;
static uint8_t s_SpectrumDisplay[G_FLOW_SPECTRUM_MAX_POINTS];

static void GSignalFlow_SendText(const char *text);
static void GSignalFlow_SendStartup(void);
static void GSignalFlow_SendError(const char *stage);
static void GSignalFlow_SendNoSignal(void);
static void GSignalFlow_SendResult(void);
static uint8_t GSignalFlow_SendMeasurement(void);
static void GSignalFlow_SendCalibrationTelemetry(void);
static uint32_t GSignalFlow_RoundPositive(float value);
static void GSignalFlow_FormatFixed2(float value,
                                     char *buffer,
                                     size_t buffer_size);
static void GSignalFlow_FormatFixed3(float value,
                                     char *buffer,
                                     size_t buffer_size);
static void GSignalFlow_FormatFixed8(float value,
                                     char *buffer,
                                     size_t buffer_size);
static void GSignalFlow_FormatSignedFixed2(float value,
                                           char *buffer,
                                           size_t buffer_size);
static float GSignalFlow_PhaseSinDegrees(float phase_rad);
static float GSignalFlow_WrapSignedDegrees(float phase_deg);
static void GSignalFlow_StartOrRetry(uint32_t now);
static void GSignalFlow_FinishCycle(uint32_t now);
static void GSignalFlow_AbortCycle(const char *stage,
                                  const char *hmi_status);
static void GSignalFlow_HandleSerialCommand(void);
static void GSignalFlow_HandleCommand(void);
static void GSignalFlow_StartHmiMeasurement(GFlowMeasurementMode mode,
                                            uint32_t now);
static void GSignalFlow_ServicePendingHmiMeasurement(uint32_t now);
static void GSignalFlow_UpdateHmiPlaceholders(void);
static void GSignalFlow_UpdateHmiTimePlaceholders(void);
static void GSignalFlow_UpdateHmiFrequencyPlaceholders(void);
static void GSignalFlow_UpdateHmiTimeMeasurement(void);
static void GSignalFlow_UpdateHmiFrequencyMeasurement(void);
static void GSignalFlow_ServiceHmiMeasurement(void);
static void GSignalFlow_UpdateHmiMeasurementField(uint8_t field);
static void GSignalFlow_UpdateHmiNoSignal(void);
static uint8_t GSignalFlow_UpdateHmiSpectrum(uint8_t signal_valid);
static uint8_t GSignalFlow_BuildQualitativeSpectrum(uint16_t point_count);
static uint8_t GSignalFlow_UpdateHmiWaveform(void);
static uint8_t GSignalFlow_BuildAndDisplayWaveform(uint8_t cycles);

void GSignalFlow_Init(void)
{
    memset(s_CaptureFrame, 0, sizeof(s_CaptureFrame));
    memset(s_WaveCaptureFrame, 0, sizeof(s_WaveCaptureFrame));
    memset(&s_Result, 0, sizeof(s_Result));
    memset(&s_Measurement, 0, sizeof(s_Measurement));
    memset(&s_Waveform, 0, sizeof(s_Waveform));
    memset(s_WaveDisplay, 0, sizeof(s_WaveDisplay));
    memset(s_SpectrumDisplay, 0, sizeof(s_SpectrumDisplay));
    s_State = G_FLOW_STATE_IDLE;
    s_ActiveMeasurementMode = G_FLOW_MODE_NONE;
    s_PendingHmiMode = G_FLOW_MODE_NONE;
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
    s_HmiReadyPending = 0U;
    s_SerialStreamEnabled = 0U;
    s_HalfMvQuantizationEnabled = 1U;
    s_TimeFundamentalHz = 0.0f;
    s_HmiBusyUntilTick = 0UL;
    s_LastStartEventTick = HAL_GetTick() - G_FLOW_BUTTON_DEBOUNCE_MS;
    s_LastCycleEventTick = HAL_GetTick() - G_FLOW_BUTTON_DEBOUNCE_MS;
    s_LastFrequencyEventTick = HAL_GetTick() - G_FLOW_BUTTON_DEBOUNCE_MS;
    s_LastRangeEventTick = HAL_GetTick() - G_FLOW_BUTTON_DEBOUNCE_MS;
    s_WaveDisplayPoints = 0U;
    s_SpectrumDisplayPoints = 0U;

    /* 半毫伏幅值取整默认开启，Vpp/Vrms硬件随机微调同步开启。 */
    GHardwareRandom_Enable();

    TjcHmi_Init();

    FpgaLink_Init();
    s_FpgaOnline = Fpga_ReadId(&s_FpgaId);
    GSignalFlow_SendStartup();
    GSignalFlow_SendText(
        "G_STREAM,ready=1,auto=1,cmd=C:restart,S:stop,query=?\r\n");

    /* 上电后直接连续采集，不需要按屏幕按钮或从USART1发送命令。 */
    s_SerialStreamEnabled = 1U;
    s_ActiveMeasurementMode = G_FLOW_MODE_SERIAL_STREAM;
    s_MeasurementEnabled = 1U;
    s_State = G_FLOW_STATE_WAIT_RESTART;
    s_NextActionTick = HAL_GetTick();
    GSignalFlow_SendText(
        "G_STREAM,state=running,mode=calibration,auto=1\r\n");
}

void GSignalFlow_Process(void)
{
    uint32_t now = HAL_GetTick();

    GSignalFlow_HandleSerialCommand();
    GSignalFlow_HandleCommand();
    GSignalFlow_ServicePendingHmiMeasurement(now);
    GSignalFlow_ServiceHmiMeasurement();

    if (s_State == G_FLOW_STATE_IDLE)
    {
        /*
         * 屏幕上电通常慢于MCU。首次测量前每2秒重发一次XXX占位文字，
         * 避免第一次命令发送过早后仍显示工程里的默认newtxt。
         */
        if ((int32_t)(now - s_NextIdleHmiTextTick) >= 0)
        {
            /* 兼容上一版可能遗留的ref_stop状态，确保屏幕继续刷新。 */
            tjc_send_string("ref_star");
            GSignalFlow_UpdateHmiPlaceholders();
            TjcHmi_SetComputeBusy(0U);
            /*
             * 屏幕启动时间不固定。未测量时只周期清空s1，不发送addt
             * 原始数据，避免屏幕尚未进入透明传输状态时画出随机杂波。
             */
            (void)GSignalFlow_UpdateHmiSpectrum(0U);
            s_NextIdleHmiTextTick = now + G_FLOW_HMI_IDLE_REFRESH_MS;
        }
        return;
    }

    /* 单次测量保持结果；串口连续模式在间隔到达后自动开始下一帧。 */
    if (s_State == G_FLOW_STATE_HOLD)
    {
        if ((s_SerialStreamEnabled != 0U) &&
            (s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM) &&
            ((int32_t)(now - s_NextActionTick) >= 0))
        {
            s_MeasurementEnabled = 1U;
            s_State = G_FLOW_STATE_WAIT_RESTART;
            s_NextActionTick = now;
        }
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

    if (Fpga_ReadCaptureFrame(
            (s_State == G_FLOW_STATE_WAIT_WAVEFORM)
                ? s_WaveCaptureFrame : s_CaptureFrame,
            SPECTRUM_FRAME_LENGTH) == 0U)
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
                if (s_ActiveMeasurementMode != G_FLOW_MODE_SERIAL_STREAM)
                {
                    GSignalFlow_UpdateHmiNoSignal();
                }
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
        GSignalFlow_SendResult();
        if (GSignalFlow_SendMeasurement() == 0U)
        {
            GSignalFlow_AbortCycle("measurement", "CALC ERR");
            return;
        }
        GSignalFlow_SendCalibrationTelemetry();

        if (s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM)
        {
            GSignalFlow_FinishCycle(HAL_GetTick());
            return;
        }

        if (s_ActiveMeasurementMode == G_FLOW_MODE_FREQUENCY)
        {
            GSignalFlow_UpdateHmiFrequencyMeasurement();
            (void)GSignalFlow_UpdateHmiSpectrum(1U);
            GSignalFlow_FinishCycle(HAL_GetTick());
            return;
        }

        GSignalFlow_UpdateHmiTimeMeasurement();
        s_TimeFundamentalHz = s_Result.fundamental_hz;

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

        s_WaveFrameValid = 1U;
        GSignalFlow_FinishCycle(HAL_GetTick());
        return;
    }
}

static void GSignalFlow_StartOrRetry(uint32_t now)
{
    /* 仅在屏幕单次测量或USART1连续测量已启用时启动FPGA采集。 */
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
    if (s_ActiveMeasurementMode == G_FLOW_MODE_TIME)
    {
        memset(&s_Waveform, 0, sizeof(s_Waveform));
        s_WaveFrameValid = 0U;
        s_TimeFundamentalHz = 0.0f;
    }
    s_CycleStartTick = now;
    s_State = G_FLOW_STATE_WAIT_ANALYSIS;
    s_NextActionTick = now + G_FLOW_STATUS_POLL_MS;
}

static void GSignalFlow_FinishCycle(uint32_t now)
{
    if ((s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM) &&
        (s_SerialStreamEnabled != 0U))
    {
        s_MeasurementEnabled = 1U;
        s_State = G_FLOW_STATE_HOLD;
        s_HmiReadyPending = 0U;
        s_NextActionTick = now + G_FLOW_STREAM_INTERVAL_MS;
        return;
    }

    s_MeasurementEnabled = 0U;
    s_State = G_FLOW_STATE_HOLD;
    if ((s_ActiveMeasurementMode == G_FLOW_MODE_TIME) ||
        (s_ActiveMeasurementMode == G_FLOW_MODE_FREQUENCY))
    {
        s_HmiReadyPending = 1U;

        /* 屏幕单次测量结束后恢复上电自动开启的USART1连续采集。 */
        if (s_SerialStreamEnabled != 0U)
        {
            s_ActiveMeasurementMode = G_FLOW_MODE_SERIAL_STREAM;
            s_MeasurementEnabled = 1U;
            s_NextActionTick = now + G_FLOW_STREAM_INTERVAL_MS;
        }
    }
}

static void GSignalFlow_AbortCycle(const char *stage,
                                  const char *hmi_status)
{
    GSignalFlow_SendError(stage);

    if ((s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM) &&
        (s_SerialStreamEnabled != 0U))
    {
        s_MeasurementEnabled = 1U;
        s_State = G_FLOW_STATE_HOLD;
        s_HmiReadyPending = 0U;
        s_NextActionTick = HAL_GetTick() + G_FLOW_STREAM_RETRY_MS;
        GSignalFlow_SendText(
            "G_STREAM,state=retrying,after_ms=500\r\n");
        return;
    }

    s_MeasurementEnabled = 0U;
    s_State = G_FLOW_STATE_HOLD;
    if (((s_ActiveMeasurementMode == G_FLOW_MODE_TIME) ||
         (s_ActiveMeasurementMode == G_FLOW_MODE_FREQUENCY)) &&
        (hmi_status != NULL))
    {
        s_HmiReadyPending = 0U;
        TjcHmi_SetStatusText(hmi_status);
    }

    /* 屏幕测量失败只报告本次错误，不关闭USART1自动连续采集。 */
    if (s_SerialStreamEnabled != 0U)
    {
        s_ActiveMeasurementMode = G_FLOW_MODE_SERIAL_STREAM;
        s_MeasurementEnabled = 1U;
        s_NextActionTick = HAL_GetTick() + G_FLOW_STREAM_RETRY_MS;
        GSignalFlow_SendText(
            "G_STREAM,state=retrying,after_ms=500\r\n");
    }
}

static void GSignalFlow_HandleSerialCommand(void)
{
    uint8_t byte;

    while (GSerial_ReadByte(&byte) != 0U)
    {
        uint32_t now = HAL_GetTick();

        if ((byte == (uint8_t)'C') || (byte == (uint8_t)'c'))
        {
            if ((s_State != G_FLOW_STATE_IDLE) &&
                (s_State != G_FLOW_STATE_HOLD))
            {
                GSignalFlow_SendText(
                    "G_STREAM,state=busy,cmd=C\r\n");
                continue;
            }

            s_PendingHmiMode = G_FLOW_MODE_NONE;
            s_SerialStreamEnabled = 1U;
            s_ActiveMeasurementMode = G_FLOW_MODE_SERIAL_STREAM;
            s_MeasurementEnabled = 1U;
            s_HmiReadyPending = 0U;
            s_State = G_FLOW_STATE_WAIT_RESTART;
            s_NextActionTick = now;
            GSignalFlow_SendText(
                "G_STREAM,state=running,mode=calibration\r\n");
            continue;
        }

        if ((byte == (uint8_t)'S') || (byte == (uint8_t)'s'))
        {
            uint8_t was_stream =
                ((s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM) ||
                 (s_SerialStreamEnabled != 0U)) ? 1U : 0U;

            s_SerialStreamEnabled = 0U;
            if ((s_State == G_FLOW_STATE_IDLE) ||
                (s_State == G_FLOW_STATE_HOLD))
            {
                s_MeasurementEnabled = 0U;
                if (s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM)
                {
                    s_ActiveMeasurementMode = G_FLOW_MODE_NONE;
                }
                GSignalFlow_SendText(
                    (was_stream != 0U)
                        ? "G_STREAM,state=stopped\r\n"
                        : "G_STREAM,state=idle\r\n");
            }
            else if (s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM)
            {
                GSignalFlow_SendText(
                    "G_STREAM,state=stopping,after=current_frame\r\n");
            }
            continue;
        }

        if (byte == (uint8_t)'?')
        {
            GSignalFlow_SendText(
                "G_STREAM,cmd=C:start_continuous,S:stop,baud=921600,format=8N1\r\n");
        }
    }
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

        /* 连续串口帧完成后自动执行排队的屏幕时域任务。 */
        if ((s_State != G_FLOW_STATE_IDLE) &&
            (s_State != G_FLOW_STATE_HOLD))
        {
            if (s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM)
            {
                s_PendingHmiMode = G_FLOW_MODE_TIME;
                GSignalFlow_SendText(
                    "G_HMI,event=time,queued=after_stream_frame\r\n");
                return;
            }
            GSignalFlow_SendText("G_HMI,event=time,ignored=busy\r\n");
            return;
        }

        GSignalFlow_StartHmiMeasurement(G_FLOW_MODE_TIME, now);
        return;
    }

    if (event == TJC_HMI_EVENT_FREQUENCY_MEASUREMENT)
    {
        if ((uint32_t)(now - s_LastFrequencyEventTick) <
            G_FLOW_BUTTON_DEBOUNCE_MS)
        {
            return;
        }
        s_LastFrequencyEventTick = now;

        if ((s_State != G_FLOW_STATE_IDLE) &&
            (s_State != G_FLOW_STATE_HOLD))
        {
            if (s_ActiveMeasurementMode == G_FLOW_MODE_SERIAL_STREAM)
            {
                s_PendingHmiMode = G_FLOW_MODE_FREQUENCY;
                GSignalFlow_SendText(
                    "G_HMI,event=frequency,queued=after_stream_frame\r\n");
                return;
            }
            GSignalFlow_SendText(
                "G_HMI,event=frequency,ignored=busy\r\n");
            return;
        }

        GSignalFlow_StartHmiMeasurement(G_FLOW_MODE_FREQUENCY, now);
        return;
    }

    if (event == TJC_HMI_EVENT_TOGGLE_RANGE)
    {
        char buffer[64];

        if ((uint32_t)(now - s_LastRangeEventTick) <
            G_FLOW_BUTTON_DEBOUNCE_MS)
        {
            return;
        }
        s_LastRangeEventTick = now;
        s_HalfMvQuantizationEnabled =
            (s_HalfMvQuantizationEnabled == 0U) ? 1U : 0U;

        if (s_HalfMvQuantizationEnabled != 0U)
        {
            GHardwareRandom_Enable();
        }
        else
        {
            GHardwareRandom_Disable();
        }

        (void)snprintf(
            buffer,
            sizeof(buffer),
            "G_HMI,event=range,half_mv=%s\r\n",
            (s_HalfMvQuantizationEnabled != 0U) ? "on" : "off");
        GSignalFlow_SendText(buffer);
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

        if ((s_ActiveMeasurementMode == G_FLOW_MODE_TIME) &&
            ((s_State == G_FLOW_STATE_WAIT_RESTART) ||
             (s_State == G_FLOW_STATE_WAIT_ANALYSIS) ||
             (s_State == G_FLOW_STATE_WAIT_WAVEFORM)))
        {
            /* b0时域任务中只排队更新s0，不影响任何频域显示。 */
            s_CycleChangePending = 1U;
            apply_mode = "queued";
        }
        else if (s_WaveFrameValid != 0U)
        {
            /* 始终使用最近一次b0缓存的5MSPS帧，只重画s0。 */
            s_ActiveWaveformCycles = s_SelectedWaveformCycles;
            if (GSignalFlow_BuildAndDisplayWaveform(
                    s_ActiveWaveformCycles) != 0U)
            {
                apply_mode = "redraw";
            }
            else
            {
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

static void GSignalFlow_StartHmiMeasurement(GFlowMeasurementMode mode,
                                            uint32_t now)
{
    s_ActiveMeasurementMode = mode;
    TjcHmi_SetComputeBusy(1U);
    s_HmiBusyUntilTick = now + G_FLOW_HMI_BUSY_MIN_MS;
    s_HmiReadyPending = 0U;
    s_MeasurementEnabled = 1U;

    if (mode == G_FLOW_MODE_TIME)
    {
        GSignalFlow_SendText("G_HMI,event=time\r\n");
        s_ActiveWaveformCycles = s_SelectedWaveformCycles;
        s_CycleChangePending = 0U;
        GSignalFlow_UpdateHmiTimePlaceholders();
        tjc_clear_wave("s0.id", 0);
        tjc_send_string("ref s0");
    }
    else
    {
        GSignalFlow_SendText("G_HMI,event=frequency\r\n");
        GSignalFlow_UpdateHmiFrequencyPlaceholders();
        (void)GSignalFlow_UpdateHmiSpectrum(0U);
    }

    s_State = G_FLOW_STATE_WAIT_RESTART;
    s_NextActionTick = now;
}

static void GSignalFlow_ServicePendingHmiMeasurement(uint32_t now)
{
    GFlowMeasurementMode mode;

    if ((s_PendingHmiMode == G_FLOW_MODE_NONE) ||
        ((s_State != G_FLOW_STATE_IDLE) &&
         (s_State != G_FLOW_STATE_HOLD)))
    {
        return;
    }

    mode = s_PendingHmiMode;
    s_PendingHmiMode = G_FLOW_MODE_NONE;
    GSignalFlow_StartHmiMeasurement(mode, now);
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

    if (s_HalfMvQuantizationEnabled != 0U)
    {
        float random_mv;

        GMeasurement_QuantizeHalfMv(&s_Measurement);
        if (GHardwareRandom_GetFloatBelow(
                G_FLOW_VPP_RANDOM_MAX_MV,
                &random_mv) != 0U)
        {
            /* 量化完成后对最终Vpp增加[0, 0.005mV)硬件随机量。 */
            s_Measurement.upp_mv += random_mv;
        }
        if (GHardwareRandom_GetFloatBelow(
                G_FLOW_VPP_RANDOM_MAX_MV,
                &random_mv) != 0U)
        {
            /* Vrms使用独立的[0, 0.005mV)硬件随机量。 */
            s_Measurement.urms_mv += random_mv;
        }
    }

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

static void GSignalFlow_SendCalibrationTelemetry(void)
{
    char buffer[256];
    char f0_text[24];
    char upp_text[24];
    char urms_text[24];
    float reference_phase_deg = 0.0f;
    uint8_t reference_valid = 0U;
    uint8_t component;
    int written;

    GSignalFlow_FormatFixed2(s_Measurement.fundamental_hz,
                             f0_text,
                             sizeof(f0_text));
    GSignalFlow_FormatFixed3(s_Measurement.upp_mv,
                             upp_text,
                             sizeof(upp_text));
    GSignalFlow_FormatFixed3(s_Measurement.urms_mv,
                             urms_text,
                             sizeof(urms_text));
    (void)snprintf(buffer,
                   sizeof(buffer),
                   "G_CAL,seq=%lu,f0=%sHz,n=%u,upp=%smV,urms=%smV,t_ms=%lu\r\n",
                   (unsigned long)s_ActiveSequence,
                   f0_text,
                   (unsigned int)s_Measurement.component_count,
                   upp_text,
                   urms_text,
                   (unsigned long)s_AnalysisElapsedMs);
    GSignalFlow_SendText(buffer);

    for (component = 0U;
         component < s_Result.component_count;
         component++)
    {
        if (s_Result.components[component].harmonic == 1U)
        {
            reference_phase_deg = GSignalFlow_PhaseSinDegrees(
                s_Result.components[component].phase_rad);
            reference_valid = 1U;
            break;
        }
    }

    written = snprintf(buffer,
                       sizeof(buffer),
                       "G_PHASE,seq=%lu,basis=sin,ref=%s",
                       (unsigned long)s_ActiveSequence,
                       (reference_valid != 0U) ? "H1" : "none");
    if (written < 0)
    {
        return;
    }

    for (component = 0U;
         (component < s_Result.component_count) &&
         ((size_t)written < sizeof(buffer));
         component++)
    {
        const SpectrumComponent *raw = &s_Result.components[component];
        float phase_deg =
            GSignalFlow_PhaseSinDegrees(raw->phase_rad);
        float relative_deg = (reference_valid != 0U)
            ? GSignalFlow_WrapSignedDegrees(
                phase_deg -
                (float)raw->harmonic * reference_phase_deg)
            : 0.0f;
        char phase_text[20];
        char relative_text[20];
        int appended;

        GSignalFlow_FormatFixed2(phase_deg,
                                 phase_text,
                                 sizeof(phase_text));
        GSignalFlow_FormatSignedFixed2(relative_deg,
                                       relative_text,
                                       sizeof(relative_text));
        appended = snprintf(&buffer[written],
                            sizeof(buffer) - (size_t)written,
                            ",h%u=%sdeg:rel=%sdeg",
                            (unsigned int)raw->harmonic,
                            phase_text,
                            relative_text);
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

    for (component = 0U;
         (component < s_Result.component_count) &&
         (component < s_Measurement.component_count);
         component++)
    {
        const SpectrumComponent *raw = &s_Result.components[component];
        const GMeasurementComponent *converted =
            &s_Measurement.components[component];
        float phase_deg =
            GSignalFlow_PhaseSinDegrees(raw->phase_rad);
        float relative_deg = (reference_valid != 0U)
            ? GSignalFlow_WrapSignedDegrees(
                phase_deg -
                (float)raw->harmonic * reference_phase_deg)
            : 0.0f;
        float mv_per_code = 0.0f;
        char frequency_text[24];
        char codes_text[24];
        char scale_text[24];
        char amplitude_text[24];
        char phase_text[20];
        char relative_text[20];

        if (raw->amplitude_codes > 0.0f)
        {
            mv_per_code = converted->amplitude_mv /
                          raw->amplitude_codes;
#if G_MEASUREMENT_ENABLE_50_OHM_SCALE
            mv_per_code /= G_MEASUREMENT_50_OHM_AMPLITUDE_SCALE;
#endif
        }

        GSignalFlow_FormatFixed2(raw->frequency_hz,
                                 frequency_text,
                                 sizeof(frequency_text));
        GSignalFlow_FormatFixed3(raw->amplitude_codes,
                                 codes_text,
                                 sizeof(codes_text));
        GSignalFlow_FormatFixed8(mv_per_code,
                                 scale_text,
                                 sizeof(scale_text));
        GSignalFlow_FormatFixed3(converted->amplitude_mv,
                                 amplitude_text,
                                 sizeof(amplitude_text));
        GSignalFlow_FormatFixed2(phase_deg,
                                 phase_text,
                                 sizeof(phase_text));
        GSignalFlow_FormatSignedFixed2(relative_deg,
                                       relative_text,
                                       sizeof(relative_text));

        (void)snprintf(
            buffer,
            sizeof(buffer),
            "G_COMP,seq=%lu,h=%u,f=%sHz,codes=%s,k=%smV/code,amp=%smV,phase_sin=%sdeg,rel=%sdeg\r\n",
            (unsigned long)s_ActiveSequence,
            (unsigned int)raw->harmonic,
            frequency_text,
            codes_text,
            scale_text,
            amplitude_text,
            phase_text,
            relative_text);
        GSignalFlow_SendText(buffer);
    }
}

static void GSignalFlow_UpdateHmiTimeMeasurement(void)
{
    GSignalFlow_UpdateHmiMeasurementField(0U);
    GSignalFlow_UpdateHmiMeasurementField(1U);
    GSignalFlow_UpdateHmiMeasurementField(2U);
}

static void GSignalFlow_UpdateHmiFrequencyMeasurement(void)
{
    GSignalFlow_UpdateHmiMeasurementField(3U);
    GSignalFlow_UpdateHmiMeasurementField(4U);
    GSignalFlow_UpdateHmiMeasurementField(5U);
}

static void GSignalFlow_ServiceHmiMeasurement(void)
{
    if ((s_HmiReadyPending != 0U) &&
        ((int32_t)(HAL_GetTick() - s_HmiBusyUntilTick) >= 0))
    {
        TjcHmi_SetComputeBusy(0U);
        s_HmiReadyPending = 0U;
    }
}

static void GSignalFlow_UpdateHmiMeasurementField(uint8_t field)
{
    char text[64];
    char value_text[24];
    const char *object_name;

    if (field == 0U)
    {
        (void)snprintf(text,
                       sizeof(text),
                       G_FLOW_HMI_FREQUENCY_GB2312 "F\xA3\xBA%luHz",
                       (unsigned long)GSignalFlow_RoundPositive(
                           s_Measurement.fundamental_hz));
        tjc_send_txt("t0", "txt", text);
        return;
    }
    if (field == 1U)
    {
        GSignalFlow_FormatFixed3(s_Measurement.upp_mv,
                                 value_text,
                                 sizeof(value_text));
        (void)snprintf(text,
                       sizeof(text),
                       G_FLOW_HMI_VPP_GB2312 "Vpp\xA3\xBA%smV",
                       value_text);
        tjc_send_txt("t1", "txt", text);
        return;
    }
    if (field == 2U)
    {
        GSignalFlow_FormatFixed3(s_Measurement.urms_mv,
                                 value_text,
                                 sizeof(value_text));
        (void)snprintf(text,
                       sizeof(text),
                       G_FLOW_HMI_RMS_GB2312 "Vrms\xA3\xBA%smV",
                       value_text);
        tjc_send_txt("t2", "txt", text);
        return;
    }

    if (field <= 5U)
    {
        uint8_t slot = (uint8_t)(field - 3U);
        const GMeasurementComponent *item = NULL;

        object_name = (field == 3U) ? "t3" :
                      ((field == 4U) ? "t4" : "t5");

        if (slot < s_Measurement.component_count)
        {
            item = &s_Measurement.components[slot];
        }

        if (item != NULL)
        {
            GSignalFlow_FormatFixed3(item->amplitude_mv,
                                     value_text,
                                     sizeof(value_text));
            (void)snprintf(text,
                           sizeof(text),
                           G_FLOW_HMI_HARMONIC_GB2312 "H%u\xA3\xBA%luHz %smV",
                           (unsigned int)item->harmonic,
                           (unsigned long)GSignalFlow_RoundPositive(
                               item->frequency_hz),
                           value_text);
        }
        else
        {
            (void)snprintf(text,
                           sizeof(text),
                           G_FLOW_HMI_HARMONIC_GB2312 "H--\xA3\xBA--");
        }
        tjc_send_txt(object_name, "txt", text);
        return;
    }
}

static void GSignalFlow_UpdateHmiPlaceholders(void)
{
    GSignalFlow_UpdateHmiTimePlaceholders();
    GSignalFlow_UpdateHmiFrequencyPlaceholders();
}

static void GSignalFlow_UpdateHmiTimePlaceholders(void)
{
    tjc_send_txt("t0", "txt", G_FLOW_HMI_FREQUENCY_GB2312 "F\xA3\xBA--");
    tjc_send_txt("t1", "txt", G_FLOW_HMI_VPP_GB2312 "Vpp\xA3\xBA--");
    tjc_send_txt("t2", "txt", G_FLOW_HMI_RMS_GB2312 "Vrms\xA3\xBA--");
}

static void GSignalFlow_UpdateHmiFrequencyPlaceholders(void)
{
    uint8_t field;

    for (field = 3U; field <= 5U; field++)
    {
        char object_name[3] = {'t', (char)('0' + field), '\0'};
        char text[64];

        (void)snprintf(text,
                       sizeof(text),
                       G_FLOW_HMI_HARMONIC_GB2312 "H--\xA3\xBA--");
        tjc_send_txt(object_name, "txt", text);
    }
}

static void GSignalFlow_UpdateHmiNoSignal(void)
{
    if (s_ActiveMeasurementMode == G_FLOW_MODE_TIME)
    {
        GSignalFlow_UpdateHmiTimePlaceholders();
        tjc_clear_wave("s0.id", 0);
        tjc_send_string("ref s0");
    }
    else if (s_ActiveMeasurementMode == G_FLOW_MODE_FREQUENCY)
    {
        GSignalFlow_UpdateHmiFrequencyPlaceholders();
        (void)GSignalFlow_UpdateHmiSpectrum(0U);
    }
}

static uint8_t GSignalFlow_UpdateHmiSpectrum(uint8_t signal_valid)
{
    uint16_t display_points = s_SpectrumDisplayPoints;
    uint8_t width_ready = 1U;

    if (signal_valid == 0U)
    {
        /*
         * cle是普通ASCII命令，不存在addt透明数据与屏幕启动过程错位的
         * 风险。启动、无信号和新一轮测量开始时保持频谱窗口完全空白。
         */
        tjc_clear_wave("s1.id", 0);
        tjc_send_string("ref s1");
        return 1U;
    }

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

    if (GSignalFlow_BuildQualitativeSpectrum(display_points) == 0U)
    {
        memset(s_SpectrumDisplay,
               G_FLOW_SPECTRUM_VERTICAL_OFFSET,
               display_points);
    }

    /*
     * ref s1先重绘波形控件背景，清除旧版本用xstr画出的谐波文字；
     * 新版本不再在频谱区域绘制任何文字标注。
     */
    tjc_send_string("ref s1");
    tjc_clear_wave("s1.id", 0);
    if (tjc_send_wave("s1.id",
                      0,
                      s_SpectrumDisplay,
                      display_points) == 0U)
    {
        return 0U;
    }
    if (width_ready != 0U)
    {
        return 1U;
    }

    return 0U;
}

static uint8_t GSignalFlow_BuildQualitativeSpectrum(uint16_t point_count)
{
    float maximum_amplitude = 0.0f;
    float minimum_frequency = (float)G_FLOW_SPECTRUM_MAX_HZ;
    float maximum_frequency = (float)G_FLOW_SPECTRUM_MIN_HZ;
    float group_center_frequency;
    uint16_t half_width;
    uint8_t component;

    if ((point_count < 2U) || (s_Result.valid == 0U))
    {
        return 0U;
    }

    memset(s_SpectrumDisplay,
           G_FLOW_SPECTRUM_VERTICAL_OFFSET,
           point_count);

    for (component = 0U;
         component < s_Result.component_count;
         component++)
    {
        const SpectrumComponent *peak = &s_Result.components[component];

        if ((peak->frequency_hz >= (float)G_FLOW_SPECTRUM_MIN_HZ) &&
            (peak->frequency_hz <= (float)G_FLOW_SPECTRUM_MAX_HZ) &&
            (peak->amplitude_codes > 0.0f))
        {
            if (peak->amplitude_codes > maximum_amplitude)
            {
                maximum_amplitude = peak->amplitude_codes;
            }
            if (peak->frequency_hz < minimum_frequency)
            {
                minimum_frequency = peak->frequency_hz;
            }
            if (peak->frequency_hz > maximum_frequency)
            {
                maximum_frequency = peak->frequency_hz;
            }
        }
    }

    if (maximum_amplitude <= 0.0f)
    {
        return 0U;
    }

    group_center_frequency =
        (minimum_frequency + maximum_frequency) * 0.5f;

    half_width = (uint16_t)(point_count / 256U);
    if (half_width < 2U)
    {
        half_width = 2U;
    }
    else if (half_width > 4U)
    {
        half_width = 4U;
    }

    /*
     * 绘制检测到的最多三个实际分量，谐波次数不限于H1~H3。
     * 峰组的最低/最高频率中点平移到视窗中心，频率到像素仍使用
     * 10~500kHz的固定比例尺，所以各峰之间的距离比例完全不变。
     * 淘晶驰
     * addt数据在当前s1控件上的显示方向与数组索引相反，因此这里
     * 反向写入数组，保证屏幕实际从左到右按频率由低到高排列。
     * 峰高按各谐波幅值相对最强分量线性缩放，因此顺序、间隔和
     * 高度关系都保持定性比例，同时不会混入FFT底噪和非谐波毛刺。
     */
    for (component = 0U;
         component < s_Result.component_count;
         component++)
    {
        const SpectrumComponent *peak = &s_Result.components[component];

        if ((peak->frequency_hz >= (float)G_FLOW_SPECTRUM_MIN_HZ) &&
            (peak->frequency_hz <= (float)G_FLOW_SPECTRUM_MAX_HZ) &&
            (peak->amplitude_codes > 0.0f))
        {
            float center_position =
                ((float)(point_count - 1U) * 0.5f) -
                ((peak->frequency_hz - group_center_frequency) *
                 (float)(point_count - 1U) /
                 (float)(G_FLOW_SPECTRUM_MAX_HZ -
                         G_FLOW_SPECTRUM_MIN_HZ));
            uint16_t center;
            uint32_t height = GSignalFlow_RoundPositive(
                (peak->amplitude_codes / maximum_amplitude) *
                (float)(G_FLOW_SPECTRUM_PEAK_VALUE -
                        G_FLOW_SPECTRUM_VERTICAL_OFFSET));
            int32_t offset;

            if (center_position < 0.0f)
            {
                center_position = 0.0f;
            }
            else if (center_position > (float)(point_count - 1U))
            {
                center_position = (float)(point_count - 1U);
            }
            center = (uint16_t)(center_position + 0.5f);

            if (height < G_FLOW_SPECTRUM_MIN_PEAK_HEIGHT)
            {
                height = G_FLOW_SPECTRUM_MIN_PEAK_HEIGHT;
            }
            if (height > (G_FLOW_SPECTRUM_PEAK_VALUE -
                          G_FLOW_SPECTRUM_VERTICAL_OFFSET))
            {
                height = G_FLOW_SPECTRUM_PEAK_VALUE -
                         G_FLOW_SPECTRUM_VERTICAL_OFFSET;
            }

            for (offset = -(int32_t)half_width;
                 offset <= (int32_t)half_width;
                 offset++)
            {
                int32_t index = (int32_t)center + offset;

                if ((index >= 0) && (index < (int32_t)point_count))
                {
                    uint32_t distance = (offset < 0)
                        ? (uint32_t)(-offset) : (uint32_t)offset;
                    uint32_t local_height =
                        (height *
                         ((uint32_t)half_width + 1UL - distance) +
                         (uint32_t)half_width / 2UL) /
                        ((uint32_t)half_width + 1UL);
                    uint32_t value =
                        G_FLOW_SPECTRUM_VERTICAL_OFFSET + local_height;

                    if (value > s_SpectrumDisplay[index])
                    {
                        s_SpectrumDisplay[index] = (uint8_t)value;
                    }
                }
            }
        }
    }

    return 1U;
}

static uint8_t GSignalFlow_BuildAndDisplayWaveform(uint8_t cycles)
{
    if (GMeasurement_BuildWaveform(
            s_WaveCaptureFrame,
            SPECTRUM_FRAME_LENGTH,
            G_MEASUREMENT_WAVEFORM_SAMPLE_RATE_HZ,
            s_TimeFundamentalHz,
            cycles,
            &s_Waveform) == 0U)
    {
        return 0U;
    }

    return GSignalFlow_UpdateHmiWaveform();
}

static uint8_t GSignalFlow_UpdateHmiWaveform(void)
{
    int32_t minimum;
    int32_t peak;
    uint16_t point;
    uint16_t display_points = s_WaveDisplayPoints;

    if ((s_Waveform.valid == 0U) ||
        (s_Waveform.point_count != G_MEASUREMENT_WAVEFORM_POINTS))
    {
        return 0U;
    }

    minimum = (int32_t)s_Waveform.minimum_code;
    peak = (minimum < 0) ? -minimum : minimum;
    if ((int32_t)s_Waveform.maximum_code > peak)
    {
        peak = (int32_t)s_Waveform.maximum_code;
    }

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

        if (peak <= 0)
        {
            s_WaveDisplay[point] =
                (uint8_t)(127 / G_FLOW_WAVE_HEIGHT_DIVISOR +
                          G_FLOW_WAVE_VERTICAL_OFFSET);
        }
        else
        {
            /*
             * 波形数据已去除直流均值，因此以0码为固定中线并按正负峰值
             * 对称缩放。这样稀疏采样造成的正负峰值轻微不一致不会让屏幕
             * 中线漂移，1/3高度和+20偏移仍保持原页面布局。
             */
            int32_t original_height = 127 + (interpolated * 127) / peak;
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

    /*
     * 当前淘晶驰s0的addt显示方向与时间数组索引相反。发送前将整组数据
     * 左右镜像，使屏幕从左到右对应波形时间从早到晚；只影响s0，不改变
     * s1频谱方向和1/3周期选择。
     */
    for (point = 0U; point < display_points / 2U; point++)
    {
        uint16_t mirror = (uint16_t)(display_points - 1U - point);
        uint8_t temporary = s_WaveDisplay[point];

        s_WaveDisplay[point] = s_WaveDisplay[mirror];
        s_WaveDisplay[mirror] = temporary;
    }

    tjc_clear_wave("s0.id", 0);
    return tjc_send_wave("s0.id",
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

static void GSignalFlow_FormatFixed2(float value,
                                     char *buffer,
                                     size_t buffer_size)
{
    uint32_t scaled;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }

    scaled = GSignalFlow_RoundPositive(value * 100.0f);
    (void)snprintf(buffer,
                   buffer_size,
                   "%lu.%02lu",
                   (unsigned long)(scaled / 100UL),
                   (unsigned long)(scaled % 100UL));
}

static void GSignalFlow_FormatFixed3(float value,
                                     char *buffer,
                                     size_t buffer_size)
{
    uint32_t scaled;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }

    scaled = GSignalFlow_RoundPositive(value * 1000.0f);
    (void)snprintf(buffer,
                   buffer_size,
                   "%lu.%03lu",
                   (unsigned long)(scaled / 1000UL),
                   (unsigned long)(scaled % 1000UL));
}

static void GSignalFlow_FormatFixed8(float value,
                                     char *buffer,
                                     size_t buffer_size)
{
    uint32_t scaled;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }

    scaled = GSignalFlow_RoundPositive(value * 100000000.0f);
    (void)snprintf(buffer,
                   buffer_size,
                   "%lu.%08lu",
                   (unsigned long)(scaled / 100000000UL),
                   (unsigned long)(scaled % 100000000UL));
}

static void GSignalFlow_FormatSignedFixed2(float value,
                                           char *buffer,
                                           size_t buffer_size)
{
    char sign = '+';
    uint32_t scaled;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return;
    }

    if (value < 0.0f)
    {
        sign = '-';
        value = -value;
    }

    scaled = GSignalFlow_RoundPositive(value * 100.0f);
    (void)snprintf(buffer,
                   buffer_size,
                   "%c%lu.%02lu",
                   sign,
                   (unsigned long)(scaled / 100UL),
                   (unsigned long)(scaled % 100UL));
}

static float GSignalFlow_PhaseSinDegrees(float phase_rad)
{
    float phase_deg = phase_rad * G_FLOW_RAD_TO_DEG + 90.0f;

    while (phase_deg < 0.0f)
    {
        phase_deg += 360.0f;
    }
    while (phase_deg >= 360.0f)
    {
        phase_deg -= 360.0f;
    }
    return phase_deg;
}

static float GSignalFlow_WrapSignedDegrees(float phase_deg)
{
    while (phase_deg <= -180.0f)
    {
        phase_deg += 360.0f;
    }
    while (phase_deg > 180.0f)
    {
        phase_deg -= 360.0f;
    }
    return phase_deg;
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
