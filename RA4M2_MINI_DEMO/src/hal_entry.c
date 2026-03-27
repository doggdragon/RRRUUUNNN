#include "hal_data.h"
#include <stdio.h>

#include "oled.h"
FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
void conveyor_display_off(void);
FSP_CPP_FOOTER

fsp_err_t err = FSP_SUCCESS;
i2c_master_event_t i2c_event = I2C_MASTER_EVENT_ABORTED;
int timeout_ms = 100000;

/* Motor control pins: only PWM is controlled in software.
 * Direction is hard-wired in hardware by user. */
#define TB6612_PIN_PWMA                BSP_IO_PORT_01_PIN_04

/* External run gate input: HIGH (or floating with pull-up) = run allowed,
 * LOW = force stop motor. Change pin according to your wiring. */
#define MOTOR_RUN_GATE_PIN             BSP_IO_PORT_00_PIN_01

/* Encoder input pins (E4A/E4B). */
#define ENC_PIN_A                      BSP_IO_PORT_00_PIN_00
#define ENC_PIN_B                      BSP_IO_PORT_02_PIN_00

/* OLED software I2C pins per user wiring. */
#define OLED_PIN_SCL                   BSP_IO_PORT_04_PIN_08
#define OLED_PIN_SDA                   BSP_IO_PORT_04_PIN_07

/* Mechanical parameters. */
#define MOTOR_GEAR_DIAMETER_MM         (12.0f)
#define BELT_PULLEY_DIAMETER_MM        (24.9f)
#define PI_F                            (3.1415926f)

/* Target speed and control parameters. */
#define TARGET_BELT_SPEED_MMPS         (40.0f)
#define TARGET_BELT_SPEED_MPS          (TARGET_BELT_SPEED_MMPS / 1000.0f)
#define MAIN_TICK_US                   (100U)
#define PWM_CARRIER_HZ                 (10000U)
#define CONTROL_PERIOD_MS              (20U)
#define MAX_ACCEL_MPS2                 (0.02f)
#define ENCODER_CPR                    (52.0f)
#define PI_CTRL_KP                     (0.35f)
#define PI_CTRL_KI                     (0.80f)
#define FEED_FORWARD_GAIN              (0.55f)
#define DEMO_AUTO_CYCLE                (0)
#define PWM_MAX_PERMIL                 (750U)
#define PWM_MIN_RUN_PERMIL             (180U)
#define OPEN_LOOP_PERMIL               (420U)
#define PWM_FF_BASE_PERMIL             (220.0f)
#define PWM_FF_GAIN_PER_MPS_PERMIL     (1300.0f)
#define ENCODER_STALL_TIMEOUT_MS       (1200U)
#define ENABLE_ENCODER_STALL_FALLBACK  (0)
#define GATE_STOP_DEBOUNCE_MS          (20U)
#define USE_SPEED_CLOSED_LOOP          (1)
#define SPEED_DISPLAY_FILTER_ALPHA     (0.05f)
#define SPEED_MEAS_WINDOW_MS           (20U)
#define SPEED_VALID_MAX_MPS            (0.60f)
#define SPEED_DISP_MAX_SLEW_MPS2       (0.20f)
#define SPEED_ZERO_DEADBAND_MPS        (0.005f)
#define ENABLE_OLED_UI                 (1)

typedef struct st_conveyor_ctrl
{
    float belt_speed_ref_mps;
    float belt_speed_meas_mps;
    float motor_rpm_meas;
    float integral;
    uint16_t pwm_permil;
    bool run_cmd;
} conveyor_ctrl_t;

static conveyor_ctrl_t g_conveyor =
{
    .belt_speed_ref_mps = 0.0f,
    .belt_speed_meas_mps = 0.0f,
    .motor_rpm_meas = 0.0f,
    .integral = 0.0f,
    .pwm_permil = 0U,
    .run_cmd = true,
};

static int32_t g_encoder_count = 0;
static int32_t g_encoder_count_last = 0;
static uint8_t g_encoder_prev_ab = 0;

static uint8_t g_pwm_phase = 0U;
static uint32_t g_uptime_ms = 0;
static uint32_t g_encoder_stall_ms = 0;
static uint32_t g_gate_low_ms = 0;
static uint32_t g_gate_high_ms = 0;
static bool g_open_loop_fallback = false;
static bool g_motor_gate_allow_run = true;
static bool g_motor_is_running = false;
static float g_belt_speed_disp_mps = 0.0f;

void user_uart_callback (uart_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
}

void rtc_callback(rtc_callback_args_t * p_args)
{
    FSP_PARAMETER_NOT_USED(p_args);
}

void i2c_master_callback(i2c_master_callback_args_t *p_args)
{
    i2c_event = I2C_MASTER_EVENT_ABORTED;
    if (NULL != p_args)
    {
        /* capture callback event for validating the i2c transfer event*/
        i2c_event = p_args->event;
    }
}

static float clampf(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        return min_v;
    }
    if (v > max_v)
    {
        return max_v;
    }

    return v;
}

static float motor_rps_to_belt_mps(float motor_rps)
{
    float pulley_rps = motor_rps * (MOTOR_GEAR_DIAMETER_MM / BELT_PULLEY_DIAMETER_MM);
    float circumference_m = PI_F * BELT_PULLEY_DIAMETER_MM / 1000.0f;

    return pulley_rps * circumference_m;
}

static void app_peripheral_init(void)
{
    /* Runtime pin remap for current hardware wiring. */
    R_IOPORT_PinCfg(&g_ioport_ctrl, TB6612_PIN_PWMA,
                    (uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT | (uint32_t) IOPORT_CFG_PORT_OUTPUT_LOW);
    R_IOPORT_PinCfg(&g_ioport_ctrl, MOTOR_RUN_GATE_PIN,
                    (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT | (uint32_t) IOPORT_CFG_PULLUP_ENABLE);
    R_IOPORT_PinCfg(&g_ioport_ctrl, ENC_PIN_A,
                    (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT | (uint32_t) IOPORT_CFG_PULLUP_ENABLE);
    R_IOPORT_PinCfg(&g_ioport_ctrl, ENC_PIN_B,
                    (uint32_t) IOPORT_CFG_PORT_DIRECTION_INPUT | (uint32_t) IOPORT_CFG_PULLUP_ENABLE);

#if ENABLE_OLED_UI
    R_IOPORT_PinCfg(&g_ioport_ctrl, OLED_PIN_SCL,
                    (uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT | (uint32_t) IOPORT_CFG_PORT_OUTPUT_HIGH);
    R_IOPORT_PinCfg(&g_ioport_ctrl, OLED_PIN_SDA,
                    (uint32_t) IOPORT_CFG_PORT_DIRECTION_OUTPUT | (uint32_t) IOPORT_CFG_PORT_OUTPUT_HIGH);
#endif
}

static void motor_hw_init(void)
{
    R_IOPORT_PinWrite(&g_ioport_ctrl, TB6612_PIN_PWMA, BSP_IO_LEVEL_LOW);

    bsp_io_level_t a_level = BSP_IO_LEVEL_LOW;
    bsp_io_level_t b_level = BSP_IO_LEVEL_LOW;
    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_A, &a_level);
    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_B, &b_level);
    g_encoder_prev_ab = (uint8_t) (((uint8_t) a_level << 1U) | (uint8_t) b_level);
}

static void motor_apply_output(uint16_t pwm_permil)
{
    bsp_io_level_t pwm_level = BSP_IO_LEVEL_LOW;
    uint16_t threshold = (uint16_t) ((pwm_permil + 5U) / 10U);  /* 0..100 */

    if (threshold >= 100U)
    {
        pwm_level = BSP_IO_LEVEL_HIGH;
    }
    else if (0U == threshold)
    {
        pwm_level = BSP_IO_LEVEL_LOW;
    }
    else
    {
        pwm_level = (g_pwm_phase < threshold) ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW;
    }

    g_pwm_phase++;
    if (g_pwm_phase >= 100U)
    {
        g_pwm_phase = 0U;
    }

    R_IOPORT_PinWrite(&g_ioport_ctrl, TB6612_PIN_PWMA, pwm_level);
}

static bool motor_gate_allow_run_read(void)
{
    bsp_io_level_t gate_level = BSP_IO_LEVEL_HIGH;

    R_IOPORT_PinRead(&g_ioport_ctrl, MOTOR_RUN_GATE_PIN, &gate_level);

    return (BSP_IO_LEVEL_LOW != gate_level);
}

static void encoder_poll_1ms(void)
{
    static const int8_t quad_table[16] =
    {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
    };

    bsp_io_level_t a_level = BSP_IO_LEVEL_LOW;
    bsp_io_level_t b_level = BSP_IO_LEVEL_LOW;
    uint8_t curr_ab = 0U;
    uint8_t idx = 0U;

    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_A, &a_level);
    R_IOPORT_PinRead(&g_ioport_ctrl, ENC_PIN_B, &b_level);

    curr_ab = (uint8_t) (((uint8_t) a_level << 1U) | (uint8_t) b_level);
    idx = (uint8_t) ((g_encoder_prev_ab << 2U) | curr_ab);
    g_encoder_count += quad_table[idx];
    g_encoder_prev_ab = curr_ab;
}

static void conveyor_control_update(void)
{
    static int32_t s_speed_acc_cnt = 0;
    static uint32_t s_speed_acc_ms = 0U;
    const float dt_s = ((float) CONTROL_PERIOD_MS) / 1000.0f;
    float cmd_mps = (g_conveyor.run_cmd) ? TARGET_BELT_SPEED_MPS : 0.0f;
    float ramp_step = MAX_ACCEL_MPS2 * dt_s;
    int32_t delta_cnt = g_encoder_count - g_encoder_count_last;
    float motor_rps = g_conveyor.motor_rpm_meas / 60.0f;
    float ff_term = 0.0f;
    float err_mps = 0.0f;
    float duty = 0.0f;
    float p_term = 0.0f;
    float i_candidate = 0.0f;
    int32_t abs_delta_cnt = delta_cnt;

    g_encoder_count_last = g_encoder_count;

    if (abs_delta_cnt < 0)
    {
        abs_delta_cnt = -abs_delta_cnt;
    }

    if (motor_rps < 0.0f)
    {
        motor_rps = -motor_rps;
    }

    s_speed_acc_cnt += delta_cnt;
    s_speed_acc_ms += CONTROL_PERIOD_MS;
    if (s_speed_acc_ms >= SPEED_MEAS_WINDOW_MS)
    {
        float win_dt_s = ((float) s_speed_acc_ms) / 1000.0f;
        int32_t win_abs_cnt = s_speed_acc_cnt;

        if (win_abs_cnt < 0)
        {
            win_abs_cnt = -win_abs_cnt;
        }

        motor_rps = ((float) win_abs_cnt) / (ENCODER_CPR * win_dt_s);

        s_speed_acc_cnt = 0;
        s_speed_acc_ms = 0U;
    }

    if (ENABLE_ENCODER_STALL_FALLBACK)
    {
        /* Treat encoder as stalled only when we saw zero pulses for a while. At target speed we expect ~1 count/20 ms,
         * so using <=1 here caused false stalls and periodic torque drop. */
        if ((abs_delta_cnt == 0) && (cmd_mps > 0.01f))
        {
            g_encoder_stall_ms += CONTROL_PERIOD_MS;
        }
        else
        {
            g_encoder_stall_ms = 0U;
        }

        g_open_loop_fallback = (g_encoder_stall_ms >= ENCODER_STALL_TIMEOUT_MS);
    }
    else
    {
        g_open_loop_fallback = false;
        g_encoder_stall_ms = 0U;
    }

    g_conveyor.motor_rpm_meas = motor_rps * 60.0f;
    g_conveyor.belt_speed_meas_mps = motor_rps_to_belt_mps(motor_rps);
    {
        float meas_mps = g_conveyor.belt_speed_meas_mps;
        float max_step = SPEED_DISP_MAX_SLEW_MPS2 * dt_s;
        float delta = 0.0f;

        if (meas_mps < 0.0f)
        {
            meas_mps = -meas_mps;
        }

        meas_mps = clampf(meas_mps, 0.0f, SPEED_VALID_MAX_MPS);

        if (meas_mps < SPEED_ZERO_DEADBAND_MPS)
        {
            meas_mps = 0.0f;
        }

        delta = meas_mps - g_belt_speed_disp_mps;
        delta = clampf(delta, -max_step, max_step);
        g_belt_speed_disp_mps += delta;

        g_belt_speed_disp_mps += SPEED_DISPLAY_FILTER_ALPHA * (meas_mps - g_belt_speed_disp_mps);
        g_belt_speed_disp_mps = clampf(g_belt_speed_disp_mps, 0.0f, SPEED_VALID_MAX_MPS);
    }

    if (!g_conveyor.run_cmd)
    {
        /* Keep controller active and ramp target to zero for smooth stop when gate is LOW. */
        if (g_conveyor.belt_speed_ref_mps <= 0.001f)
        {
            g_open_loop_fallback = false;
            g_encoder_stall_ms = 0U;
            g_belt_speed_disp_mps = 0.0f;
        }
    }

#if !USE_SPEED_CLOSED_LOOP
    {
        float duty_ol = 0.0f;
        float min_run_duty = ((float) PWM_MIN_RUN_PERMIL) / 1000.0f;
        float max_duty = ((float) PWM_MAX_PERMIL) / 1000.0f;

        if (g_conveyor.belt_speed_ref_mps < cmd_mps)
        {
            g_conveyor.belt_speed_ref_mps += ramp_step;
            if (g_conveyor.belt_speed_ref_mps > cmd_mps)
            {
                g_conveyor.belt_speed_ref_mps = cmd_mps;
            }
        }
        else
        {
            g_conveyor.belt_speed_ref_mps -= ramp_step;
            if (g_conveyor.belt_speed_ref_mps < cmd_mps)
            {
                g_conveyor.belt_speed_ref_mps = cmd_mps;
            }
        }

        g_conveyor.integral = 0.0f;
        g_open_loop_fallback = false;
        g_encoder_stall_ms = 0U;

        duty_ol = (PWM_FF_BASE_PERMIL + (PWM_FF_GAIN_PER_MPS_PERMIL * g_conveyor.belt_speed_ref_mps)) / 1000.0f;

        if (cmd_mps <= 0.001f)
        {
            duty_ol = 0.0f;
        }
        else
        {
            if (g_conveyor.run_cmd && (duty_ol < min_run_duty))
            {
                duty_ol = min_run_duty;
            }
            duty_ol = clampf(duty_ol, 0.0f, max_duty);
        }

        g_conveyor.pwm_permil = (uint16_t) (duty_ol * 1000.0f);
        g_motor_is_running = (g_conveyor.pwm_permil > 0U);
        return;
    }
#endif

    if (g_conveyor.belt_speed_ref_mps < cmd_mps)
    {
        g_conveyor.belt_speed_ref_mps += ramp_step;
        if (g_conveyor.belt_speed_ref_mps > cmd_mps)
        {
            g_conveyor.belt_speed_ref_mps = cmd_mps;
        }
    }
    else
    {
        g_conveyor.belt_speed_ref_mps -= ramp_step;
        if (g_conveyor.belt_speed_ref_mps < cmd_mps)
        {
            g_conveyor.belt_speed_ref_mps = cmd_mps;
        }
    }

    err_mps = g_conveyor.belt_speed_ref_mps - g_conveyor.belt_speed_meas_mps;

    ff_term = (PWM_FF_BASE_PERMIL + (PWM_FF_GAIN_PER_MPS_PERMIL * g_conveyor.belt_speed_ref_mps)) / 1000.0f;
    p_term = PI_CTRL_KP * err_mps;
    i_candidate = g_conveyor.integral + PI_CTRL_KI * err_mps * dt_s;

    duty = ff_term + p_term + i_candidate;
    if (duty > 1.0f)
    {
        duty = 1.0f;
        if (err_mps < 0.0f)
        {
            g_conveyor.integral = i_candidate;
        }
    }
    else if (duty < 0.0f)
    {
        duty = 0.0f;
        if (err_mps > 0.0f)
        {
            g_conveyor.integral = i_candidate;
        }
    }
    else
    {
        g_conveyor.integral = i_candidate;
    }

    g_conveyor.integral = clampf(g_conveyor.integral, -0.4f, 0.4f);

    if (g_conveyor.belt_speed_ref_mps <= 0.001f)
    {
        duty = 0.0f;
        g_conveyor.integral = 0.0f;
        g_open_loop_fallback = false;
        g_encoder_stall_ms = 0U;
    }
    else
    {
        float min_run_duty = ((float) PWM_MIN_RUN_PERMIL) / 1000.0f;
        float max_duty = ((float) PWM_MAX_PERMIL) / 1000.0f;
        if (g_conveyor.run_cmd && (duty < min_run_duty))
        {
            duty = min_run_duty;
        }
        duty = clampf(duty, 0.0f, max_duty);
    }

    g_conveyor.pwm_permil = (uint16_t) (duty * 1000.0f);
    g_motor_is_running = ((g_conveyor.pwm_permil > 0U) &&
                          ((g_conveyor.belt_speed_ref_mps > 0.01f) || (g_conveyor.belt_speed_meas_mps > 0.01f)));
}

static void conveyor_display_init(void)
{
    OLED_Init();
    OLED_Clear();
    OLED_ColorTurn(0);
    OLED_DisplayTurn(0);
}

static void conveyor_display_title_once(void)
{
    OLED_ClearBuffer();
    OLED_ShowString(10, 16, (u8 *) "CONVEYOR", 16, 1);
    OLED_ShowString(36, 36, (u8 *) "BELT", 16, 1);
    OLED_Refresh();
}

void conveyor_display_off(void)
{
    OLED_DisPlay_Off();
}

static void conveyor_auto_profile_update(void)
{
    bool profile_run = true;
    bool gate_raw_allow = motor_gate_allow_run_read();

    /* Debounce gate both ways:
     * P001 LOW  for debounce window -> stop command
     * P001 HIGH (or floating with pull-up) for debounce window -> run command */
    if (gate_raw_allow)
    {
        g_gate_low_ms = 0U;
        if (g_gate_high_ms < GATE_STOP_DEBOUNCE_MS)
        {
            g_gate_high_ms += 1U;
        }

        if (g_gate_high_ms >= GATE_STOP_DEBOUNCE_MS)
        {
            g_motor_gate_allow_run = true;
        }
    }
    else
    {
        g_gate_high_ms = 0U;
        if (g_gate_low_ms < GATE_STOP_DEBOUNCE_MS)
        {
            g_gate_low_ms += 1U;
        }

        if (g_gate_low_ms >= GATE_STOP_DEBOUNCE_MS)
        {
            g_motor_gate_allow_run = false;
        }
    }

#if DEMO_AUTO_CYCLE
    uint32_t cycle = g_uptime_ms % 12000U;
    profile_run = (cycle < 8000U);
#else
    profile_run = true;
#endif

    g_conveyor.run_cmd = (profile_run && g_motor_gate_allow_run);
}

/*******************************************************************************************************************//**
 * main() is generated by the RA Configuration editor and is used to generate threads if an RTOS is used.  This function
 * is called by main() when no RTOS is used.
 **********************************************************************************************************************/
void hal_entry(void)
{
    uint16_t tick_100us = 0U;

    app_peripheral_init();
    motor_hw_init();
#if ENABLE_OLED_UI
    conveyor_display_init();
    conveyor_display_title_once();
#endif

    while (1)
    {
        R_BSP_SoftwareDelay(MAIN_TICK_US, BSP_DELAY_UNITS_MICROSECONDS);
        tick_100us++;

        /* 10kHz stable carrier PWM. */
        motor_apply_output(g_conveyor.pwm_permil);

        if ((tick_100us % 10U) == 0U)
        {
            g_uptime_ms++;

            conveyor_auto_profile_update();
            encoder_poll_1ms();

            if ((g_uptime_ms % CONTROL_PERIOD_MS) == 0U)
            {
                conveyor_control_update();
            }
        }
    }
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}



/*******************************************************************************************************************//**
 * This function is called at various points during the startup process.  This implementation uses the event that is
 * called right before main() to set up the pins.
 *
 * @param[in]  event    Where at in the start up process the code is currently at
 **********************************************************************************************************************/
void R_BSP_WarmStart(bsp_warm_start_event_t event)
{
    if (BSP_WARM_START_RESET == event)
    {
#if BSP_FEATURE_FLASH_LP_VERSION != 0

        /* Enable reading from data flash. */
        R_FACI_LP->DFLCTL = 1U;

        /* Would normally have to wait tDSTOP(6us) for data flash recovery. Placing the enable here, before clock and
         * C runtime initialization, should negate the need for a delay since the initialization will typically take more than 6us. */
#endif
    }

    if (BSP_WARM_START_POST_C == event)
    {
        /* C runtime environment and system clocks are setup. */

        /* Configure pins. */
        R_IOPORT_Open (&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);

#if BSP_CFG_SDRAM_ENABLED

        /* Setup SDRAM and initialize it. Must configure pins first. */
        R_BSP_SdramInit(true);
#endif
    }
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ();

/* Trustzone Secure Projects require at least one nonsecure callable function in order to build (Remove this if it is not required to build). */
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable ()
{

}
FSP_CPP_FOOTER

#endif
