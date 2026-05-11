/*
 * foc_loop.h
 *
 * Struct typedefs for grouping FOC-related global variables.
 * Phase 1: typedef only – no extern declarations yet.
 */

#ifndef INC_FOC_LOOP_H_
#define INC_FOC_LOOP_H_

#include <stdint.h>
#include "config.h"
#include "pid.h"               // for pidc_t
#include "lowpass_filter.h"    // for lpf_t

/* ================================================================
 *  Struct 1 — motor_params_t  (motor physical & system parameters)
 *
 *  Cross-file usage: FOC.c, motor_control.c, pid.c, lowpass_filter.c
 *  Fields marked 'const' are set once at initialization and must not
 *  be modified at runtime (compiler will enforce this).
 * ================================================================ */
typedef struct {
    /* --- Motor physical constants (immutable after init) --- */
    float const Rs;                    // Stator resistance  (Ohm)
    float const Ld;                    // D-axis inductance  (H)
    float const Lq;                    // Q-axis inductance  (H)
    float const flux_linkage_m;        // PM flux linkage    (Wb)
    float const electrical_constant;   // Back-EMF constant  (V/(rad/s))
    float const max_current;           // Phase current limit (A)
    int   const pole_pairs;            // Number of pole pairs
    int   const dir;                   // Rotation direction (anti clockwise direction is 1, clockwise is -1)
    float const Ts;                    // Control-loop period (s)

    /* --- Runtime-writable fields --- */
    float zero_electric_angle;   // Encoder zero-offset (rad)
    float voltage_power_supply;  // DC bus voltage      (V)
    float voltage_limit;         // Modulation voltage limit (V)
} motor_params_t;

/* ================================================================
 *  Struct 2 — foc_state_t  (FOC real-time state)
 *
 *  Cross-file usage: can_app.c, state_machine.c
 *  Most members are used only inside the FOC ISR (main.c → foc_loop.c).
 * ================================================================ */
typedef struct {
    // --- Electrical angle & position ---
    float angle_now;
    float shaft_angle;
    float zero_cross;
    float open_loop_timestamp;

    // --- Measured / filtered signals ---
    float filtered_RPM;
    float filtered_Iq;
    float filtered_Id;
    float Ia;
    uint16_t current_offset[4];
    float current_phase[3];

    // --- Controller outputs ---
    float Iq_controller_output;
    float Id_controller_output;
    float Ia_controller_output;

    // --- Control targets ---
    float target_Iq;
    float target_Id;
    float Id_fw;
    float Iq_fw;
    float Id_MTPA;

    // --- Torque request & ramp ---
    float percent_torque_requested;
    float last_percent;
    float abs_last_percent;
    float max_ramp;

    // --- Flags ---
    int     enable_hw_oc;
    uint8_t enable_dc_control;
    uint8_t fast_stop_enable;
    uint8_t run;

    // --- Hardware config ---
    int   period;                // PWM half-period (timer counts)
    int   freq;                  // Control frequency (Hz)
} foc_state_t;

/* ================================================================
 *  Struct 3 — protection_t  (sliding-window protection buffers)
 *
 *  Cross-file usage: can_app.c
 * ================================================================ */
typedef struct {
    // Hardware OC sliding window
    uint8_t  oc_buf[HW_OC_TIME];
    uint16_t oc_index;
    uint16_t oc_sum;

    // Software OC sliding window
    uint8_t  soft_oc_buf[SOFT_OC_TIME];
    uint16_t soft_oc_index;
    uint16_t soft_oc_sum;

    // Encoder error sliding window
    uint8_t  enc_buf[ENC_TIME];
    uint16_t enc_index;
    uint16_t enc_sum;

    // Moving RMS (3 phases x 10 000 samples)
    int16_t  RMS_buf[3][10000];
    uint32_t RMS_sum[3];
    uint16_t indexRMS;
} protection_t;

/* ================================================================
 *  Struct 4 — telemetry_t  (CAN telemetry / reporting)
 *
 *  Cross-file usage: can_app.c
 * ================================================================ */
typedef struct {
    // Temperatures (x10 °C)
    int16_t T_Report;
    int16_t T_Mot;
    int16_t T_MCU;
    int16_t T_U;
    int16_t T_V;
    int16_t T_W;

    // CAN report fields
    uint16_t report_status;
    uint16_t report_DCV;         // DC voltage  (x10 V)
    int16_t  report_DCA;         // DC current  (x100 A)

    // Logging variables
    int16_t IU_100;
    int16_t IV_100;
    int16_t IW_100;
} telemetry_t;

/* ================================================================
 *  Extern declarations - variables defined in main.c
 * ================================================================ */

/* --- Struct instances --- */
extern motor_params_t   motor;
extern foc_state_t      foc;
extern protection_t     prot;
extern telemetry_t      telem;

/* --- PID controller instances --- */
extern pidc_t pid_controller_current_Iq;
extern pidc_t pid_controller_current_Id;
extern pidc_t pid_controller_current_OCP;
extern pidc_t pid_controller_current_Ia;
extern pidc_t pid_controller_current_Iabc[3];

/* --- Low-pass filter instances --- */
extern lpf_t filter_current_Iq;
extern lpf_t filter_current_Id;
extern lpf_t filter_current_Iabc[3];
extern lpf_t filter_current_DC_Iabc[3];
extern lpf_t filter_RPM;
extern lpf_t filter_Idfw;

/* --- DMA ADC buffers (special memory sections) --- */
extern uint16_t DMA_ADC1_arr[4];
extern uint16_t DMA_ADC2_arr[4];
extern uint16_t DMA_ADC3_arr[6];

/* --- TIMING debug (shared with main.c while-loop) --- */
#ifdef TIMING
extern int       max_time;
extern int       min_time;
extern int       prev_time;
extern int       max_btw;
extern int       max_sdwrite;
extern uint32_t  loop_time;
extern int       indexTimer;
#endif

#endif /* INC_FOC_LOOP_H_ */
