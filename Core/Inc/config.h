#ifndef __CONFIG_H__
#define __CONFIG_H__

/* ================================================================
 *  FEATURE FLAGS - Enable/Disable System Features
 * ================================================================ */
// #define SDDEBUG           //undefine to disable debug
// #define CAL_ZERO_ANGLE  //undefine to disable zero electrical angle calibration
// #define TIMING          //undefine to disable loop timing
#define CAN_OT_FAULT    //undefine to disable CAN OVERTIME Fault
// #define RMSOCP          //undefine to disable RMS overcurrent protection
// #define WAIT_GATE_READY //undefine to not wait for gate ready signal
// #define OVERRIDE_OCP    //define to enable hardware ocp override
// #define CAN_CONFIG      //define to enable use CAN to change variables
// #define SIXSTEP         //define to enable six step comutation
#define VQ_LEQ_0        //define to use Vq less than 0
#define MIDDLE_CLAMP    //define to use middle clamp
#define SVPWM           //define to use SVPWM
#define Decouopling    //define to enable decoupling in current controller
#define ANTI_WINDUP     //define to enable anti windup in PID controllers
#define FIELD_WEAKENING //define to enable field weakening control
// #define FIELD_WEAKENING_ANGLE // define to enable field weakening angle control
// #define MTPA            //define to enable MTPA control
// #define DISABLE_MOT_OT //define to disable motor overtemperature fault
// #define OVERSPEED_PROT  //define to enable overspeed protection
#define FW_STARTUP_ID_FIX   //define to enable fixed d-axis current during field weakening startup
// #define PERMANENT_FLUX      //define to always apply maximum flux weakening current

/* ================================================================
 *  MOTOR SELECTION - Choose Motor Type
 * ================================================================ */
// #define MOT_FL
#define MOT_RR
// #define MOT_RL
// #define MOT_CAL

/* ================================================================
 *  PROTECTION - Safety Thresholds
 * ================================================================ */
#define SOFTOCP 70
#define ACAOCP 85 //Amp
#define MOVRMSOCP (uint32_t)36000000 //should be 10000*OCP^2
#define MOS_OTP 1000        //should be 10 times otp temp in deg C
#define MOT_OTP 900         //should be 10 times otp temp in deg C
#define MOT_UTP 50        //should be 10 times utp temp in deg C
#define ENC_UV 50           //should be 3~5% Encoder adc full range

/* ================================================================
 *  TIMING CONSTANTS - Protection Timeouts
 * ================================================================ */
#define HW_OC_TIME          2300    //should be in pwm cycles, 2300 is 100ms
#define SOFT_OC_TIME        50      //should be in pwm cycles, 50 is 2ms
#define ENC_TIME            50      //should be in pwm cycles, 50 is 2ms

/* ================================================================
 *  PID CONTROLLERS - Current Loop Gains
 * ================================================================ */
// General PID parameters
#define PID_P 1.0f
#define PID_I 0.02f
#define PID_D 0.0f
#define PID_RAMP 100000.0f
#define PID_LIMIT 20.0f
// #define RAMP_TIME 1.0f

// Q-axis current controller
#define QKP             0.8f    // Q-axis proportional gain (// 2.3f //4.8f)
#define QKI             200.0f  // Q-axis integral gain (// 724.5f //3.0f)
#define QKD             0.0f    // Q-axis derivative gain

// D-axis current controller
#define DKP             3.0f    // D-axis proportional gain (// 2.3f //3.84f)
#define DKI             25.0f   // D-axis integral gain (// 724.5f //3.0f)
#define DKD             0.0f    // D-axis derivative gain

// Field weakening controller
#define FWKP            0.0f    // FW proportional gain
#define FWKI            1000.0f // FW integral gain

// DC current controller
#define DCKP            1.0f    // DC current proportional gain
#define DCKI            1.0f    // DC current integral gain

//MATLAB tuned PID values
// #define QKP             0.226f
// #define DKP             0.1725f
// #define QKI             72.45f
// #define DKI             72.45f

// #define QKP             1.5f
// #define DKP             0.9f
// #define QKI             0.625f
// #define DKI             0.625f
// #define QKD             0.0f
// #define DKD             0.0f

/* ================================================================
 *  FIELD WEAKENING - Flux Control Parameters
 * ================================================================ */
#define MAX_FLUX_ID         30.0f   //60A                                                                                                          .0f   //40A
#define MAX_TORQUE_FW_ID    30.0f   //40A
#define MINIMUM_FW_ID       2.0f    //2A

/* ================================================================
 *  DERATE CONTROL - Power Reduction at High Speed/Temperature
 * ================================================================ */
#define DERATE_START        9000    //should be in RPM, 1000 is 1000RPM
#define DERATE_END          13000   //should be in RPM, 12000 is 12000RPM
#define DERATE_A            1 / (DERATE_START - DERATE_END)
#define DERATE_B            DERATE_END / (DERATE_START - DERATE_END)
#define RAMP_TIME_DERATE    5       //time from 0 to 100 percent
#define T_DERATE_START      700      //should be 10 times in deg C, 500 is 50 deg C
#define T_DERATE_END        850      //should be 10 times in deg C, 800 is 80 deg C

/* ================================================================
 *  PWM FREQUENCY CONFIGURATION
 * ================================================================ */
#define FREQ_23KHZ              // Select 23kHz PWM frequency

#ifdef FREQ_11KHZ
#define FREQ    11000 // Hz - PWM frequency
#define CCR     10908 // 11000Hz PWM frequency
#define QTF     0.0008f // Q-axis filter time constant
#define DTF     0.0008f // D-axis filter time constant
#define ABCTF   0.00008f // ABC current filter time constant
#define RPMTF   0.1f // RPM filter time constant
#define DCTF    0.1f // DC current filter time constant
#endif

#ifdef FREQ_13KHZ
#define FREQ    13000 // Hz - PWM frequency
#define CCR     9230 // 13000Hz PWM frequency
#define QTF     0.000677f
#define DTF     0.000677f
#define ABCTF   0.0000677f
#define RPMTF   0.846f
#define DCTF    0.846f
#endif

#ifdef FREQ_23KHZ
#define FREQ    23000 // Hz - PWM frequency
#define DEF_CCR 5127 // 23000Hz PWM frequency
#define QTF     0.00003826f
#define DTF     0.00003826f
#define ABCTF   0.00003826f
#define RPMTF   0.04783f
#define DCTF    0.04783f
#define FWTF    0.04783f
#endif

/* ================================================================
 *  MOTOR-SPECIFIC PARAMETERS - Configuration per Motor Type
 * ================================================================ */
#ifdef CAL_ZERO_ANGLE
#define ZERO_ELECTRIC_ANGLE 0.0f //should be in radians
#define MOT_CURR 1.0f
#define FILENAME "MOT_CAL_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#else
#ifdef MOT_CAL
#define ZERO_ELECTRIC_ANGLE 5.75f //should be in radians
#define MOT_CURR 1.084f
#define FILENAME "MOT_CAL_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#endif
#ifdef MOT_FL
#define ZERO_ELECTRIC_ANGLE 5.75f //should be in radians
#define MOT_CURR 1.156f
#define FILENAME "MOT_FL_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#endif
#ifdef MOT_RR
#define ZERO_ELECTRIC_ANGLE 1.33f //should be in radians
#define MOT_CURR 0.907f
#define FILENAME "MOT_RR_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#endif
#ifdef MOT_RL
#define ZERO_ELECTRIC_ANGLE 4.86f //should be in radians
#define MOT_CURR 0.991f
#define FILENAME "MOT_RL_%04d%02d%02d_%02d%02d%02d_NEW_V2_4.bin"
#endif
#endif

/* ================================================================
 *  System Configuration Constants
 * ================================================================ */
#define MAX_ANGLE_VALUE 4096
#define CODE_VER 0x3
#define NUM_TAPS 558

/* ================================================================
 *  ADC Conversion Constants (Hardware-specific)
 * ================================================================ */
#define ACAPLSB -0.1031436f   // ACAPLSB = 3.3/15.626e-3/adc1_range
#define DCVPLSB 0.00897f       // DCVPLSB = 451*3.3/adc3_range
#define DCAPLSB 0.0402930f     // DCAPLSB = 3.3/20e-3/adc1_range

/* ================================================================
 *  Mathematical Constants
 * ================================================================ */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ================================================================
 *  Mechanical Constants
 * ================================================================ */
#define TORQUE_CONSTANT 0.291f    // Nm/A
#define MAX_TORQUE 25.0f

/* ================================================================
 *  Debug & Testing
 * ================================================================ */
#define TEST_FILE_PATH "Test.bin"

#endif /* __CONFIG_H__ */