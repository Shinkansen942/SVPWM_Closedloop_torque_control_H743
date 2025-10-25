/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PLL.h
 *
 * Code generated for Simulink model 'PLL'.
 *
 * Model version                  : 1.5
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sat Oct 25 22:52:04 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->STM32Processor
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef PLL_h_
#define PLL_h_
#ifndef PLL_COMMON_INCLUDES_
#define PLL_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* PLL_COMMON_INCLUDES_ */

#include "PLL_types.h"
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay_DSTATE;                 /* '<S79>/Delay' */
  real_T Delay_DSTATE_h;               /* '<S103>/Delay' */
  real_T Integrator_DSTATE;            /* '<S48>/Integrator' */
  real_T Delay_DSTATE_p;               /* '<S104>/Delay' */
  real_T Delay_DSTATE_f;               /* '<S10>/Delay' */
  real_T Delay_DSTATE_f1;              /* '<S96>/Delay' */
  real_T Delay2_DSTATE;                /* '<S91>/Delay2' */
  real_T Delay3_DSTATE;                /* '<S91>/Delay3' */
  real_T Delay2_DSTATE_p;              /* '<S90>/Delay2' */
  real_T Delay3_DSTATE_b;              /* '<S90>/Delay3' */
  real_T Delay_DSTATE_i;               /* '<S99>/Delay' */
  real_T Delay1_DSTATE;                /* '<S83>/Delay1' */
  real_T Delay_DSTATE_o;               /* '<S83>/Delay' */
  real_T Delay1_DSTATE_h;              /* '<S82>/Delay1' */
  real_T Delay_DSTATE_pm;              /* '<S82>/Delay' */
  int8_T Integrator_PrevResetState;    /* '<S48>/Integrator' */
  boolean_T icLoad;                    /* '<S79>/Delay' */
  boolean_T icLoad_d;                  /* '<S104>/Delay' */
  boolean_T icLoad_l;                  /* '<S10>/Delay' */
  boolean_T icLoad_dx;                 /* '<S96>/Delay' */
  boolean_T icLoad_c;                  /* '<S99>/Delay' */
  boolean_T icLoad_a;                  /* '<S86>/Delay' */
  boolean_T icLoad_k;                  /* '<S89>/Delay' */
} DW_PLL_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Delay_Reset_ZCE;          /* '<S103>/Delay' */
  ZCSigState Delay2_Reset_ZCE;         /* '<S91>/Delay2' */
  ZCSigState Delay3_Reset_ZCE;         /* '<S91>/Delay3' */
  ZCSigState Delay2_Reset_ZCE_p;       /* '<S90>/Delay2' */
  ZCSigState Delay3_Reset_ZCE_b;       /* '<S90>/Delay3' */
  ZCSigState Delay1_Reset_ZCE;         /* '<S83>/Delay1' */
  ZCSigState Delay_Reset_ZCE_i;        /* '<S83>/Delay' */
  ZCSigState Delay1_Reset_ZCE_e;       /* '<S82>/Delay1' */
  ZCSigState Delay_Reset_ZCE_p;        /* '<S82>/Delay' */
} PrevZCX_PLL_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: sine_table_values_Value
   * Referenced by: '<S106>/sine_table_values'
   */
  real32_T sine_table_values_Value[1002];
} ConstP_PLL_T;

/* Real-time Model Data Structure */
struct tag_RTM_PLL_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_PLL_T PLL_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_PLL_T PLL_PrevZCX;

/* Constant parameters (default storage) */
extern const ConstP_PLL_T PLL_ConstP;

/* Model entry point functions */
extern void PLL_initialize(void);
extern void PLL_terminate(void);

/* Customized model step function */
extern void PLL_step(real_T sine_in, real_T cosine_in, boolean_T rst_in,
                     real32_T *rad_out, real32_T *deg_sec_out);

/* Real-time Model object */
extern RT_MODEL_PLL_T *const PLL_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Data Type Duplicate' : Unused code path elimination
 * Block '<S1>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S1>/Data Type Conversion' : Unused code path elimination
 * Block '<S1>/Data Type Conversion1' : Unused code path elimination
 * Block '<S2>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/Data Type Duplicate' : Unused code path elimination
 * Block '<S3>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/Data Type Duplicate' : Unused code path elimination
 * Block '<S13>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S13>/Data Type Propagation' : Unused code path elimination
 * Block '<S5>/Data Type Duplicate' : Unused code path elimination
 * Block '<S5>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S66>/I1CnstDT' : Unused code path elimination
 * Block '<S66>/I2CnstDT' : Unused code path elimination
 * Block '<S74>/Bias' : Unused code path elimination
 * Block '<S74>/Bias1' : Unused code path elimination
 * Block '<S74>/Constant' : Unused code path elimination
 * Block '<S74>/Data Type Duplicate' : Unused code path elimination
 * Block '<S74>/Data Type Propagation' : Unused code path elimination
 * Block '<S74>/Data Type Propagation1' : Unused code path elimination
 * Block '<S74>/Divide' : Unused code path elimination
 * Block '<S74>/Divide1' : Unused code path elimination
 * Block '<S74>/OD1Cnst' : Unused code path elimination
 * Block '<S74>/OD1CnstDT' : Unused code path elimination
 * Block '<S74>/Product' : Unused code path elimination
 * Block '<S74>/Unary Minus' : Unused code path elimination
 * Block '<S66>/MaxFreqCnst' : Unused code path elimination
 * Block '<S66>/OD1CnstDT' : Unused code path elimination
 * Block '<S66>/OD2CnstDT' : Unused code path elimination
 * Block '<S75>/Bias' : Unused code path elimination
 * Block '<S75>/Constant' : Unused code path elimination
 * Block '<S75>/Data Type Duplicate' : Unused code path elimination
 * Block '<S75>/Data Type Propagation' : Unused code path elimination
 * Block '<S75>/Data Type Propagation1' : Unused code path elimination
 * Block '<S75>/Data Type Propagation2' : Unused code path elimination
 * Block '<S75>/Divide' : Unused code path elimination
 * Block '<S75>/OD1Cnst' : Unused code path elimination
 * Block '<S75>/OD1CnstDT' : Unused code path elimination
 * Block '<S75>/OD1CnstDT1' : Unused code path elimination
 * Block '<S75>/Product' : Unused code path elimination
 * Block '<S75>/Product1' : Unused code path elimination
 * Block '<S75>/SampleTime' : Unused code path elimination
 * Block '<S76>/Const' : Unused code path elimination
 * Block '<S76>/Data Type Propagation' : Unused code path elimination
 * Block '<S76>/Data Type Propagation1' : Unused code path elimination
 * Block '<S76>/MaxFreqDT' : Unused code path elimination
 * Block '<S76>/Product' : Unused code path elimination
 * Block '<S67>/Data Type Duplicate' : Unused code path elimination
 * Block '<S68>/Bias' : Unused code path elimination
 * Block '<S68>/Constant' : Unused code path elimination
 * Block '<S68>/Data Type Propagation' : Unused code path elimination
 * Block '<S68>/Data Type Propagation1' : Unused code path elimination
 * Block '<S68>/Divide' : Unused code path elimination
 * Block '<S68>/Product' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S69>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S80>/Data Type Duplicate' : Unused code path elimination
 * Block '<S81>/Data Type Duplicate' : Unused code path elimination
 * Block '<S82>/Data Type Duplicate' : Unused code path elimination
 * Block '<S82>/Data Type Propagation' : Unused code path elimination
 * Block '<S82>/Data Type Propagation1' : Unused code path elimination
 * Block '<S83>/Data Type Duplicate' : Unused code path elimination
 * Block '<S83>/Data Type Propagation' : Unused code path elimination
 * Block '<S83>/Data Type Propagation1' : Unused code path elimination
 * Block '<S72>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Duplicate' : Unused code path elimination
 * Block '<S90>/Data Type Propagation' : Unused code path elimination
 * Block '<S91>/Data Type Duplicate' : Unused code path elimination
 * Block '<S91>/Data Type Propagation' : Unused code path elimination
 * Block '<S92>/Data Type Duplicate' : Unused code path elimination
 * Block '<S93>/Data Type Duplicate' : Unused code path elimination
 * Block '<S73>/Bias' : Unused code path elimination
 * Block '<S73>/Constant' : Unused code path elimination
 * Block '<S73>/Data Type Propagation' : Unused code path elimination
 * Block '<S73>/Data Type Propagation1' : Unused code path elimination
 * Block '<S73>/Divide' : Unused code path elimination
 * Block '<S73>/Product' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S103>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S103>/Data Type Propagation' : Unused code path elimination
 * Block '<S104>/BooleanTypeCast' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S104>/Data Type Duplicate3' : Unused code path elimination
 * Block '<S104>/Data Type Propagation' : Unused code path elimination
 * Block '<S104>/InpGainDatatype' : Unused code path elimination
 * Block '<S104>/OutGainDatatype' : Unused code path elimination
 * Block '<S104>/Reset' : Unused code path elimination
 * Block '<S106>/Data Type Duplicate' : Unused code path elimination
 * Block '<S106>/Data Type Propagation' : Unused code path elimination
 * Block '<S111>/Data Type Duplicate' : Unused code path elimination
 * Block '<S112>/Data Type Duplicate' : Unused code path elimination
 * Block '<Root>/Scope1' : Unused code path elimination
 * Block '<Root>/Scope2' : Unused code path elimination
 * Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S2>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S12>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S12>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S13>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S13>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S13>/Data Type Conversion2' : Eliminate redundant data type conversion
 * Block '<S5>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S67>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S80>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S81>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S92>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S93>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S106>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S108>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S113>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S2>/One' : Unused code path elimination
 * Block '<S2>/Reset' : Unused code path elimination
 * Block '<S2>/Sum' : Unused code path elimination
 * Block '<S2>/UseInputPort' : Unused code path elimination
 * Block '<S2>/a' : Unused code path elimination
 * Block '<S67>/FilterConstant' : Unused code path elimination
 * Block '<S67>/OneMinusFilterConstant' : Unused code path elimination
 * Block '<S67>/Reset' : Unused code path elimination
 * Block '<S67>/UseInputPort' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'PLL'
 * '<S1>'   : 'PLL/PLL with Feed Forward'
 * '<S2>'   : 'PLL/PLL with Feed Forward/IIR Filter'
 * '<S3>'   : 'PLL/PLL with Feed Forward/Normalize'
 * '<S4>'   : 'PLL/PLL with Feed Forward/PID Controller'
 * '<S5>'   : 'PLL/PLL with Feed Forward/SpeedFeedforward'
 * '<S6>'   : 'PLL/PLL with Feed Forward/Subsystem'
 * '<S7>'   : 'PLL/PLL with Feed Forward/VCO'
 * '<S8>'   : 'PLL/PLL with Feed Forward/IIR Filter/IIR Filter'
 * '<S9>'   : 'PLL/PLL with Feed Forward/IIR Filter/IIR Filter/Low-pass'
 * '<S10>'  : 'PLL/PLL with Feed Forward/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S11>'  : 'PLL/PLL with Feed Forward/Normalize/Compare To Zero'
 * '<S12>'  : 'PLL/PLL with Feed Forward/Normalize/If Action Subsystem'
 * '<S13>'  : 'PLL/PLL with Feed Forward/Normalize/Subsystem'
 * '<S14>'  : 'PLL/PLL with Feed Forward/PID Controller/Anti-windup'
 * '<S15>'  : 'PLL/PLL with Feed Forward/PID Controller/D Gain'
 * '<S16>'  : 'PLL/PLL with Feed Forward/PID Controller/External Derivative'
 * '<S17>'  : 'PLL/PLL with Feed Forward/PID Controller/Filter'
 * '<S18>'  : 'PLL/PLL with Feed Forward/PID Controller/Filter ICs'
 * '<S19>'  : 'PLL/PLL with Feed Forward/PID Controller/I Gain'
 * '<S20>'  : 'PLL/PLL with Feed Forward/PID Controller/Ideal P Gain'
 * '<S21>'  : 'PLL/PLL with Feed Forward/PID Controller/Ideal P Gain Fdbk'
 * '<S22>'  : 'PLL/PLL with Feed Forward/PID Controller/Integrator'
 * '<S23>'  : 'PLL/PLL with Feed Forward/PID Controller/Integrator ICs'
 * '<S24>'  : 'PLL/PLL with Feed Forward/PID Controller/N Copy'
 * '<S25>'  : 'PLL/PLL with Feed Forward/PID Controller/N Gain'
 * '<S26>'  : 'PLL/PLL with Feed Forward/PID Controller/P Copy'
 * '<S27>'  : 'PLL/PLL with Feed Forward/PID Controller/Parallel P Gain'
 * '<S28>'  : 'PLL/PLL with Feed Forward/PID Controller/Reset Signal'
 * '<S29>'  : 'PLL/PLL with Feed Forward/PID Controller/Saturation'
 * '<S30>'  : 'PLL/PLL with Feed Forward/PID Controller/Saturation Fdbk'
 * '<S31>'  : 'PLL/PLL with Feed Forward/PID Controller/Sum'
 * '<S32>'  : 'PLL/PLL with Feed Forward/PID Controller/Sum Fdbk'
 * '<S33>'  : 'PLL/PLL with Feed Forward/PID Controller/Tracking Mode'
 * '<S34>'  : 'PLL/PLL with Feed Forward/PID Controller/Tracking Mode Sum'
 * '<S35>'  : 'PLL/PLL with Feed Forward/PID Controller/Tsamp - Integral'
 * '<S36>'  : 'PLL/PLL with Feed Forward/PID Controller/Tsamp - Ngain'
 * '<S37>'  : 'PLL/PLL with Feed Forward/PID Controller/postSat Signal'
 * '<S38>'  : 'PLL/PLL with Feed Forward/PID Controller/preInt Signal'
 * '<S39>'  : 'PLL/PLL with Feed Forward/PID Controller/preSat Signal'
 * '<S40>'  : 'PLL/PLL with Feed Forward/PID Controller/Anti-windup/Passthrough'
 * '<S41>'  : 'PLL/PLL with Feed Forward/PID Controller/D Gain/Disabled'
 * '<S42>'  : 'PLL/PLL with Feed Forward/PID Controller/External Derivative/Disabled'
 * '<S43>'  : 'PLL/PLL with Feed Forward/PID Controller/Filter/Disabled'
 * '<S44>'  : 'PLL/PLL with Feed Forward/PID Controller/Filter ICs/Disabled'
 * '<S45>'  : 'PLL/PLL with Feed Forward/PID Controller/I Gain/Internal Parameters'
 * '<S46>'  : 'PLL/PLL with Feed Forward/PID Controller/Ideal P Gain/Passthrough'
 * '<S47>'  : 'PLL/PLL with Feed Forward/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S48>'  : 'PLL/PLL with Feed Forward/PID Controller/Integrator/Discrete'
 * '<S49>'  : 'PLL/PLL with Feed Forward/PID Controller/Integrator ICs/Internal IC'
 * '<S50>'  : 'PLL/PLL with Feed Forward/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S51>'  : 'PLL/PLL with Feed Forward/PID Controller/N Gain/Disabled'
 * '<S52>'  : 'PLL/PLL with Feed Forward/PID Controller/P Copy/Disabled'
 * '<S53>'  : 'PLL/PLL with Feed Forward/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S54>'  : 'PLL/PLL with Feed Forward/PID Controller/Reset Signal/External Reset'
 * '<S55>'  : 'PLL/PLL with Feed Forward/PID Controller/Saturation/Passthrough'
 * '<S56>'  : 'PLL/PLL with Feed Forward/PID Controller/Saturation Fdbk/Disabled'
 * '<S57>'  : 'PLL/PLL with Feed Forward/PID Controller/Sum/Sum_PI'
 * '<S58>'  : 'PLL/PLL with Feed Forward/PID Controller/Sum Fdbk/Disabled'
 * '<S59>'  : 'PLL/PLL with Feed Forward/PID Controller/Tracking Mode/Disabled'
 * '<S60>'  : 'PLL/PLL with Feed Forward/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S61>'  : 'PLL/PLL with Feed Forward/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S62>'  : 'PLL/PLL with Feed Forward/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S63>'  : 'PLL/PLL with Feed Forward/PID Controller/postSat Signal/Forward_Path'
 * '<S64>'  : 'PLL/PLL with Feed Forward/PID Controller/preInt Signal/Internal PreInt'
 * '<S65>'  : 'PLL/PLL with Feed Forward/PID Controller/preSat Signal/Forward_Path'
 * '<S66>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/GainSystem'
 * '<S67>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IIR Filter'
 * '<S68>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/InpFilterFunc'
 * '<S69>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod'
 * '<S70>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/Nofunc'
 * '<S71>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/NofuncSpd'
 * '<S72>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod'
 * '<S73>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/SpdFilterFunc'
 * '<S74>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/GainSystem/IGains'
 * '<S75>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/GainSystem/ODGains'
 * '<S76>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/GainSystem/OutDTSetup'
 * '<S77>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IIR Filter/IIR Filter'
 * '<S78>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IIR Filter/IIR Filter/Low-pass'
 * '<S79>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S80>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter1'
 * '<S81>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter3'
 * '<S82>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/Integrator'
 * '<S83>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/Integrator2'
 * '<S84>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter'
 * '<S85>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S86>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S87>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter'
 * '<S88>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S89>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/IntegralMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S90>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/Differentiator'
 * '<S91>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/Differentiator1'
 * '<S92>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1'
 * '<S93>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3'
 * '<S94>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter'
 * '<S95>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S96>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S97>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter'
 * '<S98>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S99>'  : 'PLL/PLL with Feed Forward/SpeedFeedforward/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S100>' : 'PLL/PLL with Feed Forward/Subsystem/If Action Subsystem'
 * '<S101>' : 'PLL/PLL with Feed Forward/Subsystem/If Action Subsystem1'
 * '<S102>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem'
 * '<S103>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased'
 * '<S104>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Position Generator'
 * '<S105>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup'
 * '<S106>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup'
 * '<S107>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/Interpolation'
 * '<S108>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp'
 * '<S109>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype'
 * '<S110>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S111>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S112>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S113>' : 'PLL/PLL with Feed Forward/VCO/Variant Subsystem/LookupTableBased/Sine-Cosine Lookup/Sine-Cosine Lookup/datatype/datatype backpropogation'
 */
#endif                                 /* PLL_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
