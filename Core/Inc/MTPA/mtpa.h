/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: mtpa.h
 *
 * Code generated for Simulink model 'mtpa'.
 *
 * Model version                  : 1.10
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Tue Oct 28 16:22:31 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->STM32Processor
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef mtpa_h_
#define mtpa_h_
#ifndef mtpa_COMMON_INCLUDES_
#define mtpa_COMMON_INCLUDES_
#include <stdbool.h>
#include <stdint.h>
#include "math.h"
#endif                                 /* mtpa_COMMON_INCLUDES_ */

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  double Merge[2];                     /* '<S8>/Merge' */
} DW;

/* Invariant block signals (default storage) */
typedef struct {
  const double Gain1;                  /* '<S22>/Gain1' */
  const double Add1;                   /* '<S22>/Add1' */
} ConstB;

/* Self model data, for system '<Root>' */
struct tag_RTM {
  DW dwork;
};

extern const ConstB rtConstB;          /* constant block i/o */

/* Model entry point functions */
extern void mtpa_initialize(RT_MODEL *const rtM);

/* Customized model step function */
extern void mtpa_step(RT_MODEL *const rtM, double Torque_ref, double speed,
                      double V_DC, double *Id_ref, double *Iq_ref);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S13>/Data Type Duplicate' : Unused code path elimination
 * Block '<S32>/Data Type Duplicate' : Unused code path elimination
 * Block '<S32>/Data Type Propagation' : Unused code path elimination
 * Block '<S33>/Data Type Duplicate' : Unused code path elimination
 * Block '<S35>/Data Type Duplicate' : Unused code path elimination
 * Block '<S20>/Data Type Duplicate' : Unused code path elimination
 * Block '<S25>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S25>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S26>/Sqrt' : Unused code path elimination
 * Block '<S17>/2' : Eliminated nontunable gain of 1
 * Block '<S40>/2' : Eliminated nontunable gain of 1
 * Block '<S25>/enableInportSatLim' : Unused code path elimination
 * Block '<S25>/enableInportSatMethod' : Unused code path elimination
 * Block '<S20>/ReplaceInport_satLim' : Unused code path elimination
 * Block '<S20>/ReplaceInport_satMethod' : Unused code path elimination
 * Block '<S6>/Constant' : Unused code path elimination
 * Block '<S6>/Vdc_lumped' : Unused code path elimination
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
 * '<Root>' : 'mtpa'
 * '<S1>'   : 'mtpa/MTPA Control Reference'
 * '<S2>'   : 'mtpa/MTPA Control Reference/Motor_System'
 * '<S3>'   : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM'
 * '<S4>'   : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection'
 * '<S5>'   : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/Subsystem'
 * '<S6>'   : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/Subsystem2'
 * '<S7>'   : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators'
 * '<S8>'   : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate'
 * '<S9>'   : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/Circle_MTPA_intersection'
 * '<S10>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition'
 * '<S11>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/MTPA condition'
 * '<S12>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/Subsystem1'
 * '<S13>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/Circle_MTPA_intersection/Get_Iq_ref'
 * '<S14>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/If Action Subsystem'
 * '<S15>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/If Action Subsystem1'
 * '<S16>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed'
 * '<S17>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/If Action Subsystem1/fieldWeakening'
 * '<S18>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/If Action Subsystem1/fieldWeakening/torqueTangent'
 * '<S19>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/If Action Subsystem1/fieldWeakening/update iq1'
 * '<S20>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter'
 * '<S21>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/sqrt_controlled'
 * '<S22>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/update iq2'
 * '<S23>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority'
 * '<S24>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D-Q Equivalence'
 * '<S25>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/Inport or Dialog Selection'
 * '<S26>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/Magnitude_calc'
 * '<S27>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/Compare To Constant'
 * '<S28>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/Compare To Constant1'
 * '<S29>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/flipInputs'
 * '<S30>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/flipInputs1'
 * '<S31>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/limiter'
 * '<S32>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/limiter/limitRef1'
 * '<S33>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/limiter/limitRef2'
 * '<S34>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D or Q Axis Priority/limiter/passThrough'
 * '<S35>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D-Q Equivalence/Limiter'
 * '<S36>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/DQ Limiter/D-Q Equivalence/Passthrough'
 * '<S37>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/sqrt_controlled/Compare To Zero'
 * '<S38>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/sqrt_controlled/If negative value'
 * '<S39>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/FW condition/max_iq_at_speed/sqrt_controlled/If positive value'
 * '<S40>'  : 'mtpa/MTPA Control Reference/Motor_System/Interior PMSM/MTPA_FW_iteratorSelection/no_iterators/fast_and_approximate/Subsystem1/find vs'
 */
#endif                                 /* mtpa_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
