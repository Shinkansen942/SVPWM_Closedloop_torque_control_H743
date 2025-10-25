/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: PLL.c
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

#include "PLL.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "zero_crossing_types.h"

/* Block states (default storage) */
DW_PLL_T PLL_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_PLL_T PLL_PrevZCX;

/* Real-time model */
static RT_MODEL_PLL_T PLL_M_;
RT_MODEL_PLL_T *const PLL_M = &PLL_M_;

/* Model step function */
void PLL_step(real_T sine_in, real_T cosine_in, boolean_T rst_in, real32_T
              *rad_out, real32_T *deg_sec_out)
{
  real_T rtb_Delay2_b;
  real_T rtb_Merge_idx_0;
  real_T rtb_Merge_idx_1;
  real_T rtb_Sum;
  real32_T rtb_Product_k_tmp;
  real32_T rtb_Sum2_a;
  real32_T rtb_indexing;
  uint16_T rtb_Get_Integer;

  /* SwitchCase: '<S6>/Switch Case' incorporates:
   *  Inport: '<Root>/In3'
   */
  if (rst_in) {
    /* Outputs for IfAction SubSystem: '<S6>/If Action Subsystem' incorporates:
     *  ActionPort: '<S100>/Action Port'
     */
    /* SignalConversion generated from: '<S100>/Out1' incorporates:
     *  Constant: '<S6>/Constant'
     *  SignalConversion generated from: '<S100>/In1'
     */
    rtb_Merge_idx_0 = 0.0;
    rtb_Merge_idx_1 = 0.0;

    /* End of Outputs for SubSystem: '<S6>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S6>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S101>/Action Port'
     */
    /* SignalConversion generated from: '<S101>/In1' incorporates:
     *  Inport: '<Root>/In1'
     *  Inport: '<Root>/In2'
     */
    rtb_Merge_idx_0 = sine_in;
    rtb_Merge_idx_1 = cosine_in;

    /* End of Outputs for SubSystem: '<S6>/If Action Subsystem1' */
  }

  /* End of SwitchCase: '<S6>/Switch Case' */

  /* Sum: '<S3>/Sum' incorporates:
   *  Product: '<S3>/Product'
   *  Product: '<S3>/Product1'
   */
  rtb_Sum = rtb_Merge_idx_0 * rtb_Merge_idx_0 + rtb_Merge_idx_1 *
    rtb_Merge_idx_1;

  /* If: '<S3>/If' incorporates:
   *  Constant: '<S11>/Constant'
   *  RelationalOperator: '<S11>/Compare'
   */
  if (rtb_Sum > 0.0) {
    /* Outputs for IfAction SubSystem: '<S3>/Subsystem' incorporates:
     *  ActionPort: '<S13>/Action Port'
     */
    /* Sqrt: '<S13>/Sqrt' */
    rtb_Sum = sqrt(rtb_Sum);

    /* SignalConversion generated from: '<S13>/Out2' incorporates:
     *  Product: '<S13>/Divide'
     */
    rtb_Merge_idx_1 *= 1.0 / rtb_Sum;

    /* SignalConversion generated from: '<S13>/Out1' incorporates:
     *  Product: '<S13>/Divide1'
     */
    rtb_Merge_idx_0 /= rtb_Sum;

    /* End of Outputs for SubSystem: '<S3>/Subsystem' */
  }

  /* End of If: '<S3>/If' */

  /* Outputs for IfAction SubSystem: '<S5>/OptimizedDifferentiationMethod' incorporates:
   *  ActionPort: '<S72>/Action Port'
   */
  /* SwitchCase: '<S5>/Switch Case' incorporates:
   *  Constant: '<S66>/OD1Cnst'
   *  Constant: '<S66>/OD2Cnst'
   *  Constant: '<S92>/IC'
   *  Constant: '<S93>/IC'
   *  Delay: '<S90>/Delay2'
   *  Delay: '<S90>/Delay3'
   *  Delay: '<S91>/Delay2'
   *  Delay: '<S91>/Delay3'
   *  Delay: '<S96>/Delay'
   *  Delay: '<S99>/Delay'
   *  Inport: '<Root>/In3'
   *  Product: '<S90>/Product'
   *  Product: '<S90>/Product1'
   *  Product: '<S91>/Product'
   *  Product: '<S91>/Product1'
   *  Product: '<S96>/Product'
   *  Product: '<S96>/Product1'
   *  Product: '<S99>/Product'
   *  Product: '<S99>/Product1'
   *  Sum: '<S90>/Sum1'
   *  Sum: '<S91>/Sum1'
   *  Sum: '<S96>/Add1'
   *  Sum: '<S99>/Add1'
   *  Switch: '<S96>/Switch'
   *  Switch: '<S99>/Switch'
   * */
  if (PLL_DW.icLoad_dx) {
    PLL_DW.Delay_DSTATE_f1 = 0.0;
  }

  PLL_DW.Delay_DSTATE_f1 = rtb_Merge_idx_1 * 0.73203421367634736 +
    0.26796578632365264 * PLL_DW.Delay_DSTATE_f1;
  rtb_Sum = PLL_DW.Delay_DSTATE_f1 * 314159.26535897935;
  if ((((PLL_PrevZCX.Delay2_Reset_ZCE == POS_ZCSIG) != (int32_T)rst_in) &&
       (PLL_PrevZCX.Delay2_Reset_ZCE != UNINITIALIZED_ZCSIG)) || rst_in) {
    PLL_DW.Delay2_DSTATE = 0.0;
  }

  PLL_PrevZCX.Delay2_Reset_ZCE = rst_in;
  if ((((PLL_PrevZCX.Delay3_Reset_ZCE == POS_ZCSIG) != (int32_T)rst_in) &&
       (PLL_PrevZCX.Delay3_Reset_ZCE != UNINITIALIZED_ZCSIG)) || rst_in) {
    PLL_DW.Delay3_DSTATE = 0.0;
  }

  PLL_PrevZCX.Delay3_Reset_ZCE = rst_in;
  PLL_DW.Delay3_DSTATE = ((PLL_DW.Delay3_DSTATE + rtb_Sum) -
    PLL_DW.Delay2_DSTATE) * 0.068217018967316523;
  if ((((PLL_PrevZCX.Delay2_Reset_ZCE_p == POS_ZCSIG) != (int32_T)rst_in) &&
       (PLL_PrevZCX.Delay2_Reset_ZCE_p != UNINITIALIZED_ZCSIG)) || rst_in) {
    PLL_DW.Delay2_DSTATE_p = 0.0;
  }

  PLL_PrevZCX.Delay2_Reset_ZCE_p = rst_in;
  rtb_Delay2_b = PLL_DW.Delay2_DSTATE_p;
  if ((((PLL_PrevZCX.Delay3_Reset_ZCE_b == POS_ZCSIG) != (int32_T)rst_in) &&
       (PLL_PrevZCX.Delay3_Reset_ZCE_b != UNINITIALIZED_ZCSIG)) || rst_in) {
    PLL_DW.Delay3_DSTATE_b = 0.0;
  }

  PLL_PrevZCX.Delay3_Reset_ZCE_b = rst_in;
  if (PLL_DW.icLoad_c) {
    PLL_DW.Delay_DSTATE_i = 0.0;
  }

  PLL_DW.Delay_DSTATE_i = rtb_Merge_idx_0 * 0.73203421367634736 +
    0.26796578632365264 * PLL_DW.Delay_DSTATE_i;
  PLL_DW.Delay2_DSTATE_p = PLL_DW.Delay_DSTATE_i * 314159.26535897935;
  PLL_DW.Delay3_DSTATE_b = ((PLL_DW.Delay3_DSTATE_b + PLL_DW.Delay2_DSTATE_p) -
    rtb_Delay2_b) * 0.068217018967316523;
  PLL_DW.icLoad_dx = false;
  PLL_DW.Delay2_DSTATE = rtb_Sum;
  PLL_DW.icLoad_c = false;

  /* End of Outputs for SubSystem: '<S5>/OptimizedDifferentiationMethod' */

  /* Delay: '<S79>/Delay' */
  if (PLL_DW.icLoad) {
    /* Sum: '<S79>/Add1' incorporates:
     *  Constant: '<S67>/IC'
     */
    PLL_DW.Delay_DSTATE = 0.0;
  }

  /* Outputs for IfAction SubSystem: '<S5>/OptimizedDifferentiationMethod' incorporates:
   *  ActionPort: '<S72>/Action Port'
   */
  /* SwitchCase: '<S5>/Switch Case' incorporates:
   *  Delay: '<S79>/Delay'
   *  Delay: '<S90>/Delay3'
   *  Delay: '<S91>/Delay3'
   *  Product: '<S72>/Product'
   *  Product: '<S72>/Product1'
   *  Product: '<S79>/Product'
   *  Product: '<S79>/Product1'
   *  Sum: '<S72>/Sum'
   *  Sum: '<S79>/Add1'
   */
  PLL_DW.Delay_DSTATE = (PLL_DW.Delay3_DSTATE_b * rtb_Merge_idx_1 -
    PLL_DW.Delay3_DSTATE * rtb_Merge_idx_0) * 0.00408100668647519 +
    0.99591899331352485 * PLL_DW.Delay_DSTATE;

  /* End of Outputs for SubSystem: '<S5>/OptimizedDifferentiationMethod' */

  /* Delay: '<S103>/Delay' incorporates:
   *  Inport: '<Root>/In3'
   */
  if ((((PLL_PrevZCX.Delay_Reset_ZCE == POS_ZCSIG) != (int32_T)rst_in) &&
       (PLL_PrevZCX.Delay_Reset_ZCE != UNINITIALIZED_ZCSIG)) || rst_in) {
    /* Sum: '<S104>/Sum' */
    PLL_DW.Delay_DSTATE_h = 0.0;
  }

  /* Outputs for IfAction SubSystem: '<S5>/OptimizedDifferentiationMethod' incorporates:
   *  ActionPort: '<S72>/Action Port'
   */
  /* SwitchCase: '<S5>/Switch Case' incorporates:
   *  Delay: '<S103>/Delay'
   *  Delay: '<S91>/Delay2'
   *  Inport: '<Root>/In3'
   */
  PLL_PrevZCX.Delay_Reset_ZCE = rst_in;

  /* End of Outputs for SubSystem: '<S5>/OptimizedDifferentiationMethod' */

  /* If: '<S108>/If' incorporates:
   *  Constant: '<S110>/Constant'
   *  DataTypeConversion: '<S103>/Data Type Conversion3'
   *  Delay: '<S103>/Delay'
   *  RelationalOperator: '<S110>/Compare'
   */
  if ((real32_T)PLL_DW.Delay_DSTATE_h < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S108>/If Action Subsystem' incorporates:
     *  ActionPort: '<S111>/Action Port'
     */
    /* DataTypeConversion: '<S111>/Convert_uint16' */
    rtb_Sum2_a = floorf((real32_T)PLL_DW.Delay_DSTATE_h);
    if (rtIsInfF(rtb_Sum2_a)) {
      rtb_Sum2_a = 0.0F;
    } else {
      rtb_Sum2_a = fmodf(rtb_Sum2_a, 65536.0F);
    }

    /* Sum: '<S111>/Sum' incorporates:
     *  DataTypeConversion: '<S111>/Convert_back'
     *  DataTypeConversion: '<S111>/Convert_uint16'
     */
    rtb_indexing = (real32_T)PLL_DW.Delay_DSTATE_h - (real32_T)(rtb_Sum2_a <
      0.0F ? (int32_T)(int16_T)-(int16_T)(uint16_T)-rtb_Sum2_a : (int32_T)
      (int16_T)(uint16_T)rtb_Sum2_a);

    /* End of Outputs for SubSystem: '<S108>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S108>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S112>/Action Port'
     */
    /* DataTypeConversion: '<S112>/Convert_uint16' */
    rtb_Sum2_a = truncf((real32_T)PLL_DW.Delay_DSTATE_h);
    if (rtIsNaNF(rtb_Sum2_a) || rtIsInfF(rtb_Sum2_a)) {
      rtb_Sum2_a = 0.0F;
    } else {
      rtb_Sum2_a = fmodf(rtb_Sum2_a, 65536.0F);
    }

    /* Sum: '<S112>/Sum' incorporates:
     *  DataTypeConversion: '<S112>/Convert_back'
     *  DataTypeConversion: '<S112>/Convert_uint16'
     */
    rtb_indexing = (real32_T)PLL_DW.Delay_DSTATE_h - (real32_T)(int16_T)
      (uint16_T)rtb_Sum2_a;

    /* End of Outputs for SubSystem: '<S108>/If Action Subsystem1' */
  }

  /* End of If: '<S108>/If' */

  /* Gain: '<S106>/indexing' */
  rtb_indexing *= 800.0F;

  /* DataTypeConversion: '<S106>/Get_Integer' */
  rtb_Sum2_a = truncf(rtb_indexing);
  if (rtIsNaNF(rtb_Sum2_a) || rtIsInfF(rtb_Sum2_a)) {
    rtb_Sum2_a = 0.0F;
  } else {
    rtb_Sum2_a = fmodf(rtb_Sum2_a, 65536.0F);
  }

  rtb_Get_Integer = (uint16_T)(rtb_Sum2_a < 0.0F ? (int32_T)(uint16_T)-(int16_T)
    (uint16_T)-rtb_Sum2_a : (int32_T)(uint16_T)rtb_Sum2_a);

  /* End of DataTypeConversion: '<S106>/Get_Integer' */

  /* Sum: '<S106>/Sum2' incorporates:
   *  DataTypeConversion: '<S106>/Data Type Conversion1'
   */
  rtb_Sum2_a = rtb_indexing - (real32_T)rtb_Get_Integer;

  /* Selector: '<S106>/Lookup' incorporates:
   *  Constant: '<S106>/offset'
   *  Constant: '<S106>/sine_table_values'
   *  Sum: '<S106>/Sum'
   *  Sum: '<S107>/Sum3'
   *  Sum: '<S107>/Sum5'
   */
  rtb_indexing = PLL_ConstP.sine_table_values_Value[(int32_T)(rtb_Get_Integer +
    200U)];
  rtb_Product_k_tmp = PLL_ConstP.sine_table_values_Value[rtb_Get_Integer];

  /* Sum: '<S1>/Sum' incorporates:
   *  Constant: '<S106>/offset'
   *  Constant: '<S106>/sine_table_values'
   *  DataTypeConversion: '<S103>/Data Type Conversion'
   *  DataTypeConversion: '<S103>/Data Type Conversion1'
   *  Product: '<S107>/Product'
   *  Product: '<S107>/Product1'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product1'
   *  Selector: '<S106>/Lookup'
   *  Sum: '<S106>/Sum'
   *  Sum: '<S107>/Sum3'
   *  Sum: '<S107>/Sum4'
   *  Sum: '<S107>/Sum5'
   *  Sum: '<S107>/Sum6'
   */
  rtb_Merge_idx_0 = ((PLL_ConstP.sine_table_values_Value[(int32_T)
                      (rtb_Get_Integer + 201U)] - rtb_indexing) * rtb_Sum2_a +
                     rtb_indexing) * rtb_Merge_idx_0 -
    ((PLL_ConstP.sine_table_values_Value[(int32_T)(rtb_Get_Integer + 1U)] -
      rtb_Product_k_tmp) * rtb_Sum2_a + rtb_Product_k_tmp) * rtb_Merge_idx_1;

  /* DiscreteIntegrator: '<S48>/Integrator' incorporates:
   *  Inport: '<Root>/In3'
   */
  if (rst_in || (PLL_DW.Integrator_PrevResetState != 0)) {
    PLL_DW.Integrator_DSTATE = 0.0;
  }

  /* DiscreteIntegrator: '<S48>/Integrator' incorporates:
   *  Gain: '<S45>/Integral Gain'
   */
  PLL_DW.Integrator_DSTATE += 38.620191134782608 * rtb_Merge_idx_0;

  /* Sum: '<S1>/Sum1' incorporates:
   *  Gain: '<S53>/Proportional Gain'
   *  Sum: '<S57>/Sum'
   */
  rtb_Merge_idx_0 = (1884.9556 * rtb_Merge_idx_0 + PLL_DW.Integrator_DSTATE) +
    PLL_DW.Delay_DSTATE;

  /* Product: '<S104>/Product' incorporates:
   *  Gain: '<S103>/Gain'
   */
  rtb_Merge_idx_1 = 6.919780134430233E-6 * rtb_Merge_idx_0;

  /* Delay: '<S104>/Delay' incorporates:
   *  Constant: '<S104>/InputGain'
   *  Constant: '<S104>/Offset'
   *  Product: '<S104>/Product1'
   *  Sum: '<S104>/Sum1'
   */
  if (PLL_DW.icLoad_d) {
    PLL_DW.Delay_DSTATE_p = 0.0 - rtb_Merge_idx_1;
  }

  /* Sum: '<S104>/Sum' incorporates:
   *  Delay: '<S104>/Delay'
   */
  PLL_DW.Delay_DSTATE_h = rtb_Merge_idx_1 + PLL_DW.Delay_DSTATE_p;

  /* Switch: '<S104>/Switch1' */
  if (!(PLL_DW.Delay_DSTATE_h > 0.0)) {
    /* Sum: '<S104>/Sum' incorporates:
     *  Constant: '<S104>/Bias1'
     *  Sum: '<S104>/Sum2'
     */
    PLL_DW.Delay_DSTATE_h++;
  }

  /* End of Switch: '<S104>/Switch1' */

  /* Switch: '<S104>/Switch2' */
  if (PLL_DW.Delay_DSTATE_h >= 1.0) {
    /* Sum: '<S104>/Sum' incorporates:
     *  Sum: '<S104>/Sum3'
     */
    PLL_DW.Delay_DSTATE_h--;
  }

  /* End of Switch: '<S104>/Switch2' */

  /* Outport: '<Root>/Out1' incorporates:
   *  Delay: '<S103>/Delay'
   *  Gain: '<S1>/PositionGain'
   */
  *rad_out = (real32_T)(6.2831853071795862 * PLL_DW.Delay_DSTATE_h);

  /* Product: '<S10>/Product' */
  rtb_Merge_idx_0 *= 0.00408100668647519;

  /* Delay: '<S10>/Delay' */
  if (PLL_DW.icLoad_l) {
    /* Sum: '<S10>/Add1' incorporates:
     *  Constant: '<S2>/IC'
     */
    PLL_DW.Delay_DSTATE_f = 0.0;
  }

  /* Sum: '<S10>/Add1' incorporates:
   *  Delay: '<S10>/Delay'
   *  Product: '<S10>/Product1'
   */
  PLL_DW.Delay_DSTATE_f = 0.99591899331352485 * PLL_DW.Delay_DSTATE_f +
    rtb_Merge_idx_0;

  /* Outport: '<Root>/Out3' incorporates:
   *  Gain: '<S1>/FreqGain'
   */
  *deg_sec_out = (real32_T)(57.295779513082323 * PLL_DW.Delay_DSTATE_f);

  /* Update for Delay: '<S79>/Delay' */
  PLL_DW.icLoad = false;

  /* Update for DiscreteIntegrator: '<S48>/Integrator' incorporates:
   *  Inport: '<Root>/In3'
   */
  PLL_DW.Integrator_PrevResetState = (int8_T)rst_in;

  /* Update for Delay: '<S104>/Delay' */
  PLL_DW.icLoad_d = false;
  PLL_DW.Delay_DSTATE_p = PLL_DW.Delay_DSTATE_h;

  /* Update for Delay: '<S10>/Delay' */
  PLL_DW.icLoad_l = false;
}

/* Model initialize function */
void PLL_initialize(void)
{
  PLL_PrevZCX.Delay_Reset_ZCE = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay1_Reset_ZCE = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay_Reset_ZCE_i = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay1_Reset_ZCE_e = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay_Reset_ZCE_p = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay2_Reset_ZCE = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay3_Reset_ZCE = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay2_Reset_ZCE_p = UNINITIALIZED_ZCSIG;
  PLL_PrevZCX.Delay3_Reset_ZCE_b = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for Delay: '<S79>/Delay' */
  PLL_DW.icLoad = true;

  /* InitializeConditions for Delay: '<S104>/Delay' */
  PLL_DW.icLoad_d = true;

  /* InitializeConditions for Delay: '<S10>/Delay' */
  PLL_DW.icLoad_l = true;

  /* SystemInitialize for IfAction SubSystem: '<S5>/IntegralMethod' */
  /* InitializeConditions for Delay: '<S86>/Delay' */
  PLL_DW.icLoad_a = true;

  /* InitializeConditions for Delay: '<S89>/Delay' */
  PLL_DW.icLoad_k = true;

  /* End of SystemInitialize for SubSystem: '<S5>/IntegralMethod' */

  /* SystemInitialize for IfAction SubSystem: '<S5>/OptimizedDifferentiationMethod' */
  /* InitializeConditions for Delay: '<S96>/Delay' */
  PLL_DW.icLoad_dx = true;

  /* InitializeConditions for Delay: '<S99>/Delay' */
  PLL_DW.icLoad_c = true;

  /* End of SystemInitialize for SubSystem: '<S5>/OptimizedDifferentiationMethod' */
}

/* Model terminate function */
void PLL_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
