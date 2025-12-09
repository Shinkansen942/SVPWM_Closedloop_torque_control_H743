/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: mtpa.c
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

#include "mtpa.h"
#include <math.h>

/*===========*
 * Constants *
 *===========*/
#define RT_PI                          3.14159265358979323846
#define RT_PIF                         3.1415927F
#define RT_LN_10                       2.30258509299404568402
#define RT_LN_10F                      2.3025851F
#define RT_LOG10E                      0.43429448190325182765
#define RT_LOG10EF                     0.43429449F
#define RT_E                           2.7182818284590452354
#define RT_EF                          2.7182817F

/*
 * UNUSED_PARAMETER(x)
 *   Used to specify that a function parameter (argument) is required but not
 *   accessed by the function body.
 */
#ifndef UNUSED_PARAMETER
#if defined(__LCC__)
#define UNUSED_PARAMETER(x)                                      /* do nothing */
#else

/*
 * This is the semi-ANSI standard way of indicating that an
 * unused function parameter is required.
 */
#define UNUSED_PARAMETER(x)            (void) (x)
#endif
#endif

/* Model step function */
void mtpa_step(RT_MODEL *const rtM, double Torque_ref, double speed, double V_DC,
               double *Id_ref, double *Iq_ref)
{
  double rtb_Abs;
  double rtb_Abs_h;
  double rtb_Abs_o_tmp;
  double rtb_Gain;
  double rtb_Imag;
  double rtb_Product2;
  double rtb_Saturation2;
  double rtb_Sqrt_i;
  double rtb_we;

  /* Gain: '<S8>/Gain' incorporates:
   *  Abs: '<S3>/Abs'
   *  Inport: '<Root>/Torque_ref'
   */
  rtb_Gain = 4.0257648953301128 * fabs(Torque_ref);

  /* Saturate: '<S8>/Saturation' */
  if (rtb_Gain > 80.0) {
    rtb_Imag = 80.0;
  } else {
    rtb_Imag = rtb_Gain;
  }

  /* End of Saturate: '<S8>/Saturation' */

  /* Outputs for IfAction SubSystem: '<S8>/FW condition' incorporates:
   *  ActionPort: '<S10>/Action Port'
   */
  /* Outputs for IfAction SubSystem: '<S20>/D or Q Axis Priority' incorporates:
   *  ActionPort: '<S23>/Action Port'
   */
  /* If: '<S20>/If' incorporates:
   *  If: '<S8>/If'
   *  Product: '<S31>/Product2'
   *  Product: '<S9>/Square'
   */
  rtb_Abs_o_tmp = rtb_Imag * rtb_Imag;

  /* End of Outputs for SubSystem: '<S20>/D or Q Axis Priority' */
  /* End of Outputs for SubSystem: '<S8>/FW condition' */

  /* Sum: '<S9>/Sum2' incorporates:
   *  Abs: '<S9>/Abs'
   *  Constant: '<S9>/term1'
   *  Constant: '<S9>/term2'
   *  Gain: '<S9>/term3'
   *  Product: '<S9>/Square'
   *  Product: '<S9>/Square1'
   *  Sqrt: '<S9>/Sqrt'
   *  Sum: '<S9>/Sum3'
   */
  rtb_Saturation2 = 79.007633587786231 - sqrt(fabs(-0.5 * rtb_Abs_o_tmp -
    6242.2061651418871));

  /* Abs: '<S1>/Abs' incorporates:
   *  Inport: '<Root>/speed'
   */
  rtb_Abs = fabs(speed);

  /* Gain: '<S8>/Gain1' */
  rtb_we = 4.0 * rtb_Abs;

  /* Sum: '<S40>/Sum1' incorporates:
   *  Gain: '<S40>/FluxPM'
   *  Gain: '<S40>/Ld'
   *  Product: '<S40>/Product1'
   *  Saturate: '<S13>/Saturation2'
   */
  rtb_Sqrt_i = rtb_Saturation2 * rtb_we * 0.000249 + 0.0414 * rtb_we;

  /* Product: '<S40>/Product2' */
  rtb_Product2 = rtb_Sqrt_i * rtb_Sqrt_i;

  /* Sqrt: '<S13>/Sqrt' incorporates:
   *  Abs: '<S13>/Abs'
   *  Product: '<S13>/Product1'
   *  Product: '<S9>/Square'
   *  Saturate: '<S13>/Saturation2'
   *  Sum: '<S13>/Sum2'
   */
  rtb_Abs_h = sqrt(fabs(rtb_Abs_o_tmp - rtb_Saturation2 * rtb_Saturation2));

  /* Gain: '<S40>/Lq' incorporates:
   *  Product: '<S40>/Product'
   */
  rtb_Sqrt_i = rtb_we * rtb_Abs_h * 0.00038;

  /* Sum: '<S40>/Sum3' incorporates:
   *  Product: '<S40>/Product3'
   */
  rtb_Sqrt_i = rtb_Sqrt_i * rtb_Sqrt_i + rtb_Product2;

  /* Outputs for IfAction SubSystem: '<S8>/FW condition' incorporates:
   *  ActionPort: '<S10>/Action Port'
   */
  /* If: '<S8>/If' incorporates:
   *  Constant: '<S12>/Constant'
   *  Gain: '<S12>/Gain'
   *  Inport: '<Root>/V_DC'
   *  RelationalOperator: '<S12>/GreaterThan'
   *  Sqrt: '<S40>/Sqrt'
   *  Sum: '<S12>/Subtract'
   *  Sum: '<S16>/Subtract1'
   */
  rtb_Product2 = 0.57735026918962584 * V_DC - 10.08;

  /* End of Outputs for SubSystem: '<S8>/FW condition' */
  if (!(sqrt(rtb_Sqrt_i) >= rtb_Product2)) {
    /* Outputs for IfAction SubSystem: '<S8>/MTPA condition' incorporates:
     *  ActionPort: '<S11>/Action Port'
     */
    /* SignalConversion generated from: '<S11>/id' incorporates:
     *  Merge: '<S8>/Merge'
     *  Saturate: '<S13>/Saturation2'
     */
    rtM->dwork.Merge[0] = rtb_Saturation2;

    /* SignalConversion generated from: '<S11>/iq' incorporates:
     *  Merge: '<S8>/Merge'
     */
    rtM->dwork.Merge[1] = rtb_Abs_h;

    /* End of Outputs for SubSystem: '<S8>/MTPA condition' */
  } else {
    /* Outputs for IfAction SubSystem: '<S8>/FW condition' incorporates:
     *  ActionPort: '<S10>/Action Port'
     */
    /* Gain: '<S16>/  ' incorporates:
     *  Math: '<S16>/Math Function'
     *  Product: '<S16>/Square1'
     *
     * About '<S16>/Math Function':
     *  Operator: reciprocal
     */
    rtb_Saturation2 = 1.0 / rtb_Abs * rtb_Product2 * 870.9215525987147;

    /* Sum: '<S16>/Sum3' incorporates:
     *  Constant: '<S16>/term2'
     *  Constant: '<S16>/term3'
     *  Gain: '<S16>/ '
     *  Product: '<S16>/Square'
     *  Product: '<S16>/Square2'
     *  Sum: '<S16>/Sum1'
     */
    rtb_Saturation2 = 15651.483013942925 - (-32016.407966116083 -
      (-(rtb_Saturation2 * rtb_Saturation2)));

    /* If: '<S21>/If' incorporates:
     *  Constant: '<S37>/Constant'
     *  Gain: '<S38>/zero'
     *  RelationalOperator: '<S37>/Compare'
     *  Sqrt: '<S39>/Sqrt'
     */
    if (rtb_Saturation2 < 0.0) {
      /* Outputs for IfAction SubSystem: '<S21>/If negative value' incorporates:
       *  ActionPort: '<S38>/Action Port'
       */
      rtb_Product2 = 0.0 * rtb_Saturation2;

      /* End of Outputs for SubSystem: '<S21>/If negative value' */
    } else {
      /* Outputs for IfAction SubSystem: '<S21>/If positive value' incorporates:
       *  ActionPort: '<S39>/Action Port'
       */
      rtb_Product2 = sqrt(rtb_Saturation2);

      /* End of Outputs for SubSystem: '<S21>/If positive value' */
    }

    /* Outputs for IfAction SubSystem: '<S20>/D or Q Axis Priority' incorporates:
     *  ActionPort: '<S23>/Action Port'
     */
    /* If: '<S20>/If' incorporates:
     *  Constant: '<S16>/term1'
     *  Constant: '<S25>/Constant3'
     *  If: '<S21>/If'
     *  Product: '<S31>/Product'
     *  RelationalOperator: '<S32>/LowerRelop1'
     *  Sum: '<S16>/Sum2'
     *  Sum: '<S31>/Sum'
     *  Switch: '<S23>/Switch'
     *  Switch: '<S25>/Switch'
     *  Switch: '<S32>/Switch2'
     */
    rtb_Abs = rtb_Imag;
    if (125.10588720736897 - rtb_Product2 > 80.0) {
      rtb_Imag = 80.0;
    } else {
      /* Gain: '<S31>/Gain' */
      rtb_Imag = -80.0;

      /* Switch: '<S32>/Switch' incorporates:
       *  RelationalOperator: '<S32>/UpperRelop'
       */
      if (!(125.10588720736897 - rtb_Product2 < -80.0)) {
        rtb_Imag = 125.10588720736897 - rtb_Product2;
      }

      /* End of Switch: '<S32>/Switch' */
    }

    rtb_Saturation2 = 6400.0 - rtb_Imag * rtb_Imag;

    /* If: '<S31>/If' incorporates:
     *  If: '<S20>/If'
     *  RelationalOperator: '<S31>/Relational Operator'
     *  Switch: '<S33>/Switch'
     *  Switch: '<S33>/Switch1'
     */
    if (rtb_Saturation2 >= rtb_Abs_o_tmp) {
      /* Outputs for IfAction SubSystem: '<S31>/passThrough' incorporates:
       *  ActionPort: '<S34>/Action Port'
       */
      /* SignalConversion generated from: '<S34>/ref2' */
      rtb_Saturation2 = rtb_Abs;

      /* End of Outputs for SubSystem: '<S31>/passThrough' */

      /* Outputs for IfAction SubSystem: '<S31>/limitRef2' incorporates:
       *  ActionPort: '<S33>/Action Port'
       */
    } else if (rtb_Abs >= 0.0) {
      /* Switch: '<S33>/Switch1' incorporates:
       *  Constant: '<S33>/Constant'
       */
      if (!(rtb_Saturation2 > 0.0)) {
        rtb_Saturation2 = 0.0;
      }

      /* Switch: '<S33>/Switch' incorporates:
       *  Sqrt: '<S33>/Sqrt'
       *  Switch: '<S33>/Switch1'
       */
      rtb_Saturation2 = sqrt(rtb_Saturation2);
    } else {
      if (!(rtb_Saturation2 > 0.0)) {
        /* Switch: '<S33>/Switch1' incorporates:
         *  Constant: '<S33>/Constant'
         */
        rtb_Saturation2 = 0.0;
      }

      /* Switch: '<S33>/Switch' incorporates:
       *  Gain: '<S33>/Gain'
       *  Sqrt: '<S33>/Sqrt'
       *  Switch: '<S33>/Switch1'
       */
      rtb_Saturation2 = -sqrt(rtb_Saturation2);

      /* End of Outputs for SubSystem: '<S31>/limitRef2' */
    }

    /* End of If: '<S31>/If' */

    /* Outputs for IfAction SubSystem: '<S10>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S15>/Action Port'
     */
    /* If: '<S10>/If' incorporates:
     *  Constant: '<S22>/Constant'
     *  Gain: '<S22>/Gain'
     *  If: '<S20>/If'
     *  Sum: '<S19>/Add1'
     *  Sum: '<S22>/Add'
     *  Switch: '<S23>/Switch1'
     */
    rtb_Abs_o_tmp = -0.0031642512077294698 * rtb_Imag + 1.0;

    /* End of Outputs for SubSystem: '<S10>/If Action Subsystem1' */
    /* End of Outputs for SubSystem: '<S20>/D or Q Axis Priority' */

    /* Product: '<S22>/Divide' incorporates:
     *  Sum: '<S22>/Add'
     */
    rtb_Gain = 1.0 / rtb_Abs_o_tmp * rtb_Gain * rtConstB.Add1;

    /* Outputs for IfAction SubSystem: '<S20>/D or Q Axis Priority' incorporates:
     *  ActionPort: '<S23>/Action Port'
     */
    /* If: '<S10>/If' incorporates:
     *  If: '<S20>/If'
     *  RelationalOperator: '<S10>/GreaterThan'
     *  Switch: '<S23>/Switch1'
     */
    if (rtb_Gain > rtb_Saturation2) {
      /* Outputs for IfAction SubSystem: '<S10>/If Action Subsystem' incorporates:
       *  ActionPort: '<S14>/Action Port'
       */
      /* Merge: '<S8>/Merge' incorporates:
       *  SignalConversion generated from: '<S14>/In1'
       */
      rtM->dwork.Merge[0] = rtb_Imag;
      rtM->dwork.Merge[1] = rtb_Saturation2;

      /* End of Outputs for SubSystem: '<S10>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S10>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S15>/Action Port'
       */
      /* Product: '<S17>/Divide' incorporates:
       *  Constant: '<S17>/Constant'
       *  Gain: '<S10>/Gain'
       *  Gain: '<S17>/Gain'
       *  Sum: '<S17>/Subtract'
       */
      rtb_Abs = (1519.34281365691 * V_DC - 26526.315789473683) / rtb_we;

      /* Product: '<S17>/Product' */
      rtb_Saturation2 = rtb_Abs * rtb_Abs;

      /* UnaryMinus: '<S18>/Unary Minus' incorporates:
       *  Product: '<S18>/Divide'
       *  Sum: '<S18>/Sum1'
       */
      rtb_Abs = -(rtb_Gain / (rtb_Imag - 316.03053435114492));

      /* Sum: '<S18>/constant output' incorporates:
       *  Product: '<S18>/Product'
       */
      rtb_Imag = rtb_Gain - rtb_Abs * rtb_Imag;

      /* Sum: '<S17>/C' incorporates:
       *  Constant: '<S17>/fluxPM2_Lq2'
       *  Product: '<S17>/Product1'
       *  Product: '<S17>/c2'
       */
      rtb_Saturation2 = (rtb_Imag * rtb_Imag + 11869.529085872575) -
        rtb_Saturation2;

      /* Sum: '<S17>/A' incorporates:
       *  Constant: '<S17>/ld2_lq2'
       *  Product: '<S17>/m2'
       */
      rtb_we = rtb_Abs * rtb_Abs + 0.42936980609418274;

      /* Sum: '<S17>/B' incorporates:
       *  Constant: '<S17>/fluxPMLd_Lq2'
       *  Product: '<S17>/mc'
       */
      rtb_Imag = rtb_Abs * rtb_Imag + 71.389196675900266;

      /* Product: '<S17>/Divide1' incorporates:
       *  Abs: '<S17>/Abs'
       *  Product: '<S17>/AC'
       *  Product: '<S17>/B2'
       *  Sqrt: '<S17>/Sqrt'
       *  Sum: '<S17>/Subtract4'
       *  Sum: '<S17>/Subtract5'
       */
      rtb_we = (sqrt(fabs(rtb_Imag * rtb_Imag - rtb_we * rtb_Saturation2)) -
                rtb_Imag) / rtb_we;

      /* Merge: '<S8>/Merge' incorporates:
       *  Constant: '<S19>/Constant'
       *  Gain: '<S19>/Gain'
       *  Product: '<S19>/Divide'
       *  SignalConversion generated from: '<S15>/Out1'
       *  Sum: '<S19>/Add'
       */
      rtM->dwork.Merge[0] = rtb_we;
      rtM->dwork.Merge[1] = 1.0 / (-0.0031642512077294698 * rtb_we + 1.0) *
        rtb_Gain * rtb_Abs_o_tmp;

      /* End of Outputs for SubSystem: '<S10>/If Action Subsystem1' */
    }

    /* End of Outputs for SubSystem: '<S20>/D or Q Axis Priority' */
    /* End of Outputs for SubSystem: '<S8>/FW condition' */
  }

  /* Switch: '<S5>/sign' incorporates:
   *  Inport: '<Root>/Torque_ref'
   */
  if (Torque_ref > 0.0) {
    /* Outport: '<Root>/Iq_ref' */
    *Iq_ref = rtM->dwork.Merge[1];
  } else {
    /* Outport: '<Root>/Iq_ref' incorporates:
     *  UnaryMinus: '<S5>/Unary Minus'
     */
    *Iq_ref = -rtM->dwork.Merge[1];
  }

  /* End of Switch: '<S5>/sign' */

  /* Outport: '<Root>/Id_ref' */
  *Id_ref = rtM->dwork.Merge[0];
}

/* Model initialize function */
void mtpa_initialize(RT_MODEL *const rtM)
{
  (void) (rtM);

  /* (no initialization code required) */
  UNUSED_PARAMETER(rtM);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
