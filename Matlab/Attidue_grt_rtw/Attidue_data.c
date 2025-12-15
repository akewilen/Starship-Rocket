/*
 * Attidue_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Attidue".
 *
 * Model version              : 1.26
 * Simulink Coder version : 24.2 (R2024b) 21-Jun-2024
 * C source code generated on : Sun Dec 14 22:27:30 2025
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Attidue.h"

/* Block parameters (default storage) */
P_Attidue_T Attidue_P = {
  /* Variable: C_I
   * Referenced by: '<S1>/C'
   */
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0 },

  /* Computed Parameter: DiscreteTimeIntegrator_gainval
   * Referenced by: '<S1>/Discrete-Time Integrator'
   */
  0.01,

  /* Expression: 0
   * Referenced by: '<S1>/Discrete-Time Integrator'
   */
  0.0,

  /* Computed Parameter: DiscreteStateSpace_A
   * Referenced by: '<S1>/Discrete State-Space'
   */
  { 1.0012430117966629, 0.24865384914431568, 1.0012430117966629,
    0.24865384914431568, 1.0, 0.01000414302940638, 1.0012430117966629,
    0.01000414302940638, 1.0012430117966629, 0.010000000000000002, 1.0 },

  /* Computed Parameter: DiscreteStateSpace_B
   * Referenced by: '<S1>/Discrete State-Space'
   */
  { 0.019425520445449287, 0.019425520445449287, 0.0625, 9.7107489600907184E-5,
    9.7107489600907184E-5, 0.00031250000000000006 },

  /* Computed Parameter: DiscreteStateSpace_C
   * Referenced by: '<S1>/Discrete State-Space'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: 0
   * Referenced by: '<S1>/Discrete State-Space'
   */
  0.0,

  /* Expression: sys_att.A
   * Referenced by: '<S1>/A'
   */
  { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 24.855087378640778, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    24.855087378640778, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  /* Expression: sys_att.B
   * Referenced by: '<S1>/Gain'
   */
  { 1.941747572815534, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.941747572815534, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 6.25, 0.0, 0.0, 0.0 }
};
