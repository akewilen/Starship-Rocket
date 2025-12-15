/*
 * Attidue.c
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
#include "rtwtypes.h"
#include <emmintrin.h>
#include <string.h>
#include "Attidue_private.h"

/* Block states (default storage) */
DW_Attidue_T Attidue_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Attidue_T Attidue_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Attidue_T Attidue_Y;

/* Real-time model */
static RT_MODEL_Attidue_T Attidue_M_;
RT_MODEL_Attidue_T *const Attidue_M = &Attidue_M_;

/* Model step function */
void Attidue_step(void)
{
  __m128d tmp_1;
  __m128d tmp_2;
  __m128d tmp_3;
  real_T tmp[6];
  real_T tmp_0[6];
  real_T y;
  int32_T i;
  int32_T i_0;

  /* DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
   *  Inport: '<Root>/moments'
   *  Outport: '<Root>/Out1'
   */
  {
    Attidue_Y.Out1[0] = (Attidue_P.DiscreteStateSpace_C[0])*
      Attidue_DW.DiscreteStateSpace_DSTATE[0];
    Attidue_Y.Out1[1] = (Attidue_P.DiscreteStateSpace_C[1])*
      Attidue_DW.DiscreteStateSpace_DSTATE[1];
    Attidue_Y.Out1[2] = (Attidue_P.DiscreteStateSpace_C[2])*
      Attidue_DW.DiscreteStateSpace_DSTATE[2];
    Attidue_Y.Out1[3] = (Attidue_P.DiscreteStateSpace_C[3])*
      Attidue_DW.DiscreteStateSpace_DSTATE[3];
    Attidue_Y.Out1[4] = (Attidue_P.DiscreteStateSpace_C[4])*
      Attidue_DW.DiscreteStateSpace_DSTATE[4];
    Attidue_Y.Out1[5] = (Attidue_P.DiscreteStateSpace_C[5])*
      Attidue_DW.DiscreteStateSpace_DSTATE[5];
  }

  /* Outport: '<Root>/y' incorporates:
   *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
   *  Gain: '<S1>/C'
   */
  for (i_0 = 0; i_0 < 3; i_0++) {
    y = 0.0;
    for (i = 0; i < 6; i++) {
      y += Attidue_P.C_I[3 * i + i_0] *
        Attidue_DW.DiscreteTimeIntegrator_DSTATE[i];
    }

    Attidue_Y.y[i_0] = y;
  }

  /* End of Outport: '<Root>/y' */
  for (i = 0; i < 6; i++) {
    /* Outport: '<Root>/x' incorporates:
     *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
     */
    Attidue_Y.x[i] = Attidue_DW.DiscreteTimeIntegrator_DSTATE[i];

    /* Gain: '<S1>/Gain' incorporates:
     *  Inport: '<Root>/moments'
     */
    tmp[i] = (Attidue_P.Gain_Gain[i + 6] * Attidue_U.moments[1] +
              Attidue_P.Gain_Gain[i] * Attidue_U.moments[0]) +
      Attidue_P.Gain_Gain[i + 12] * Attidue_U.moments[2];

    /* Gain: '<S1>/A' incorporates:
     *  DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
     */
    y = 0.0;
    for (i_0 = 0; i_0 < 6; i_0++) {
      y += Attidue_P.A_Gain[6 * i_0 + i] *
        Attidue_DW.DiscreteTimeIntegrator_DSTATE[i_0];
    }

    tmp_0[i] = y;

    /* End of Gain: '<S1>/A' */
  }

  /* Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' incorporates:
   *  Sum: '<S1>/Sum'
   */
  for (i_0 = 0; i_0 <= 4; i_0 += 2) {
    /* Sum: '<S1>/Sum' */
    tmp_1 = _mm_loadu_pd(&tmp[i_0]);
    tmp_2 = _mm_loadu_pd(&tmp_0[i_0]);
    tmp_3 = _mm_loadu_pd(&Attidue_DW.DiscreteTimeIntegrator_DSTATE[i_0]);
    _mm_storeu_pd(&Attidue_DW.DiscreteTimeIntegrator_DSTATE[i_0], _mm_add_pd
                  (_mm_mul_pd(_mm_add_pd(tmp_1, tmp_2), _mm_set1_pd
      (Attidue_P.DiscreteTimeIntegrator_gainval)), tmp_3));
  }

  /* End of Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */

  /* Update for DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
   *  Inport: '<Root>/moments'
   *  Outport: '<Root>/Out1'
   */
  {
    real_T xnew[6];
    xnew[0] = (Attidue_P.DiscreteStateSpace_A[0])*
      Attidue_DW.DiscreteStateSpace_DSTATE[0]
      + (Attidue_P.DiscreteStateSpace_A[1])*
      Attidue_DW.DiscreteStateSpace_DSTATE[3];
    xnew[0] += (Attidue_P.DiscreteStateSpace_B[0])*Attidue_U.moments[0];
    xnew[1] = (Attidue_P.DiscreteStateSpace_A[2])*
      Attidue_DW.DiscreteStateSpace_DSTATE[1]
      + (Attidue_P.DiscreteStateSpace_A[3])*
      Attidue_DW.DiscreteStateSpace_DSTATE[4];
    xnew[1] += (Attidue_P.DiscreteStateSpace_B[1])*Attidue_U.moments[1];
    xnew[2] = (Attidue_P.DiscreteStateSpace_A[4])*
      Attidue_DW.DiscreteStateSpace_DSTATE[2];
    xnew[2] += (Attidue_P.DiscreteStateSpace_B[2])*Attidue_U.moments[2];
    xnew[3] = (Attidue_P.DiscreteStateSpace_A[5])*
      Attidue_DW.DiscreteStateSpace_DSTATE[0]
      + (Attidue_P.DiscreteStateSpace_A[6])*
      Attidue_DW.DiscreteStateSpace_DSTATE[3];
    xnew[3] += (Attidue_P.DiscreteStateSpace_B[3])*Attidue_U.moments[0];
    xnew[4] = (Attidue_P.DiscreteStateSpace_A[7])*
      Attidue_DW.DiscreteStateSpace_DSTATE[1]
      + (Attidue_P.DiscreteStateSpace_A[8])*
      Attidue_DW.DiscreteStateSpace_DSTATE[4];
    xnew[4] += (Attidue_P.DiscreteStateSpace_B[4])*Attidue_U.moments[1];
    xnew[5] = (Attidue_P.DiscreteStateSpace_A[9])*
      Attidue_DW.DiscreteStateSpace_DSTATE[2]
      + (Attidue_P.DiscreteStateSpace_A[10])*
      Attidue_DW.DiscreteStateSpace_DSTATE[5];
    xnew[5] += (Attidue_P.DiscreteStateSpace_B[5])*Attidue_U.moments[2];
    (void) memcpy(&Attidue_DW.DiscreteStateSpace_DSTATE[0], xnew,
                  sizeof(real_T)*6);
  }

  /* Matfile logging */
  rt_UpdateTXYLogVars(Attidue_M->rtwLogInfo, (&Attidue_M->Timing.taskTime0));

  /* signal main to stop simulation */
  {                                    /* Sample time: [0.01s, 0.0s] */
    if ((rtmGetTFinal(Attidue_M)!=-1) &&
        !((rtmGetTFinal(Attidue_M)-Attidue_M->Timing.taskTime0) >
          Attidue_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(Attidue_M, "Simulation finished");
    }
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   * Timer of this task consists of two 32 bit unsigned integers.
   * The two integers represent the low bits Timing.clockTick0 and the high bits
   * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
   */
  if (!(++Attidue_M->Timing.clockTick0)) {
    ++Attidue_M->Timing.clockTickH0;
  }

  Attidue_M->Timing.taskTime0 = Attidue_M->Timing.clockTick0 *
    Attidue_M->Timing.stepSize0 + Attidue_M->Timing.clockTickH0 *
    Attidue_M->Timing.stepSize0 * 4294967296.0;
}

/* Model initialize function */
void Attidue_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)Attidue_M, 0,
                sizeof(RT_MODEL_Attidue_T));
  rtmSetTFinal(Attidue_M, 10.0);
  Attidue_M->Timing.stepSize0 = 0.01;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (NULL);
    Attidue_M->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo(Attidue_M->rtwLogInfo, (NULL));
    rtliSetLogXSignalPtrs(Attidue_M->rtwLogInfo, (NULL));
    rtliSetLogT(Attidue_M->rtwLogInfo, "tout");
    rtliSetLogX(Attidue_M->rtwLogInfo, "");
    rtliSetLogXFinal(Attidue_M->rtwLogInfo, "");
    rtliSetLogVarNameModifier(Attidue_M->rtwLogInfo, "rt_");
    rtliSetLogFormat(Attidue_M->rtwLogInfo, 4);
    rtliSetLogMaxRows(Attidue_M->rtwLogInfo, 0);
    rtliSetLogDecimation(Attidue_M->rtwLogInfo, 1);
    rtliSetLogY(Attidue_M->rtwLogInfo, "");
    rtliSetLogYSignalInfo(Attidue_M->rtwLogInfo, (NULL));
    rtliSetLogYSignalPtrs(Attidue_M->rtwLogInfo, (NULL));
  }

  /* states (dwork) */
  (void) memset((void *)&Attidue_DW, 0,
                sizeof(DW_Attidue_T));

  /* external inputs */
  (void)memset(&Attidue_U, 0, sizeof(ExtU_Attidue_T));

  /* external outputs */
  (void)memset(&Attidue_Y, 0, sizeof(ExtY_Attidue_T));

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime(Attidue_M->rtwLogInfo, 0.0, rtmGetTFinal
    (Attidue_M), Attidue_M->Timing.stepSize0, (&rtmGetErrorStatus(Attidue_M)));

  {
    int32_T i;

    /* InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */
    for (i = 0; i < 6; i++) {
      Attidue_DW.DiscreteTimeIntegrator_DSTATE[i] =
        Attidue_P.DiscreteTimeIntegrator_IC;
    }

    /* End of InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' */

    /* InitializeConditions for DiscreteStateSpace: '<S1>/Discrete State-Space' incorporates:
     *  Inport: '<Root>/moments'
     *  Outport: '<Root>/Out1'
     */
    {
      int_T i1;
      real_T *dw_DSTATE = &Attidue_DW.DiscreteStateSpace_DSTATE[0];
      for (i1=0; i1 < 6; i1++) {
        dw_DSTATE[i1] = Attidue_P.DiscreteStateSpace_InitialCondi;
      }
    }
  }
}

/* Model terminate function */
void Attidue_terminate(void)
{
  /* (no terminate code required) */
}
