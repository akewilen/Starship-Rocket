/*
 * Attidue.h
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

#ifndef Attidue_h_
#define Attidue_h_
#ifndef Attidue_COMMON_INCLUDES_
#define Attidue_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* Attidue_COMMON_INCLUDES_ */

#include "Attidue_types.h"
#include <string.h>
#include <float.h>
#include <stddef.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE[6];/* '<S1>/Discrete-Time Integrator' */
  real_T DiscreteStateSpace_DSTATE[6]; /* '<S1>/Discrete State-Space' */
} DW_Attidue_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T moments[3];                   /* '<Root>/moments' */
} ExtU_Attidue_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T y[3];                         /* '<Root>/y' */
  real_T x[6];                         /* '<Root>/x' */
  real_T Out1[6];                      /* '<Root>/Out1' */
} ExtY_Attidue_T;

/* Parameters (default storage) */
struct P_Attidue_T_ {
  real_T C_I[18];                      /* Variable: C_I
                                        * Referenced by: '<S1>/C'
                                        */
  real_T DiscreteTimeIntegrator_gainval;
                           /* Computed Parameter: DiscreteTimeIntegrator_gainval
                            * Referenced by: '<S1>/Discrete-Time Integrator'
                            */
  real_T DiscreteTimeIntegrator_IC;    /* Expression: 0
                                        * Referenced by: '<S1>/Discrete-Time Integrator'
                                        */
  real_T DiscreteStateSpace_A[11];   /* Computed Parameter: DiscreteStateSpace_A
                                      * Referenced by: '<S1>/Discrete State-Space'
                                      */
  real_T DiscreteStateSpace_B[6];    /* Computed Parameter: DiscreteStateSpace_B
                                      * Referenced by: '<S1>/Discrete State-Space'
                                      */
  real_T DiscreteStateSpace_C[6];    /* Computed Parameter: DiscreteStateSpace_C
                                      * Referenced by: '<S1>/Discrete State-Space'
                                      */
  real_T DiscreteStateSpace_InitialCondi;/* Expression: 0
                                          * Referenced by: '<S1>/Discrete State-Space'
                                          */
  real_T A_Gain[36];                   /* Expression: sys_att.A
                                        * Referenced by: '<S1>/A'
                                        */
  real_T Gain_Gain[18];                /* Expression: sys_att.B
                                        * Referenced by: '<S1>/Gain'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_Attidue_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_Attidue_T Attidue_P;

/* Block states (default storage) */
extern DW_Attidue_T Attidue_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Attidue_T Attidue_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Attidue_T Attidue_Y;

/* Model entry point functions */
extern void Attidue_initialize(void);
extern void Attidue_step(void);
extern void Attidue_terminate(void);

/* Real-time Model object */
extern RT_MODEL_Attidue_T *const Attidue_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('Rocket_Simulink/Attidue Plant')    - opens subsystem Rocket_Simulink/Attidue Plant
 * hilite_system('Rocket_Simulink/Attidue Plant/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Rocket_Simulink'
 * '<S1>'   : 'Rocket_Simulink/Attidue Plant'
 */
#endif                                 /* Attidue_h_ */
