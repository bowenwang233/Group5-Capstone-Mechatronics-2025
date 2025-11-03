//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: test_module.h
//
// Code generated for Simulink model 'test_module'.
//
// Model version                  : 1.1
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Tue Oct 28 14:49:07 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_test_module_h_
#define RTW_HEADER_test_module_h_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "dt_info.h"
#include "ext_work.h"
#include "slros_read_image.h"
#include "slros2_initialize.h"
#include "test_module_types.h"
#include <float.h>
#include <stddef.h>

// Macros for accessing real-time model data structure
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
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

// Block signals (default storage)
struct B_test_module_T {
  uint8_T ReadImage_o1[12000000];      // '<Root>/Read Image'
  uint8_T rawImage_data[12000000];
  uint8_T fullImage[12000000];
  SL_Bus_sensor_msgs_CompressedImage In1;// '<S2>/In1'
  SL_Bus_sensor_msgs_CompressedImage b_varargout_2;
  char_T signalFormat_data[128];
  uint8_T busstruct_format_data[128];
  int32_T rawImage_size[3];
  int32_T signalFormat_size[2];
  int32_T busstruct_format_size[2];
  int32_T height_size[2];
  uint8_T ReadImage_o2;                // '<Root>/Read Image'
  boolean_T SourceBlock_o1;            // '<S1>/SourceBlock'
};

// Block states (default storage) for system '<Root>'
struct DW_test_module_T {
  ros_slros2_internal_block_Rea_T obj; // '<Root>/Read Image'
  ros_slros2_internal_block_Sub_T obj_g;// '<S1>/SourceBlock'
  int8_T EnabledSubsystem_SubsysRanBC; // '<S1>/Enabled Subsystem'
};

// Parameters (default storage)
struct P_test_module_T_ {
  SL_Bus_sensor_msgs_CompressedImage Out1_Y0;// Computed Parameter: Out1_Y0
                                                //  Referenced by: '<S2>/Out1'

  SL_Bus_sensor_msgs_CompressedImage Constant_Value;// Computed Parameter: Constant_Value
                                                       //  Referenced by: '<S1>/Constant'

};

// Real-time Model Data Structure
struct tag_RTM_test_module_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  //
  //  Sizes:
  //  The following substructure contains sizes information
  //  for many of the model attributes such as inputs, outputs,
  //  dwork, sample times, etc.

  struct {
    uint32_T checksums[4];
  } Sizes;

  //
  //  SpecialInfo:
  //  The following substructure contains special information
  //  related to other components that are dependent on RTW.

  struct {
    const void *mappingInfo;
  } SpecialInfo;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_test_module_T test_module_P;

#ifdef __cplusplus

}

#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_test_module_T test_module_B;

#ifdef __cplusplus

}

#endif

// Block states (default storage)
extern struct DW_test_module_T test_module_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void test_module_initialize(void);
  extern void test_module_step(void);
  extern void test_module_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_test_module_T *const test_module_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'test_module'
//  '<S1>'   : 'test_module/Subscribe'
//  '<S2>'   : 'test_module/Subscribe/Enabled Subsystem'

#endif                                 // RTW_HEADER_test_module_h_

//
// File trailer for generated code.
//
// [EOF]
//
