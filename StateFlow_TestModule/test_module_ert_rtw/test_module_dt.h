//
//  test_module_dt.h
//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  Code generation for model "test_module".
//
//  Model version              : 1.1
//  Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
//  C++ source code generated on : Tue Oct 28 14:49:07 2025
//
//  Target selection: ert.tlc
//  Embedded hardware selection: Intel->x86-64 (Windows64)
//  Code generation objectives: Unspecified
//  Validation result: Not run


#include "ext_types.h"

// data type size table
static uint_T rtDataTypeSizes[] = {
  sizeof(real_T),
  sizeof(real32_T),
  sizeof(int8_T),
  sizeof(uint8_T),
  sizeof(int16_T),
  sizeof(uint16_T),
  sizeof(int32_T),
  sizeof(uint32_T),
  sizeof(boolean_T),
  sizeof(fcn_call_T),
  sizeof(int_T),
  sizeof(pointer_T),
  sizeof(action_T),
  2*sizeof(uint32_T),
  sizeof(int32_T),
  sizeof(SL_Bus_builtin_interfaces_Time),
  sizeof(SL_Bus_ROSVariableLengthArrayInfo),
  sizeof(SL_Bus_std_msgs_Header),
  sizeof(SL_Bus_sensor_msgs_CompressedImage),
  sizeof(ros_slros2_internal_block_Rea_T),
  sizeof(ros_slros2_internal_block_Sub_T),
  sizeof(uint_T),
  sizeof(char_T),
  sizeof(uchar_T),
  sizeof(time_T)
};

// data type name table
static const char_T * rtDataTypeNames[] = {
  "real_T",
  "real32_T",
  "int8_T",
  "uint8_T",
  "int16_T",
  "uint16_T",
  "int32_T",
  "uint32_T",
  "boolean_T",
  "fcn_call_T",
  "int_T",
  "pointer_T",
  "action_T",
  "timer_uint32_pair_T",
  "physical_connection",
  "SL_Bus_builtin_interfaces_Time",
  "SL_Bus_ROSVariableLengthArrayInfo",
  "SL_Bus_std_msgs_Header",
  "SL_Bus_sensor_msgs_CompressedImage",
  "ros_slros2_internal_block_Rea_T",
  "ros_slros2_internal_block_Sub_T",
  "uint_T",
  "char_T",
  "uchar_T",
  "time_T"
};

// data type transitions for block I/O structure
static DataTypeTransition rtBTransitions[] = {
  { (char_T *)(&test_module_B.ReadImage_o1[0]), 3, 0, 12000000 },

  { (char_T *)(&test_module_B.In1), 18, 0, 1 },

  { (char_T *)(&test_module_B.ReadImage_o2), 3, 0, 1 },

  { (char_T *)(&test_module_B.SourceBlock_o1), 8, 0, 1 }
  ,

  { (char_T *)(&test_module_DW.obj), 19, 0, 1 },

  { (char_T *)(&test_module_DW.obj_g), 20, 0, 1 },

  { (char_T *)(&test_module_DW.EnabledSubsystem_SubsysRanBC), 2, 0, 1 }
};

// data type transition table for block I/O structure
static DataTypeTransitionTable rtBTransTable = {
  7U,
  rtBTransitions
};

// data type transitions for Parameters structure
static DataTypeTransition rtPTransitions[] = {
  { (char_T *)(&test_module_P.Out1_Y0), 18, 0, 1 },

  { (char_T *)(&test_module_P.Constant_Value), 18, 0, 1 }
};

// data type transition table for Parameters structure
static DataTypeTransitionTable rtPTransTable = {
  2U,
  rtPTransitions
};

// [EOF] test_module_dt.h
