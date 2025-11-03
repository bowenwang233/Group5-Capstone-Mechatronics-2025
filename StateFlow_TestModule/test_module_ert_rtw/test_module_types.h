//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: test_module_types.h
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
#ifndef RTW_HEADER_test_module_types_h_
#define RTW_HEADER_test_module_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_
#define DEFINED_TYPEDEF_FOR_SL_Bus_builtin_interfaces_Time_

// MsgType=builtin_interfaces/Time
struct SL_Bus_builtin_interfaces_Time
{
  int32_T sec;
  uint32_T nanosec;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Header_

// MsgType=std_msgs/Header
struct SL_Bus_std_msgs_Header
{
  // MsgType=builtin_interfaces/Time
  SL_Bus_builtin_interfaces_Time stamp;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=frame_id_SL_Info:TruncateAction=warn 
  uint8_T frame_id[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=frame_id
  SL_Bus_ROSVariableLengthArrayInfo frame_id_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_CompressedImage_
#define DEFINED_TYPEDEF_FOR_SL_Bus_sensor_msgs_CompressedImage_

// MsgType=sensor_msgs/CompressedImage
struct SL_Bus_sensor_msgs_CompressedImage
{
  // MsgType=std_msgs/Header
  SL_Bus_std_msgs_Header header;

  // PrimitiveROSType=string:IsVarLen=1:VarLenCategory=data:VarLenElem=format_SL_Info:TruncateAction=warn 
  uint8_T format[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=format
  SL_Bus_ROSVariableLengthArrayInfo format_SL_Info;

  // IsVarLen=1:VarLenCategory=data:VarLenElem=data_SL_Info:TruncateAction=warn
  uint8_T data[128];

  // IsVarLen=1:VarLenCategory=length:VarLenElem=data
  SL_Bus_ROSVariableLengthArrayInfo data_SL_Info;
};

#endif

// Custom Type definition for MATLABSystem: '<S1>/SourceBlock'
#include "rmw/qos_profiles.h"
#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
};

#endif                                // struct_ros_slros2_internal_block_Sub_T

#ifndef struct_ros_slros2_internal_block_Rea_T
#define struct_ros_slros2_internal_block_Rea_T

struct ros_slros2_internal_block_Rea_T
{
  int32_T isInitialized;
  uint8_T Image[12000000];
  uint32_T ImageSize[2];
};

#endif                                // struct_ros_slros2_internal_block_Rea_T

// Parameters (default storage)
typedef struct P_test_module_T_ P_test_module_T;

// Forward declaration for rtModel
typedef struct tag_RTM_test_module_T RT_MODEL_test_module_T;

#endif                                 // RTW_HEADER_test_module_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
