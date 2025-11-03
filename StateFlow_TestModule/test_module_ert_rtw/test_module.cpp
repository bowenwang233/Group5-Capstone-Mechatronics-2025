//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: test_module.cpp
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
#include "test_module.h"
#include "test_module_types.h"
#include "rtwtypes.h"
#include <string.h>
#include "rmw/qos_profiles.h"
#include <stddef.h>
#include "test_module_private.h"
#include "test_module_dt.h"

// Block signals (default storage)
B_test_module_T test_module_B;

// Block states (default storage)
DW_test_module_T test_module_DW;

// Real-time model
RT_MODEL_test_module_T test_module_M_ = RT_MODEL_test_module_T();
RT_MODEL_test_module_T *const test_module_M = &test_module_M_;

// Forward declaration for local functions
static void test_module_char(const uint8_T varargin_1_data[], const int32_T
  varargin_1_size[2], char_T y_data[], int32_T y_size[2]);
static boolean_T test_module_strcmp(const char_T a_data[], const int32_T a_size
  [2]);
static void test_module_strcmp_c(const char_T b_data[], const int32_T b_size[2],
  boolean_T b_bool[3]);
static boolean_T test_module_vectorAny(const boolean_T x_data[], const int32_T
  x_size[2]);
static void test__ReadImage_decompressImage(const uint8_T busstruct_data[128],
  uint32_T busstruct_data_SL_Info_CurrentL, uint8_T image_data[], int32_T
  image_size[3], uint8_T *errorCode);
static uint8_T tes_ReadImage_updateBusAndState(ros_slros2_internal_block_Rea_T
  *obj, const uint8_T busstruct_format[128], uint32_T
  busstruct_format_SL_Info_Curren, const uint8_T busstruct_data[128], uint32_T
  busstruct_data_SL_Info_CurrentL, uint32_T busstruct_data_SL_Info_Received);
static void test_module_ReadImage_stepImpl(ros_slros2_internal_block_Rea_T *obj,
  const uint8_T busstruct_format[128], uint32_T busstruct_format_SL_Info_Curren,
  const uint8_T busstruct_data[128], uint32_T busstruct_data_SL_Info_CurrentL,
  uint32_T busstruct_data_SL_Info_Received, uint8_T varargout_1[12000000],
  uint8_T *varargout_2);
static void test_module_SystemCore_step(ros_slros2_internal_block_Rea_T *obj,
  const uint8_T varargin_1_format[128], uint32_T varargin_1_format_SL_Info_Curre,
  const uint8_T varargin_1_data[128], uint32_T varargin_1_data_SL_Info_Current,
  uint32_T varargin_1_data_SL_Info_Receive, uint8_T varargout_1[12000000],
  uint8_T *varargout_2);
static void test_module_SystemCore_setup(ros_slros2_internal_block_Sub_T *obj);
static void test_module_ReadImage_resetImpl(ros_slros2_internal_block_Rea_T *obj);
static void test_module_char(const uint8_T varargin_1_data[], const int32_T
  varargin_1_size[2], char_T y_data[], int32_T y_size[2])
{
  int32_T loop_ub;
  y_size[0] = 1;

  // Start for MATLABSystem: '<Root>/Read Image'
  y_size[1] = varargin_1_size[1];
  loop_ub = varargin_1_size[1];
  for (int32_T y_data_tmp = 0; y_data_tmp < loop_ub; y_data_tmp++) {
    // Start for MATLABSystem: '<Root>/Read Image'
    y_data[y_data_tmp] = static_cast<int8_T>(varargin_1_data[y_data_tmp]);
  }
}

static boolean_T test_module_strcmp(const char_T a_data[], const int32_T a_size
  [2])
{
  int32_T minnanb;
  boolean_T b_bool;
  static const char_T tmp[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\a', '\b', '\t', '\n', '\v', '\f', '\r', '\x0e', '\x0f',
    '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17', '\x18',
    '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!', '\"', '#',
    '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'a',
    'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p',
    'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']', '^', '_',
    '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}',
    '~', '\x7f' };

  static const char_T tmp_0[5] = { 'r', 'g', 'b', '8', ';' };

  b_bool = false;

  // Start for MATLABSystem: '<Root>/Read Image'
  if (a_size[1] <= 5) {
    minnanb = a_size[1];
  } else {
    minnanb = 5;
  }

  if ((minnanb >= 5) || (a_size[1] == 5)) {
    minnanb = 1;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (minnanb - 1 < 5) {
        if (tmp[static_cast<uint8_T>(a_data[minnanb - 1]) & 127] != tmp[
            static_cast<int32_T>(tmp_0[minnanb - 1])]) {
          exitg1 = 1;
        } else {
          minnanb++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  // End of Start for MATLABSystem: '<Root>/Read Image'
  return b_bool;
}

static void test_module_strcmp_c(const char_T b_data[], const int32_T b_size[2],
  boolean_T b_bool[3])
{
  int32_T b_kstr;
  static const char_T tmp[4] = { 'j', 'p', 'e', 'g' };

  static const char_T tmp_0[3] = { 'j', 'p', 'g' };

  static const char_T tmp_1[3] = { 'p', 'n', 'g' };

  int32_T exitg1;
  b_bool[0] = false;

  // Start for MATLABSystem: '<Root>/Read Image'
  if (b_size[1] == 4) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 4) {
        if (tmp[b_kstr - 1] != b_data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool[0] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[1] = false;

  // Start for MATLABSystem: '<Root>/Read Image'
  if (b_size[1] == 3) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 3) {
        if (tmp_0[b_kstr - 1] != b_data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool[1] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  b_bool[2] = false;

  // Start for MATLABSystem: '<Root>/Read Image'
  if (b_size[1] == 3) {
    b_kstr = 1;
    do {
      exitg1 = 0;
      if (b_kstr - 1 < 3) {
        if (tmp_1[b_kstr - 1] != b_data[b_kstr - 1]) {
          exitg1 = 1;
        } else {
          b_kstr++;
        }
      } else {
        b_bool[2] = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
}

static boolean_T test_module_vectorAny(const boolean_T x_data[], const int32_T
  x_size[2])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k <= x_size[1] - 1)) {
    if (x_data[b_k]) {
      y = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  return y;
}

static void test__ReadImage_decompressImage(const uint8_T busstruct_data[128],
  uint32_T busstruct_data_SL_Info_CurrentL, uint8_T image_data[], int32_T
  image_size[3], uint8_T *errorCode)
{
  int32_T height;
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T loop_ub;
  int32_T loop_ub_0;
  int32_T width;
  boolean_T height_data[3];
  height = 2000;
  width = 2000;
  memset(&test_module_B.fullImage[0], 0, 12000000U * sizeof(uint8_T));
  decompressImage<3>(&busstruct_data[0], busstruct_data_SL_Info_CurrentL,
                     &test_module_B.fullImage[0], &width, &height);

  // Start for MATLABSystem: '<Root>/Read Image'
  if (height < 1) {
    loop_ub = 0;
  } else {
    loop_ub = height;
  }

  if (width < 1) {
    loop_ub_0 = 0;
  } else {
    loop_ub_0 = width;
  }

  image_size[0] = loop_ub;
  image_size[1] = loop_ub_0;
  image_size[2] = 3;
  for (i = 0; i < 3; i++) {
    for (i_1 = 0; i_1 < loop_ub_0; i_1++) {
      for (i_0 = 0; i_0 < loop_ub; i_0++) {
        image_data[(i_0 + loop_ub * i_1) + loop_ub * loop_ub_0 * i] =
          test_module_B.fullImage[(2000 * i_1 + i_0) + 4000000 * i];
      }
    }
  }

  *errorCode = 0U;

  // Start for MATLABSystem: '<Root>/Read Image'
  test_module_B.height_size[0] = 1;
  test_module_B.height_size[1] = 2;
  height_data[0] = (height > 2000);
  height_data[1] = (width > 2000);
  if (test_module_vectorAny(height_data, test_module_B.height_size)) {
    *errorCode = 2U;
  }
}

static uint8_T tes_ReadImage_updateBusAndState(ros_slros2_internal_block_Rea_T
  *obj, const uint8_T busstruct_format[128], uint32_T
  busstruct_format_SL_Info_Curren, const uint8_T busstruct_data[128], uint32_T
  busstruct_data_SL_Info_CurrentL, uint32_T busstruct_data_SL_Info_Received)
{
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T loop_ub;
  int32_T loop_ub_0;
  uint8_T errorCode;
  boolean_T tmp[3];
  static const int32_T tmp_0[2] = { 1, 3 };

  boolean_T guard1;
  if (busstruct_data_SL_Info_CurrentL < busstruct_data_SL_Info_Received) {
    errorCode = 3U;
  } else {
    if (busstruct_format_SL_Info_Curren < 1U) {
      loop_ub = 0;
    } else {
      // Start for MATLABSystem: '<Root>/Read Image'
      loop_ub = static_cast<int32_T>(busstruct_format_SL_Info_Curren);
    }

    // Start for MATLABSystem: '<Root>/Read Image'
    test_module_B.busstruct_format_size[0] = 1;
    test_module_B.busstruct_format_size[1] = loop_ub;
    if (loop_ub - 1 >= 0) {
      memcpy(&test_module_B.busstruct_format_data[0], &busstruct_format[0],
             static_cast<uint32_T>(loop_ub) * sizeof(uint8_T));
    }

    test_module_char(test_module_B.busstruct_format_data,
                     test_module_B.busstruct_format_size,
                     test_module_B.signalFormat_data,
                     test_module_B.signalFormat_size);

    // Start for MATLABSystem: '<Root>/Read Image'
    guard1 = false;
    if (!test_module_strcmp(test_module_B.signalFormat_data,
                            test_module_B.signalFormat_size)) {
      test_module_strcmp_c(test_module_B.signalFormat_data,
                           test_module_B.signalFormat_size, tmp);
      if (!test_module_vectorAny(tmp, tmp_0)) {
        errorCode = 1U;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      test__ReadImage_decompressImage(busstruct_data,
        busstruct_data_SL_Info_CurrentL, test_module_B.rawImage_data,
        test_module_B.rawImage_size, &errorCode);
      if (errorCode == 0) {
        memset(&obj->Image[0], 0, 12000000U * sizeof(uint8_T));
        loop_ub = test_module_B.rawImage_size[1];
        for (i = 0; i < 3; i++) {
          for (i_1 = 0; i_1 < loop_ub; i_1++) {
            loop_ub_0 = test_module_B.rawImage_size[0];
            for (i_0 = 0; i_0 < loop_ub_0; i_0++) {
              obj->Image[(i_0 + 2000 * i_1) + 4000000 * i] =
                test_module_B.rawImage_data[(test_module_B.rawImage_size[0] *
                i_1 + i_0) + test_module_B.rawImage_size[0] *
                test_module_B.rawImage_size[1] * i];
            }
          }
        }

        obj->ImageSize[0] = static_cast<uint32_T>(test_module_B.rawImage_size[0]);
        obj->ImageSize[1] = static_cast<uint32_T>(test_module_B.rawImage_size[1]);
        errorCode = 0U;
      }
    }
  }

  return errorCode;
}

static void test_module_ReadImage_stepImpl(ros_slros2_internal_block_Rea_T *obj,
  const uint8_T busstruct_format[128], uint32_T busstruct_format_SL_Info_Curren,
  const uint8_T busstruct_data[128], uint32_T busstruct_data_SL_Info_CurrentL,
  uint32_T busstruct_data_SL_Info_Received, uint8_T varargout_1[12000000],
  uint8_T *varargout_2)
{
  *varargout_2 = tes_ReadImage_updateBusAndState(obj, busstruct_format,
    busstruct_format_SL_Info_Curren, busstruct_data,
    busstruct_data_SL_Info_CurrentL, busstruct_data_SL_Info_Received);

  // Start for MATLABSystem: '<Root>/Read Image'
  memcpy(&varargout_1[0], &obj->Image[0], 12000000U * sizeof(uint8_T));
}

static void test_module_SystemCore_step(ros_slros2_internal_block_Rea_T *obj,
  const uint8_T varargin_1_format[128], uint32_T varargin_1_format_SL_Info_Curre,
  const uint8_T varargin_1_data[128], uint32_T varargin_1_data_SL_Info_Current,
  uint32_T varargin_1_data_SL_Info_Receive, uint8_T varargout_1[12000000],
  uint8_T *varargout_2)
{
  test_module_ReadImage_stepImpl(obj, varargin_1_format,
    varargin_1_format_SL_Info_Curre, varargin_1_data,
    varargin_1_data_SL_Info_Current, varargin_1_data_SL_Info_Receive,
    varargout_1, varargout_2);
}

static void test_module_SystemCore_setup(ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  char_T b_zeroDelimTopic[36];
  static const char_T b_zeroDelimTopic_0[36] =
    "/depth_cam/rgb/image_raw/compressed";

  // Start for MATLABSystem: '<S1>/SourceBlock'
  obj->isInitialized = 1;
  qos_profile = rmw_qos_profile_default;

  // Start for MATLABSystem: '<S1>/SourceBlock'
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  for (int32_T i = 0; i < 36; i++) {
    // Start for MATLABSystem: '<S1>/SourceBlock'
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_test_module_2.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
  obj->isSetupComplete = true;
}

static void test_module_ReadImage_resetImpl(ros_slros2_internal_block_Rea_T *obj)
{
  // Start for MATLABSystem: '<Root>/Read Image'
  memset(&obj->Image[0], 0, 12000000U * sizeof(uint8_T));
  obj->ImageSize[0] = 2000U;
  obj->ImageSize[1] = 2000U;
}

// Model step function
void test_module_step(void)
{
  // Reset subsysRan breadcrumbs
  srClearBC(test_module_DW.EnabledSubsystem_SubsysRanBC);

  // MATLABSystem: '<S1>/SourceBlock'
  test_module_B.SourceBlock_o1 = Sub_test_module_2.getLatestMessage
    (&test_module_B.b_varargout_2);

  // Outputs for Enabled SubSystem: '<S1>/Enabled Subsystem' incorporates:
  //   EnablePort: '<S2>/Enable'

  if (test_module_B.SourceBlock_o1) {
    // SignalConversion generated from: '<S2>/In1' incorporates:
    //   MATLABSystem: '<S1>/SourceBlock'
    //
    test_module_B.In1 = test_module_B.b_varargout_2;
    srUpdateBC(test_module_DW.EnabledSubsystem_SubsysRanBC);
  }

  // End of Outputs for SubSystem: '<S1>/Enabled Subsystem'

  // MATLABSystem: '<Root>/Read Image'
  test_module_SystemCore_step(&test_module_DW.obj, test_module_B.In1.format,
    test_module_B.In1.format_SL_Info.CurrentLength, test_module_B.In1.data,
    test_module_B.In1.data_SL_Info.CurrentLength,
    test_module_B.In1.data_SL_Info.ReceivedLength, test_module_B.ReadImage_o1,
    &test_module_B.ReadImage_o2);

  // External mode
  rtExtModeUploadCheckTrigger(1);

  {                                    // Sample time: [0.2s, 0.0s]
    rtExtModeUpload(0, (real_T)test_module_M->Timing.taskTime0);
  }

  // signal main to stop simulation
  {                                    // Sample time: [0.2s, 0.0s]
    if ((rtmGetTFinal(test_module_M)!=-1) &&
        !((rtmGetTFinal(test_module_M)-test_module_M->Timing.taskTime0) >
          test_module_M->Timing.taskTime0 * (DBL_EPSILON))) {
      rtmSetErrorStatus(test_module_M, "Simulation finished");
    }

    if (rtmGetStopRequested(test_module_M)) {
      rtmSetErrorStatus(test_module_M, "Simulation finished");
    }
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  test_module_M->Timing.taskTime0 =
    ((time_T)(++test_module_M->Timing.clockTick0)) *
    test_module_M->Timing.stepSize0;
}

// Model initialize function
void test_module_initialize(void)
{
  // Registration code
  rtmSetTFinal(test_module_M, -1);
  test_module_M->Timing.stepSize0 = 0.2;

  // External mode info
  test_module_M->Sizes.checksums[0] = (1584982554U);
  test_module_M->Sizes.checksums[1] = (2039898799U);
  test_module_M->Sizes.checksums[2] = (163018589U);
  test_module_M->Sizes.checksums[3] = (3395989695U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[4];
    test_module_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = (sysRanDType *)&test_module_DW.EnabledSubsystem_SubsysRanBC;
    systemRan[3] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(test_module_M->extModeInfo,
      &test_module_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(test_module_M->extModeInfo,
                        test_module_M->Sizes.checksums);
    rteiSetTPtr(test_module_M->extModeInfo, rtmGetTPtr(test_module_M));
  }

  // data type transition information
  {
    static DataTypeTransInfo dtInfo;
    test_module_M->SpecialInfo.mappingInfo = (&dtInfo);
    dtInfo.numDataTypes = 25;
    dtInfo.dataTypeSizes = &rtDataTypeSizes[0];
    dtInfo.dataTypeNames = &rtDataTypeNames[0];

    // Block I/O transition table
    dtInfo.BTransTable = &rtBTransTable;

    // Parameters transition table
    dtInfo.PTransTable = &rtPTransTable;
  }

  // SystemInitialize for Enabled SubSystem: '<S1>/Enabled Subsystem'
  // SystemInitialize for SignalConversion generated from: '<S2>/In1' incorporates:
  //   Outport: '<S2>/Out1'

  test_module_B.In1 = test_module_P.Out1_Y0;

  // End of SystemInitialize for SubSystem: '<S1>/Enabled Subsystem'

  // Start for MATLABSystem: '<S1>/SourceBlock'
  test_module_SystemCore_setup(&test_module_DW.obj_g);

  // Start for MATLABSystem: '<Root>/Read Image'
  test_module_DW.obj.isInitialized = 1;

  // InitializeConditions for MATLABSystem: '<Root>/Read Image'
  test_module_ReadImage_resetImpl(&test_module_DW.obj);
}

// Model terminate function
void test_module_terminate(void)
{
  // Terminate for MATLABSystem: '<S1>/SourceBlock'
  if (!test_module_DW.obj_g.matlabCodegenIsDeleted) {
    test_module_DW.obj_g.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S1>/SourceBlock'
}

//
// File trailer for generated code.
//
// [EOF]
//
