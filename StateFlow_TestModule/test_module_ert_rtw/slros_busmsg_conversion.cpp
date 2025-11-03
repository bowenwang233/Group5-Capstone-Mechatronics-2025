#include "slros_busmsg_conversion.h"


// Conversions between SL_Bus_builtin_interfaces_Time and builtin_interfaces::msg::Time

void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  msgPtr.nanosec =  busPtr->nanosec;
  msgPtr.sec =  busPtr->sec;
}

void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr)
{
  const std::string rosMessageType("builtin_interfaces/Time");

  busPtr->nanosec =  msgPtr.nanosec;
  busPtr->sec =  msgPtr.sec;
}


// Conversions between SL_Bus_sensor_msgs_CompressedImage and sensor_msgs::msg::CompressedImage

void convertFromBus(sensor_msgs::msg::CompressedImage& msgPtr, SL_Bus_sensor_msgs_CompressedImage const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/CompressedImage");

  convertFromBusVariablePrimitiveArray(msgPtr.data, busPtr->data, busPtr->data_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr.format, busPtr->format, busPtr->format_SL_Info);
  convertFromBus(msgPtr.header, &busPtr->header);
}

void convertToBus(SL_Bus_sensor_msgs_CompressedImage* busPtr, const sensor_msgs::msg::CompressedImage& msgPtr)
{
  const std::string rosMessageType("sensor_msgs/CompressedImage");

  convertToBusVariablePrimitiveArray(busPtr->data, busPtr->data_SL_Info, msgPtr.data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBusVariablePrimitiveArray(busPtr->format, busPtr->format_SL_Info, msgPtr.format, slros::EnabledWarning(rosMessageType, "format"));
  convertToBus(&busPtr->header, msgPtr.header);
}


// Conversions between SL_Bus_std_msgs_Header and std_msgs::msg::Header

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr.frame_id, busPtr->frame_id, busPtr->frame_id_SL_Info);
  convertFromBus(msgPtr.stamp, &busPtr->stamp);
}

void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->frame_id, busPtr->frame_id_SL_Info, msgPtr.frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  convertToBus(&busPtr->stamp, msgPtr.stamp);
}

