#ifndef _SLROS_BUSMSG_CONVERSION_H_
#define _SLROS_BUSMSG_CONVERSION_H_

#include "rclcpp/rclcpp.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/header.hpp>
#include "test_module_types.h"
#include "slros_msgconvert_utils.h"


void convertFromBus(builtin_interfaces::msg::Time& msgPtr, SL_Bus_builtin_interfaces_Time const* busPtr);
void convertToBus(SL_Bus_builtin_interfaces_Time* busPtr, const builtin_interfaces::msg::Time& msgPtr);

void convertFromBus(sensor_msgs::msg::CompressedImage& msgPtr, SL_Bus_sensor_msgs_CompressedImage const* busPtr);
void convertToBus(SL_Bus_sensor_msgs_CompressedImage* busPtr, const sensor_msgs::msg::CompressedImage& msgPtr);

void convertFromBus(std_msgs::msg::Header& msgPtr, SL_Bus_std_msgs_Header const* busPtr);
void convertToBus(SL_Bus_std_msgs_Header* busPtr, const std_msgs::msg::Header& msgPtr);


#endif
