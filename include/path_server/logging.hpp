#ifndef _PATH_SERVER__LOGGING_HPP_
#define _PATH_SERVER__LOGGING_HPP_

#if __has_include("rclcpp/rclcpp.hpp")
#include "rclcpp/rclcpp.hpp"
#elif __has_include("ros/ros.h")
#include "ros/ros.h"
#define RCLCPP_DEBUG(logger, ...) ROS_DEBUG(__VA_ARGS__)
#define RCLCPP_DEBUG_THROTTLE(logger, clock, time_ms, ...) ROS_DEBUG_THROTTLE(time_ms/1000.0, __VA_ARGS__)
#define RCLCPP_INFO(logger, ...) ROS_INFO(__VA_ARGS__)
#define RCLCPP_INFO_THROTTLE(logger, clock, time_ms, ...) ROS_INFO_THROTTLE(time_ms/1000.0, __VA_ARGS__)
#define RCLCPP_WARN(logger, ...) ROS_WARN(__VA_ARGS__)
#define RCLCPP_WARN_THROTTLE(logger, clock, time_ms, ...) ROS_WARN_THROTTLE(time_ms/1000.0, __VA_ARGS__)
#define RCLCPP_ERROR(logger, ...) ROS_ERROR(__VA_ARGS__)
#define RCLCPP_ERROR_THROTTLE(logger, clock, time_ms, ...) ROS_ERROR_THROTTLE(time_ms/1000.0, __VA_ARGS__)
#define RCLCPP_FATAL(logger, ...) ROS_FATAL(__VA_ARGS__)
#define RCLCPP_FATAL_THROTTLE(logger, clock, time_ms, ...) ROS_FATAL_THROTTLE(time_ms/1000.0, __VA_ARTS__)
#endif

#endif
