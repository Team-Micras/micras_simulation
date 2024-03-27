/**
 * @file target.hpp
 *
 * @brief Target configuration constants
 *
 * @date 03/2024
 */

#ifndef __TARGET_HPP__
#define __TARGET_HPP__

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "proxy/button.hpp"
#include "proxy/distance_sensor.hpp"
#include "proxy/encoder.hpp"
#include "proxy/imu.hpp"
#include "proxy/led.hpp"
#include "proxy/locomotion.hpp"
#include "proxy/odometry.hpp"
#include "proxy/torque_sensor.hpp"

extern std::shared_ptr<rclcpp::Node> micras_node;

extern proxy::Button::Config button_config;
extern proxy::DistanceSensors<4>::Config distance_sensors_config;
extern proxy::Encoder::Config encoder_left_config;
extern proxy::Encoder::Config encoder_right_config;
extern proxy::Imu::Config imu_config;
extern proxy::Led::Config led_config;
extern proxy::Locomotion::Config locomotion_config;
extern proxy::Odometry::Config odometry_config;
extern proxy::TorqueSensor::Config torque_sensor_left_config;
extern proxy::TorqueSensor::Config torque_sensor_right_config;

#endif // __TARGET_HPP__
