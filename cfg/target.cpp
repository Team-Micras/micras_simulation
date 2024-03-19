/**
 * @file target.cpp
 *
 * @brief Target configuration constants
 *
 * @date 03/2024
 */

#include "target.hpp"

std::shared_ptr<rclcpp::Node> micras_node;

proxy::Button::Config button_config{
    micras_node, // node
    "button",    // topic
};

proxy::DistanceSensor::Config distance_sensor_0_config{
    micras_node,         // node
    "distance_sensor_0", // topic
};

proxy::DistanceSensor::Config distance_sensor_1_config{
    micras_node,         // node
    "distance_sensor_1", // topic
};

proxy::DistanceSensor::Config distance_sensor_2_config{
    micras_node,         // node
    "distance_sensor_2", // topic
};

proxy::DistanceSensor::Config distance_sensor_3_config{
    micras_node,         // node
    "distance_sensor_3", // topic
};

proxy::Encoder::Config encoder_left_config{
    micras_node,    // node
    "encoder_left", // topic
};

proxy::Encoder::Config encoder_right_config{
    micras_node,     // node
    "encoder_right", // topic
};

proxy::Imu::Config imu_config{
    micras_node, // node
    "imu",       // topic
};

proxy::Led::Config led_config{
    micras_node, // node
    "led",       // topic
};

proxy::Locomotion::Config locomotion_config{
    micras_node, // node
    "cmd_vel",   // topic
};

proxy::Odometry::Config odometry_config{
    micras_node, // node
    "odometry",  // topic
};

proxy::TorqueSensor::Config torque_sensor_left_config{
    micras_node,  // node
    "torque_flw", // front_topic
    "torque_rlw", // rear_topic
};

proxy::TorqueSensor::Config torque_sensor_right_config{
    micras_node,  // node
    "torque_frw", // front_topic
    "torque_rrw", // rear_topic
};
