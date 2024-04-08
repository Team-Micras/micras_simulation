/**
 * @file target.cpp
 *
 * @brief Target configuration constants
 *
 * @date 03/2024
 */

#include "target.hpp"

// clang-format off
namespace micras {
std::shared_ptr<rclcpp::Node> micras_node;

const proxy::Argb<2>::Config argb_config {
    micras_node,  // node
    {
        "rgb_0",
        "rgb_1"
    },  // topic_array
};

const proxy::Battery::Config battery_config {
    micras_node, // node
    "battery"    // topic
};

const proxy::Button::Config button_config {
    micras_node, // node
    "button"     // topic
};

const proxy::Buzzer::Config buzzer_config {
    micras_node, // node
    "buzzer"     // topic
};

const proxy::DipSwitch<4>::Config dip_switch_config {
    micras_node,  // node
    {
        "dip_switch_0",
        "dip_switch_1",
        "dip_switch_2",
        "dip_switch_3"
    },  // topic_array
};

const proxy::DistanceSensors<4>::Config distance_sensors_config {
    micras_node,  // node
    {
        "distance_sensor_0",
        "distance_sensor_1",
        "distance_sensor_2",
        "distance_sensor_3"
    },    // topic_array
    0.3f, // max_distance
};

const proxy::Fan::Config fan_config {
    micras_node, // node
    "fan"        // topic
};

const proxy::Imu::Config imu_config {
    micras_node, // node
    "imu"        // topic
};

const proxy::Led::Config led_config {
    micras_node, // node
    "led"        // topic
};

const proxy::Locomotion::Config locomotion_config {
    micras_node, // node
    "cmd_vel"    // topic
};

const proxy::RotarySensor::Config rotary_sensor_left_config {
    micras_node,         // node
    "rotary_sensor_left" // topic
};

const proxy::RotarySensor::Config rotary_sensor_right_config {
    micras_node,          // node
    "rotary_sensor_right" // topic
};

const proxy::TorqueSensors<2>::Config torque_sensors_config {
    micras_node,  // node
    {
        {
            {
                "torque_flw",
                "torque_rlw"
            },  // front_topic, rear_topic
            {
                "torque_frw",
                "torque_rrw"
            }  // front_topic, rear_topic
        }
    }  // wheel_pairs_topics
};
}  // namespace micras

// clang-format on
