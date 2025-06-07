/**
 * @file target.hpp
 *
 * @brief Target specific configuration
 *
 * @date 03/2024
 */

#ifndef MICRAS_TARGET_HPP
#define MICRAS_TARGET_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "micras/proxy/argb.hpp"
#include "micras/proxy/battery.hpp"
#include "micras/proxy/button.hpp"
#include "micras/proxy/buzzer.hpp"
#include "micras/proxy/dip_switch.hpp"
#include "micras/proxy/wall_sensors.hpp"
#include "micras/proxy/fan.hpp"
#include "micras/proxy/imu.hpp"
#include "micras/proxy/led.hpp"
#include "micras/proxy/locomotion.hpp"
#include "micras/proxy/rotary_sensor.hpp"
#include "micras/proxy/storage.hpp"
#include "micras/proxy/torque_sensors.hpp"

// clang-format off
namespace micras {
/*****************************************
 * Template Instantiations
 *****************************************/

namespace proxy {
    using Argb = proxy::TArgb<2>;
    using DipSwitch = TDipSwitch<4>;
    using TorqueSensors = TTorqueSensors<2>;
    using WallSensors = TWallSensors<4>;
}  // namespace proxy


inline std::shared_ptr<rclcpp::Node> micras_node;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

const proxy::Stopwatch::Config stopwatch_config {
};

const proxy::Storage::Config maze_storage_config {
    .start_page = 2,
    .number_of_pages = 1 //unused
};

/*****************************************
 * Interface
 *****************************************/

const proxy::Led::Config led_config {
    micras_node, // node
    "led"        // topic
};

const proxy::Argb::Config argb_config {
    micras_node,  // node
    {
        "rgb_0",
        "rgb_1"
    },  // topic_array
};

const proxy::Button::Config button_config {
    micras_node, // node
    "button"     // topic
};

const proxy::DipSwitch::Config dip_switch_config {
    micras_node,  // node
    {
        "dip_switch_0",
        "dip_switch_1",
        "dip_switch_2",
        "dip_switch_3"
    },  // topic_array
};

const proxy::Buzzer::Config buzzer_config {
    micras_node, // node
    "buzzer"     // topic
};

/*****************************************
 * Sensors
 *****************************************/

const proxy::RotarySensor::Config rotary_sensor_left_config {
    micras_node,         // node
    "rotary_sensor_left" // topic
};

const proxy::RotarySensor::Config rotary_sensor_right_config {
    micras_node,          // node
    "rotary_sensor_right" // topic
};

const proxy::TorqueSensors::Config torque_sensors_config {
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
    },  // wheel_pairs_topics
    0.04F * 20, // shunt_resistor
    0.5F, // max_torque
    3.3F  // reference_voltage
};

const proxy::WallSensors::Config wall_sensors_config {
    micras_node,  // node
    {
        "wall_sensor_0",
        "wall_sensor_1",
        "wall_sensor_2",
        "wall_sensor_3"
    },    // topic_array
    0.3F, // max_distance
    4095,  // max_reading
    5.0F,  // filter_cutoff
    {
        0.413F,
        0.161F,
        0.177F,
        0.230F,
    },  // base_readings
    0.5F   // uncertainty
};

const proxy::Imu::Config imu_config {
    micras_node, // node
    "imu"        // topic
};

const proxy::Battery::Config battery_config {
    micras_node, // node
    "battery",   // topic
    9.9F,        // max_voltage
    4095         // max_reading
};


/*****************************************
 * Actuators
 *****************************************/

const proxy::Fan::Config fan_config {
    micras_node, // node
    "fan"        // topic
};

const proxy::Locomotion::Config locomotion_config {
    micras_node, // node
    "cmd_vel"    // topic
};
}  // namespace micras

// clang-format on

#endif  // MICRAS_TARGET_HPP
