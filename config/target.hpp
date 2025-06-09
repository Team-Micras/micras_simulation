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
#include "micras/proxy/bluetooth_serial.hpp"

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
    .node = micras_node,
    .topic = "led"
};

const proxy::Argb::Config argb_config {
    .node = micras_node,
    .topic_array = {
        "rgb_0",
        "rgb_1"
    }
};

const proxy::Button::Config button_config {
    .node = micras_node,
    .topic = "button",
    .long_press_delay = 4000U,
    .extra_long_press_delay = 4001U
};

const proxy::DipSwitch::Config dip_switch_config {
    .node = micras_node,
    .topic_array = {
        "dip_switch_0",
        "dip_switch_1",
        "dip_switch_2",
        "dip_switch_3"
    },
};

const proxy::Buzzer::Config buzzer_config {
    .node = micras_node,
    .topic = "buzzer"
};

const proxy::BluetoothSerial::Config bluetooth_config = {
    .port = 8080,
};

/*****************************************
 * Sensors
 *****************************************/

const proxy::RotarySensor::Config rotary_sensor_left_config {
    .node = micras_node,
    .topic = "sensors/encoder_left"
};

const proxy::RotarySensor::Config rotary_sensor_right_config {
    .node = micras_node,
    .topic = "sensors/encoder_right"
};

const proxy::TorqueSensors::Config torque_sensors_config {
    .node = micras_node,
    .wheel_pairs_topics = {
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
    },
    .shunt_resistor = 0.04F * 20,
    .max_torque = 3.0F,
    .reference_voltage = 3.3F
};

const proxy::WallSensors::Config wall_sensors_config {
    .node = micras_node,
    .topic_array = {
        "sensors/lidar_0",
        "sensors/lidar_1",
        "sensors/lidar_2",
        "sensors/lidar_3"
    },
    .uncertainty = 0.5F,
    .base_readings = {
        0.0696F,
        0.1090F,
        0.1090F,
        0.0696F,
    },
    .max_sensor_reading = 0.6F,
    .min_sensor_reading = 0.01F,
    .max_sensor_distance = 0.18F * 2,
    .filter_cutoff = 20.0F
};

const proxy::Imu::Config imu_config {
    .node = micras_node,
    .gyro_topic = "sensors/gyro",
    .accelerometer_topic = "sensors/accelerometer"
};

const proxy::Battery::Config battery_config {
    .node = micras_node,
    .topic = "battery",
    .max_voltage = 9.9F,
    .max_reading = 4095
};


/*****************************************
 * Actuators
 *****************************************/

const proxy::Fan::Config fan_config {
    .node = micras_node,
    .topic = "actuators/fan/command"
};

const proxy::Locomotion::Config locomotion_config {
    .left_motor = {
        .node = micras_node,
        .topic = "actuators/motor_left/command",
    },
    .right_motor = {
        .node = micras_node,
        .topic = "actuators/motor_right/command"
    }
};
}  // namespace micras

// clang-format on

#endif  // MICRAS_TARGET_HPP
