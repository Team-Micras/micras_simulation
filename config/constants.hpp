/**
 * @file
 */

#ifndef MICRAS_CONSTANTS_HPP
#define MICRAS_CONSTANTS_HPP

#include <cstdint>
#include <numbers>

#include "micras/nav/action_queuer.hpp"
#include "micras/nav/follow_wall.hpp"
#include "micras/nav/maze.hpp"
#include "micras/nav/odometry.hpp"
#include "micras/nav/speed_controller.hpp"

namespace micras {
/***************
 * Constants
 ***************/

constexpr uint8_t  maze_width{16};
constexpr uint8_t  maze_height{16};
constexpr float    cell_size{0.18};
constexpr uint32_t loop_time_us{1042};
constexpr float    wall_thickness{0.012F};
constexpr float    start_offset{0.05F + wall_thickness / 2.0F};
constexpr float    max_angular_acceleration{400.0F};
constexpr float    crash_acceleration{1000000.0F};
constexpr float    fan_speed{100.0F};

constexpr core::WallSensorsIndex wall_sensors_index{
    .left_front = 0,
    .left = 1,
    .right = 2,
    .right_front = 3,
};

/***************
 * Template Instantiations
 ***************/

namespace nav {
using Maze = TMaze<maze_width, maze_height>;
}  // namespace nav

/***************
 * Configurations
 ***************/

const nav::ActionQueuer::Config action_queuer_config{
    .cell_size = cell_size,
    .start_offset = start_offset,
    .curve_safety_margin = 0.0375F + 0.015F,
    .exploring =
        {
            .max_linear_speed = 0.2F,
            .max_linear_acceleration = 9.0F,
            .max_linear_deceleration = 9.0F,
            .max_centrifugal_acceleration = 2.0F,
            .max_angular_acceleration = 300.0F,
        },
    .solving =
        {
            .max_linear_speed = 5.0F,
            .max_linear_acceleration = 12.0F,
            .max_linear_deceleration = 20.0F,
            .max_centrifugal_acceleration = 40.0F,
            .max_angular_acceleration = max_angular_acceleration,
        },
};

const nav::FollowWall::Config follow_wall_config{
    .pid =
        {
            .kp = 30.0F,
            .ki = 0.0F,
            .kd = 0.008F,
            .setpoint = 0.0F,
            .saturation = 1.0F,
            .max_integral = -1.0F,
        },
    .wall_sensor_index = wall_sensors_index,
    .max_angular_acceleration = max_angular_acceleration,
    .cell_size = cell_size,
    .post_threshold = 4.0F,
    .post_reference = 0.44F * cell_size,
    .post_clearance = 0.035F,
};

const nav::Maze::Config maze_config{
    .start = {{0, 0}, nav::Side::UP},
    .goal = {{
        {maze_width / 2, maze_height / 2},
        {(maze_width - 1) / 2, maze_height / 2},
        {maze_width / 2, (maze_height - 1) / 2},
        {(maze_width - 1) / 2, (maze_height - 1) / 2},
    }},
    .cost_margin = 1.2F,
    .action_queuer_config = action_queuer_config,
};

const nav::Odometry::Config odometry_config{
    .linear_cutoff_frequency = 50.0F,
    .wheel_radius = 0.0112F,
    .initial_pose = {{cell_size / 2.0f, start_offset}, std::numbers::pi_v<float> / 2.0f},
};

const nav::SpeedController::Config speed_controller_config{
    .linear_pid =
        {
            .kp = 10.0F,
            .ki = 10.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 20.0F,
            .max_integral = -1.0F,
        },
    .angular_pid =
        {
            .kp = 0.7F,
            .ki = 5.0F,
            .kd = 0.0F,
            .setpoint = 0.0F,
            .saturation = 20.0F,
            .max_integral = -1.0F,
        },
    .left_feed_forward =
        {
            .linear_speed = 16.4F,
            .linear_acceleration = 3.98F,
            .angular_speed = -0.55F,
            .angular_acceleration = -0.143F,
        },
    .right_feed_forward =
        {
            .linear_speed = 16.4F,
            .linear_acceleration = 3.98F,
            .angular_speed = +0.55F,
            .angular_acceleration = +0.143F,
        },
};
}  // namespace micras

#endif  // MICRAS_CONSTANTS_HPP
