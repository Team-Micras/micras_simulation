/**
 * @file micras_node.cpp
 *
 * @brief Micras Node class implementation
 *
 * @date 03/2024
 */

#include <rclcpp/rclcpp.hpp>

#include "target.hpp"
#include "micras/micras_controller.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    micras::micras_node = std::make_shared<rclcpp::Node>("micras_node");
    micras::MicrasController micras_controller;

    auto timer = micras::micras_node->create_wall_timer(10ms, [&micras_controller]() { micras_controller.run(); });

    rclcpp::spin(micras::micras_node);

    rclcpp::shutdown();

    return 0;
}
