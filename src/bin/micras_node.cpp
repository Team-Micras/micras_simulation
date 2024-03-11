/**
 * @file micras_node.cpp
 *
 * @brief Micras Node class implementation
 *
 * @date 03/2024
 */

#include <rclcpp/rclcpp.hpp>

#include "controller/micras_controller.hpp"
#include "target.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    micras_node = std::make_shared<rclcpp::Node>("micras_node");
    MicrasController micras_controller;

    while (true) {
        rclcpp::spin_some(micras_node);
        micras_controller.run();
    }

    rclcpp::shutdown();

    return 0;
}
