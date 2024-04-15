/**
 * @file micras_node.cpp
 *
 * @brief Micras Node class implementation
 *
 * @date 03/2024
 */

#include <rclcpp/rclcpp.hpp>

#include "micras/micras_controller.hpp"
#include "target.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    micras::micras_node = std::make_shared<rclcpp::Node>("micras_node");
    rclcpp::Node::SharedPtr main_node = std::make_shared<rclcpp::Node>("main_node");

    rclcpp::executors::MultiThreadedExecutor executor;

    micras::MicrasController micras_controller;

    try {
        auto timer = main_node->create_wall_timer(10ms, [&micras_controller]() { micras_controller.run(); });

        executor.add_node(main_node);
        executor.add_node(micras::micras_node);
        executor.spin();

    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(micras::micras_node->get_logger(), e.what());
    }

    rclcpp::shutdown();

    return 0;
}
