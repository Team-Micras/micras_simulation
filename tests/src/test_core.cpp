/**
 * @file test_core.cpp
 *
 * @brief Core class to the test
 *
 * @date 04/2024
 */

#include <rclcpp/rclcpp.hpp>

#include "test_core.hpp"

using namespace std::chrono_literals;

namespace micras {
void TestCore::init(int argc, char** argv) {
    rclcpp::init(argc, argv);

    micras::micras_node = std::make_shared<rclcpp::Node>("micras_node");
}

void TestCore::loop(const std::function<void()>& loop_func) {
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_node");

    auto timer = test_node->create_wall_timer(1ms, [&loop_func]() { loop_func(); });

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(test_node);
    executor.add_node(micras::micras_node);

    executor.spin();
    rclcpp::shutdown();
}
}  // namespace micras
