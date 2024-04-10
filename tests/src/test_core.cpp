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
    auto timer = micras::micras_node->create_wall_timer(10ms, [&loop_func]() { loop_func(); });
    rclcpp::spin(micras::micras_node);

    rclcpp::shutdown();
}
}  // namespace micras
