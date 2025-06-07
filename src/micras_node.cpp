/**
 * @file
 */

#include <rclcpp/rclcpp.hpp>

#include "micras/micras.hpp"
#include "target.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    micras::micras_node = std::make_shared<rclcpp::Node>("micras_node");
    rclcpp::Node::SharedPtr main_node = std::make_shared<rclcpp::Node>("main_node");

    rclcpp::executors::MultiThreadedExecutor executor;

    micras::Micras micras;

    try {
        auto timer = main_node->create_wall_timer(10ms, [&micras]() { micras.update(); });

        executor.add_node(main_node);
        executor.add_node(micras::micras_node);
        executor.spin();

    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(micras::micras_node->get_logger(), e.what());
    }

    rclcpp::shutdown();

    return 0;
}
