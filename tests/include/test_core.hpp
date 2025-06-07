/**
 * @file test_core.hpp
 *
 * @brief Core class to the test
 *
 * @date 04/2024
 */

#ifndef MICRAS_TEST_CORE_HPP
#define MICRAS_TEST_CORE_HPP

#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "target.hpp"

using namespace std::chrono_literals;

namespace micras {
template <typename F>
concept VoidFunction = requires(F void_function) {
    { void_function() } -> std::same_as<void>;
};

/**
 * @brief Core class to the tests.
 */
class TestCore {
public:
    /**
     * @brief Delete the default constructor
     */
    TestCore() = delete;

    /**
     * @brief Initialize the test core.
     *
     * @param argc Number of main arguments.
     * @param argv Main arguments.
     */
    static void init(int argc = 0, char** argv = nullptr) {
        rclcpp::init(argc, argv);

        micras::micras_node = std::make_shared<rclcpp::Node>("micras_node");
    }

    /**
     * @brief Loop the test core with a custom function.
     *
     * @param loop_func Custom loop function.
     */
    static void loop(VoidFunction auto loop_func) {
        rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test_node");

        auto timer = test_node->create_wall_timer(1ms, [&loop_func]() { loop_func(); });

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(test_node);
        executor.add_node(micras::micras_node);

        executor.spin();
        rclcpp::shutdown();
    }
};
}  // namespace micras

#endif  // MICRAS_TEST_CORE_HPP
