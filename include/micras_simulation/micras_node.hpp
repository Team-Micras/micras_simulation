/**
 * @file micras_node.hpp
 *
 * @brief Micras Node class header
 *
 * @date 03/2024
 */

#ifndef __MICRAS_NODE_HPP__
#define __MICRAS_NODE_HPP__

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "proxy/button.hpp"
#include "proxy/led.hpp"

class MicrasNode :
    public rclcpp::Node {
    private:
        /**
         * @brief Struct to make the constructor inaccessible
         */
        struct Private { };

    public:
        /**
         * @brief Construct a new Micras Node object
         */
        MicrasNode(Private);

        /**
         * @brief Create a new MicrasNode
         *
         * @return std::shared_ptr<MicrasNode> to the new node
         */
        static std::shared_ptr<MicrasNode> create();

        /**
         * @brief Run the node
         */
        void run();

    private:
        /**
         * @brief Timer to make the loop time constant
         */
        rclcpp::TimerBase::SharedPtr timer;

        /**
         * @brief Executor to run all the nodes
         */
        rclcpp::executors::MultiThreadedExecutor executor;

        /**
         * @brief Shared pointer to the button proxy
         */
        std::shared_ptr<proxy::Button> button;

        /**
         * @brief Shared pointer to the led proxy
         */
        std::shared_ptr<proxy::Led> led;

        /**
         * @brief Timer callback
         */
        void timer_callback();
};

#endif // __MICRAS_NODE_HPP__
