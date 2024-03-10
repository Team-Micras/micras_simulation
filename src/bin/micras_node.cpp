/**
 * @file micras_node.cpp
 *
 * @brief Micras Node class implementation
 *
 * @date 03/2024
 */

#include <chrono>

#include "controller/micras_controller_test.hpp"
#include "micras_node.hpp"
#include "target.hpp"

MicrasNode::MicrasNode(Private) : Node("micras_node") {
    using namespace std::chrono_literals;

    this->timer = this->create_wall_timer(10ms, std::bind(&MicrasNode::timer_callback, this));

    this->button = std::make_shared<proxy::Button>(button_config);
    this->led = std::make_shared<proxy::Led>(led_config);

    this->executor.add_node(button);
    this->executor.add_node(led);
}

std::shared_ptr<MicrasNode> MicrasNode::create() {
    return std::make_shared<MicrasNode>(Private());
}

void MicrasNode::run() {
    this->executor.add_node(shared_from_this());
    this->executor.spin();
}

void MicrasNode::timer_callback() {
    micras_controller_test_loop(*(this->button), *(this->led));
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto micras_node = MicrasNode::create();

    micras_node->run();
    rclcpp::shutdown();

    return 0;
}
