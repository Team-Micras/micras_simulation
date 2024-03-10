#include <cstdio>

#include "controller/micras_controller_test.hpp"
#include "target.hpp"

int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    proxy::Led led(led_config);
    proxy::Button button(button_config);

    for (;;) {
        micras_controller_test_loop(led, button);
    }
}
