#ifndef GZ_GUI_MICRAS_PLUGIN_HPP
#define GZ_GUI_MICRAS_PLUGIN_HPP

#include <array>
#include <gz/transport.hh>
#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>
#include <string>

namespace gz::gui {

/**
 * @brief MicrasPlugin class
 */
class MicrasPlugin : public Plugin {
    Q_OBJECT

public:
    /**
     * @brief Construct a new Micras Plugin object
     */
    MicrasPlugin();

    /**
     * @brief Load the plugin configuration
     *
     * @param _pluginElem Plugin configuration xml element
     */
    void LoadConfig(const tinyxml2::XMLElement* _pluginElem) override;

signals:
    /**
     * @brief Signal emitted when the led state changes
     *
     * @param new_state New led state
     */
    void led_state_changed(bool new_state);

    /**
     * @brief Signal emitted when the rgb_0 led color changes
     *
     * @param red Red color component
     * @param green Green color component
     * @param blue Blue color component
     */
    void led_rgb_0_changed(float red, float green, float blue);

    /**
     * @brief Signal emitted when the rgb_1 led color changes
     *
     * @param red Red color component
     * @param green Green color component
     * @param blue Blue color component
     */
    void led_rgb_1_changed(float red, float green, float blue);

    /**
     * @brief Signal emitted when the buzzer frequency changes
     *
     * @param freq New buzzer frequency
     */
    void buzzer_changed(int freq);

    /**
     * @brief Signal emitted when the fan speed changes
     *
     * @param speed New fan speed
     */
    void fan_speed_changed(float speed);

protected slots:
    /**
     * @brief Slot called when the button is clicked
     */
    void on_button_click();

    /**
     * @brief Slot called when the button is released
     */
    void on_button_release();

    /**
     * @brief Slot called when the slider value changes
     *
     * @param value New slider value
     */
    void on_slider_change(float value);

    /**
     * @brief Slot called when a switch state changes
     *
     * @param index Switch index
     * @param state New switch state
     */
    void set_switch_state(int index, bool state);

private:
    /**
     * @brief Set the gazebo transport publishers
     */
    void set_publishers();

    /**
     * @brief Set the gazebo transport subscriber for the led topic
     */
    void set_led_subscriber();

    /**
     * @brief Set the gazebo transport subscriber for the rgb_0 topic
     */
    void set_rgb_0_subscriber();

    /**
     * @brief Set the gazebo transport subscriber for the rgb_1 topic
     */
    void set_rgb_1_subscriber();

    /**
     * @brief Set the gazebo transport subscriber for the buzzer topic
     */
    void set_buzzer_subscriber();

    /**
     * @brief Set the gazebo transport subscriber for the fan topic
     */
    void set_fan_subscriber();

    /**
     * @brief Gazebo transport node
     */
    gz::transport::Node node;

    /**
     * @brief Gazebo transport publishers
     */
    std::array<gz::transport::v13::Node::Publisher, 4> switch_pub;

    /**
     * @brief Gazebo transport publisher for the button topic
     */
    gz::transport::v13::Node::Publisher button_pub;

    /**
     * @brief Gazebo transport publisher for the battery topic
     */
    gz::transport::v13::Node::Publisher battery_pub;

    /**
     * @brief Plugin topic names
     */
    const std::string button_topic{"/button"};
    const std::string battery_topic{"/battery"};
    const std::string dip_switch_topic{"/dip_switch_"};
    const std::string led_topic{"/led"};
    const std::string led_rgb_0_topic{"/rgb_0"};
    const std::string led_rgb_1_topic{"/rgb_1"};
    const std::string buzzer_topic{"/buzzer"};
    const std::string fan_topic{"/fan"};
};
}  // namespace gz::gui

#endif  // GZ_GUI_MICRAS_PLUGIN_HPP
