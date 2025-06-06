/**
 * @file
 */

#ifndef MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
#define MICRAS_PROXY_BLUETOOTH_SERIAL_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

namespace micras::proxy {

/**
 * @brief Mock class for bluetooth serial that communicates through WebSockets instead of UART
 */
class BluetoothSerial {
public:
    using WebSocketServer = websocketpp::server<websocketpp::config::asio>;
    using ConnectionPtr = WebSocketServer::connection_ptr;
    using ConnectionHdl = websocketpp::connection_hdl;
    using MessagePtr = WebSocketServer::message_ptr;

    /**
     * @brief Configuration struct for the BluetoothSerial.
     */
    struct Config {
        uint16_t port = 9002;
    };

    /**
     * @brief Construct a new BluetoothSerial object.
     *
     * @param config Configuration for the BluetoothSerial.
     */
    explicit BluetoothSerial(const Config& config);

    /**
     * @brief Destructor for BluetoothSerial
     */
    ~BluetoothSerial();

    /**
     * @brief Process all received messages and send queued data.
     *
     * This method handles the actual sending and receiving of data via WebSocket.
     * It should be called regularly in the main loop.
     */
    void update();

    /**
     * @brief Queue data to be sent during the next update call.
     *
     * @param data Bytes to be sent over WebSocket.
     */
    void send_data(std::vector<uint8_t> data);

    /**
     * @brief Get data that has been received since the last call.
     *
     * @return Vector containing received bytes (will be empty if no new data)
     */
    std::vector<uint8_t> get_data();

private:
    /**
     * @brief Initialize the WebSocket server
     */
    void init_websocket_server();

    /**
     * @brief Handle a new WebSocket connection
     *
     * @param hdl Handle to the connection
     */
    void on_open(ConnectionHdl hdl);

    /**
     * @brief Handle a closed WebSocket connection
     *
     * @param hdl Handle to the connection
     */
    void on_close(ConnectionHdl hdl);

    /**
     * @brief Handle a message received from a WebSocket connection
     *
     * @param hdl Handle to the connection
     * @param msg The received message
     */
    void on_message(ConnectionHdl hdl, MessagePtr msg);

    /**
     * @brief WebSocket server instance
     */
    WebSocketServer server;

    /**
     * @brief Thread for the WebSocket server
     */
    std::thread server_thread;

    /**
     * @brief Mutex to protect the received data
     */
    std::mutex rx_mutex;

    /**
     * @brief Mutex to protect the data to be sent
     */
    std::mutex tx_mutex;

    /**
     * @brief Buffer for received data waiting to be retrieved
     */
    std::vector<uint8_t> received_data;

    /**
     * @brief Buffer for data waiting to be sent in the next update
     */
    std::vector<uint8_t> tx_queue;

    /**
     * @brief Set of active connections
     */
    std::set<ConnectionHdl, std::owner_less<ConnectionHdl>> connections;

    /**
     * @brief Mutex to protect the connections set
     */
    std::mutex connection_mutex;

    /**
     * @brief Port on which the WebSocket server listens
     */
    uint16_t port;
};

}  // namespace micras::proxy

#endif  // MICRAS_PROXY_BLUETOOTH_SERIAL_HPP
