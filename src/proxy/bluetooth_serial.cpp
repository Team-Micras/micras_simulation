/**
 * @file
 */

#include "micras/proxy/bluetooth_serial.hpp"

#include <algorithm>
#include <functional>
#include <iostream>

namespace micras::proxy {

BluetoothSerial::BluetoothSerial(const Config& config) : port{port} {
    init_websocket_server();
}

BluetoothSerial::~BluetoothSerial() {
    this->running = false;
    this->server.stop();

    if (this->server.is_listening()) {
        this->server.stop_listening();
    }

    {
        std::lock_guard<std::mutex> lock(this->connection_mutex);
        for (auto& hdl : this->connections) {
            try {
                this->server.close(hdl, websocketpp::close::status::going_away, "Server shutting down");
            } catch (const std::exception& e) {
                std::cerr << "Error closing connection: " << e.what() << std::endl;
            }
        }
        this->connections.clear();
    }

    if (this->server_thread.joinable()) {
        this->server_thread.join();
    }
}

void BluetoothSerial::init_websocket_server() {
    try {
        this->server.clear_access_channels(websocketpp::log::alevel::all);
        this->server.set_error_channels(websocketpp::log::elevel::fatal);

        this->server.init_asio();

        this->server.set_reuse_addr(true);

        this->server.set_open_handler(std::bind(&BluetoothSerial::on_open, this, std::placeholders::_1));
        this->server.set_close_handler(std::bind(&BluetoothSerial::on_close, this, std::placeholders::_1));
        this->server.set_message_handler(
            std::bind(&BluetoothSerial::on_message, this, std::placeholders::_1, std::placeholders::_2)
        );

        this->server.listen(this->port);
        this->server.start_accept();

        this->server_thread = std::thread([this]() {
            try {
                this->server.run();
            } catch (const std::exception& e) {
                std::cerr << "WebSocket server error: " << e.what() << std::endl;
            }
        });

        std::cout << "WebSocket server started on port " << this->port;
        std::cout << " -> ws://localhost:" << this->port << std::endl;
    } catch (const websocketpp::exception& e) {
        std::cerr << "WebSocket initialization error: " << e.what() << std::endl;
        throw;
    } catch (const std::exception& e) {
        std::cerr << "General error during WebSocket initialization: " << e.what() << std::endl;
        throw;
    }
}

void BluetoothSerial::update() {
    std::vector<uint8_t> bytes_to_send;

    {
        std::lock_guard<std::mutex> lock(this->tx_mutex);
        if (!this->tx_queue.empty()) {
            bytes_to_send = std::move(this->tx_queue);
            this->tx_queue.clear();
        }
    }

    if (!bytes_to_send.empty()) {
        std::lock_guard<std::mutex> lock(this->connection_mutex);
        for (auto& hdl : this->connections) {
            try {
                auto connection = this->server.get_con_from_hdl(hdl);
                if (connection->get_state() == websocketpp::session::state::open) {
                    this->server.send(
                        hdl, bytes_to_send.data(), bytes_to_send.size(), websocketpp::frame::opcode::binary
                    );
                }
            } catch (const std::exception& e) {
                std::cerr << "Error sending data: " << e.what() << std::endl;
            }
        }
    }
}

void BluetoothSerial::send_data(std::vector<uint8_t> data) {
    std::lock_guard<std::mutex> lock(this->tx_mutex);
    this->tx_queue.insert(this->tx_queue.end(), data.begin(), data.end());
}

std::vector<uint8_t> BluetoothSerial::get_data() {
    std::lock_guard<std::mutex> lock(this->rx_mutex);
    std::vector<uint8_t>        data = std::move(this->received_data);
    this->received_data.clear();
    return data;
}

bool BluetoothSerial::is_running() const {
    return this->running && this->server.is_listening();
}

void BluetoothSerial::on_open(ConnectionHdl hdl) {
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    this->connections.insert(hdl);
    std::cout << "WebSocket connection opened. Total connections: " << this->connections.size() << std::endl;
}

void BluetoothSerial::on_close(ConnectionHdl hdl) {
    std::lock_guard<std::mutex> lock(this->connection_mutex);
    this->connections.erase(hdl);
    std::cout << "WebSocket connection closed. Total connections: " << this->connections.size() << std::endl;
}

void BluetoothSerial::on_message(ConnectionHdl hdl, MessagePtr msg) {
    auto payload = msg->get_payload();

    {
        std::lock_guard<std::mutex> lock(rx_mutex);

        if (msg->get_opcode() == websocketpp::frame::opcode::binary) {
            this->received_data.insert(
                this->received_data.end(), reinterpret_cast<const uint8_t*>(payload.data()),
                reinterpret_cast<const uint8_t*>(payload.data() + payload.size())
            );
        } else {
            this->received_data.insert(this->received_data.end(), payload.begin(), payload.end());
        }
    }
}

}  // namespace micras::proxy
