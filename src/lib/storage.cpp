/**
 * @file storage.cpp
 *
 * @brief Proxy Storage class source
 *
 * @date 04/2024
 */

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "micras/proxy/storage.hpp"

namespace micras::proxy {
Storage::Storage(const Config& config) : start_page{config.start_page} {
    uint64_t header{};

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("micras_simulation");
    this->storage_path = package_share_directory + "/storage/";
    this->storage_file = "storage_" + std::to_string(this->start_page) + ".bin";

    if (!std::filesystem::exists(this->storage_path)) {
        std::filesystem::create_directory(this->storage_path);
    }

    std::ifstream file(this->storage_path + this->storage_file, std::ios::binary);

    if (!file.is_open()) {
        std::ofstream file(this->storage_path + this->storage_file, std::ios::binary);
    }

    file.read(reinterpret_cast<char*>(&header), sizeof(header));

    if (header >> 48 != start_symbol) {
        return;
    }

    uint16_t total_size = header >> 32;
    uint16_t num_primitives = header >> 16;
    uint16_t num_serializables = header;

    this->buffer.resize(8L * total_size);

    file.read(reinterpret_cast<char*>(this->buffer.data()), 8L * total_size);
    file.close();

    this->primitives = deserialize_var_map<PrimitiveVariable>(this->buffer, num_primitives);
    this->serializables = deserialize_var_map<SerializableVariable>(this->buffer, num_serializables);
}

void Storage::create(const std::string& name, const ISerializable& data) {
    this->serializables[name].ram_pointer = &data;
}

void Storage::sync(const std::string& name, ISerializable& data) {
    if (this->serializables.contains(name) and this->serializables.at(name).ram_pointer == nullptr) {
        const auto& serializable = this->serializables.at(name);
        data.deserialize(&this->buffer.at(serializable.buffer_address), serializable.size);
    }

    this->create(name, data);
}

void Storage::save() {
    this->buffer.clear();

    for (auto& [name, variable] : this->primitives) {
        if (variable.ram_pointer == nullptr) {
            this->primitives.erase(name);
            continue;
        }

        const auto* aux = reinterpret_cast<const uint8_t*>(variable.ram_pointer);
        variable.buffer_address = buffer.size();
        this->buffer.insert(this->buffer.end(), aux, aux + variable.size);
    }

    for (auto& [name, variable] : this->serializables) {
        if (variable.ram_pointer == nullptr) {
            this->serializables.erase(name);
            continue;
        }

        std::vector<uint8_t> aux = variable.ram_pointer->serialize();
        variable.buffer_address = this->buffer.size();
        variable.size = aux.size();
        this->buffer.insert(this->buffer.end(), aux.begin(), aux.end());
    }

    auto serialized_serializables = serialize_var_map<SerializableVariable>(this->serializables);
    this->buffer.insert(this->buffer.begin(), serialized_serializables.begin(), serialized_serializables.end());

    auto serialized_primitives = serialize_var_map<PrimitiveVariable>(this->primitives);
    this->buffer.insert(this->buffer.begin(), serialized_primitives.begin(), serialized_primitives.end());

    this->buffer.insert(this->buffer.end(), (8 - (this->buffer.size() % 8)) % 8, 0);
    uint16_t total_size = this->buffer.size() / 8;

    std::array<uint8_t, 8> header;

    header[0] = this->serializables.size();
    header[1] = this->serializables.size() >> 8;
    header[2] = this->primitives.size();
    header[3] = this->primitives.size() >> 8;

    header[4] = total_size;
    header[5] = total_size >> 8;
    header[6] = start_symbol;
    header[7] = start_symbol >> 8;

    this->buffer.insert(this->buffer.begin(), header.begin(), header.end());

    std::ofstream file(this->storage_path + this->storage_file, std::ios::binary);
    file.write(reinterpret_cast<char*>(this->buffer.data()), this->buffer.size());
    file.close();
}

template <typename T>
std::vector<uint8_t> Storage::serialize_var_map(const std::unordered_map<std::string, T>& variables) {
    std::vector<uint8_t> buffer;

    for (auto [name, variable] : variables) {
        buffer.emplace_back(name.size());
        buffer.insert(buffer.end(), name.begin(), name.end());

        buffer.emplace_back(variable.buffer_address);
        buffer.emplace_back(variable.buffer_address >> 8);

        buffer.emplace_back(variable.size);
        buffer.emplace_back(variable.size >> 8);
    }

    return buffer;
}

template <typename T>
std::unordered_map<std::string, T> Storage::deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars) {
    std::unordered_map<std::string, T> variables;

    uint16_t current_addr = 0;

    for (uint16_t decoded_vars = 0; decoded_vars < num_vars; decoded_vars++) {
        uint8_t var_name_len = buffer.at(current_addr);

        std::string var_name(buffer.begin() + current_addr + 1, buffer.begin() + current_addr + 1 + var_name_len);
        current_addr += var_name_len + 1;

        variables[var_name].buffer_address = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;

        variables.at(var_name).size = buffer.at(current_addr) | buffer.at(current_addr + 1) << 8;
        current_addr += 2;
    }

    buffer.erase(buffer.begin(), buffer.begin() + current_addr);
    return variables;
}

// Explicit instantiation of template functions
template std::vector<uint8_t>
    Storage::serialize_var_map(const std::unordered_map<std::string, PrimitiveVariable>& variables);

template std::unordered_map<std::string, Storage::PrimitiveVariable>
    Storage::deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars);

template std::vector<uint8_t>
    Storage::serialize_var_map(const std::unordered_map<std::string, SerializableVariable>& variables);

template std::unordered_map<std::string, Storage::SerializableVariable>
    Storage::deserialize_var_map(std::vector<uint8_t>& buffer, uint16_t num_vars);
}  // namespace micras::proxy
