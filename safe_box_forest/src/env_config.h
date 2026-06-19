#pragma once

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace rbf::detail {

inline std::string env_string_or_empty(const char* name) {
    const char* raw = std::getenv(name);
    return raw != nullptr ? std::string(raw) : std::string();
}

inline int env_int_or_default(const char* name, int fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long value = std::strtol(raw, &end, 10);
    if (end == raw) {
        return fallback;
    }
    return static_cast<int>(value);
}

inline double env_double_or_default(const char* name, double fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const double value = std::strtod(raw, &end);
    if (end == raw || !std::isfinite(value)) {
        return fallback;
    }
    return value;
}

inline bool env_flag_or_default(const char* name, bool fallback) {
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return fallback;
    }
    char* end = nullptr;
    const long value = std::strtol(raw, &end, 10);
    if (end != raw) {
        return value != 0;
    }
    return raw[0] == 't' || raw[0] == 'T' ||
           raw[0] == 'y' || raw[0] == 'Y';
}

inline int env_indexed_int_or_default(const char* prefix, int index, int fallback) {
    if (index < 0) {
        return fallback;
    }
    const std::string indexed = std::string(prefix) + "_" + std::to_string(index);
    return env_int_or_default(indexed.c_str(), fallback);
}

inline double env_indexed_double_or_default(const char* prefix, int index, double fallback) {
    if (index < 0) {
        return fallback;
    }
    const std::string indexed = std::string(prefix) + "_" + std::to_string(index);
    return env_double_or_default(indexed.c_str(), fallback);
}

inline std::vector<int> env_int_list_or_empty(const char* name) {
    std::vector<int> values;
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return values;
    }
    std::stringstream stream(raw);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        }), item.end());
        if (item.empty()) {
            continue;
        }
        char* end = nullptr;
        const long value = std::strtol(item.c_str(), &end, 10);
        if (end != item.c_str()) {
            values.push_back(static_cast<int>(value));
        }
    }
    return values;
}

inline std::vector<double> env_double_list_or_empty(const char* name) {
    std::vector<double> values;
    const char* raw = std::getenv(name);
    if (raw == nullptr || raw[0] == '\0') {
        return values;
    }
    std::stringstream stream(raw);
    std::string item;
    while (std::getline(stream, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), [](unsigned char c) {
            return std::isspace(c) != 0;
        }), item.end());
        if (item.empty()) {
            continue;
        }
        char* end = nullptr;
        const double value = std::strtod(item.c_str(), &end);
        if (end != item.c_str() && std::isfinite(value)) {
            values.push_back(value);
        }
    }
    return values;
}

}  // namespace rbf::detail
