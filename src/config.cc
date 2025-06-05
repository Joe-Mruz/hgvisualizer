/*
 * This file is part of HGVisualizer.
 *
 * Copyright (C) Joe Mruz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "config.h"
#include <fstream>
#include <sstream>
#include <cctype>

using namespace config;

namespace {
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}
}

bool Properties::load(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        auto pos = line.find('=');
        if (pos == std::string::npos) {
            continue;
        }

        std::string key = trim(line.substr(0, pos));
        std::string value = trim(line.substr(pos + 1));
        if (!key.empty()) {
            properties_[key] = value;
        }
    }
    return true;
}

std::optional<size_t> Properties::get_size_t(const std::string& key) const {
    auto it = properties_.find(key);
    if (it == properties_.end()) { 
        return std::nullopt;
    }

    try { 
        return static_cast<size_t>(std::stoull(it->second)); 
    } catch (...) { 
        return std::nullopt; 
    }
}

std::optional<float> Properties::get_float(const std::string& key) const {
    auto it = properties_.find(key);
    if (it == properties_.end()) {
        return std::nullopt;
    }

    try { 
        return std::stof(it->second); 
    } catch (...) { 
        return std::nullopt; 
    }
}

std::optional<int> Properties::get_int(const std::string& key) const {
    auto it = properties_.find(key);
    if (it == properties_.end()) {
        return std::nullopt;
    }
    
    try { 
        return std::stoi(it->second); 
    } catch (...) { 
        return std::nullopt; 
    }
}

std::optional<std::string> Properties::get_string(const std::string& key) const {
    auto it = properties_.find(key);
    if (it == properties_.end()) {
        return std::nullopt;
    }
    return it->second;
}