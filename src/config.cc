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
#include <regex>
#include <iostream>

namespace {
std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start, end - start + 1);
}
}

namespace config {

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

void Rules::load(const std::string& filename) {
    using namespace model;

    std::ifstream infile(filename);

    if (!infile) {
        std::cout << "Failed to open rules file: " << filename << std::endl;
        return;
    }

    std::string line;
    // Regex: lhs op rhs -> op_name(value)
    // One rule per line
    // Example: edges <= 2 -> split(3)
    std::regex rule_regex(R"(^\s*(\w+)\s*(<=|>=|==|!=|<|>)\s*(\d+)\s*->\s*(\w+)\((\d+)\)\s*$)");
    std::smatch match;
    while (std::getline(infile, line)) {
        if (std::regex_match(line, match, rule_regex)) {
            std::string lhs = match[1];
            std::string op = match[2];
            int rhs = std::stoi(match[3]);
            std::string action = match[4];
            int value = std::stoi(match[5]);

            FilterType filter_type;
            if (lhs == "edges") {
                filter_type = FilterType::Edge;
            } else {
                std::cout << "Unknown filter type: " << lhs << std::endl;
                continue;
            }

            FilterCompareType compare_type;
            if (op == "<=") {
                compare_type = FilterCompareType::LessThanOrEqual;
            } else if (op == ">=") {
                compare_type = FilterCompareType::GreaterThanOrEqual;
            } else if (op == "==") {
                compare_type = FilterCompareType::Equal;
            } else if (op == "!=") {
                compare_type = FilterCompareType::NotEqual;
            } else if (op == "<") {
                compare_type = FilterCompareType::LessThan;
            } else if (op == ">") {
                compare_type = FilterCompareType::GreaterThan;
            } else {
                std::cout << "Unknown comparison operator: " << op << std::endl;
                continue;
            }

            OperationType operation_type;
            if (action == "split") {
                operation_type = OperationType::Split;
            } else if (action == "merge") {
                operation_type = OperationType::Merge;
            } else if (action == "expand") {
                operation_type = OperationType::Expand;
            } else if (action == "integrate") {
                operation_type = OperationType::Integrate;
            } else if (action == "decay") {
                operation_type = OperationType::Decay;
            } else if (action == "expand_all") {
                operation_type = OperationType::ExpandAll;
            } else {
                std::cout << "Unknown operation type: " << action << std::endl;
                continue;
            }

            Rule rule {
                Filter{filter_type, compare_type, static_cast<size_t>(rhs)},
                Operation{operation_type, static_cast<size_t>(value)}
            };

            rules_.emplace_back(std::move(rule));
        } else {
            std::cout << "Invalid rule format: " << line << std::endl;
        }
    }
}

}