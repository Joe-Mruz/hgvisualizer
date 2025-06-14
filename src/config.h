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
#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <unordered_map>
#include <optional>
#include "model.h"

// This module provides the capability to load configuration properties from a file.
namespace config {

// The Properties class is responsible for loading properties from a file and providing access to them.
class Properties {
public:
    bool load(const std::string& filename);

    std::optional<size_t> get_size_t(const std::string& key) const;
    std::optional<float> get_float(const std::string& key) const;
    std::optional<int> get_int(const std::string& key) const;
    std::optional<std::string> get_string(const std::string& key) const;

private:
    std::unordered_map<std::string, std::string> properties_;
};

// The Config class provides access to the specific configuration properties needed for the application.
// It is initialized with a Properties object, which contains the loaded properties.
class Config {
public:
    Config(Properties properties) : properties_(properties) {}

    int     model_updates_per_frame()        const {return properties_.get_int("model.updates_per_frame").value_or(10);}
    float   layout_spring_length()           const {return properties_.get_float("layout.spring_length").value_or(5.0f);}
    float   layout_repulsion_constant()      const {return properties_.get_float("layout.repulsion_constant").value_or(10000.0f);}
    float   layout_attraction_constant()     const {return properties_.get_float("layout.attraction_constant").value_or(0.5f);}
    float   layout_timestep()                const {return properties_.get_float("layout.timestep").value_or(1.0f);}
    float   layout_damping_factor()          const {return properties_.get_float("layout.damping_factor").value_or(0.005f);}
    int     layout_stablization_iterations() const {return properties_.get_int("layout.stabilization_iterations").value_or(5);}
private:
    const Properties properties_;
};

// The Rules class is responsible for loading and storing the rules used in the model.
// It provides a method to load rules from a file and a method to retrieve the loaded rules.
class Rules {
public:
    void load(const std::string& filename);
    std::vector<model::Rule> rules() const { return rules_; }
private:
    std::vector<model::Rule> rules_;

};

}

#endif