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
#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <cmath>
#include "allocator.h"
#include "model.h"

// This module provides a graph layout algorithm using a spring-electrical embedding method.
// The algorithm is based on the concept of simulating forces between nodes in a graph,
// where nodes repel each other like charged particles, and edges act like springs that attract connected nodes.
// The goal is to find a layout that minimizes the energy of the system, resulting in a visually appealing and well-distributed graph layout.
// This is based on the Wolfram Language's SpringElectricalEmbedding method documented here:
// https://reference.wolfram.com/language/ref/method/SpringElectricalEmbedding.html
namespace layout {
class Vec2 {
public:
    Vec2(float x = 0.0f, float y = 0.0f) : x_(x), y_(y) {}
    
    float x() const { return x_; }
    float y() const { return y_; }

    float length() const {
        return std::sqrt(x_ * x_ + y_ * y_);
    }

    Vec2 normalized() const {
        float length = this->length();
        if (length == 0.0f) return Vec2(0.0f, 0.0f);
        return Vec2(x_ / length, y_ / length);
    }

    Vec2 operator+(const Vec2& other) const {
        return Vec2(x_ + other.x_, y_ + other.y_);
    }
    Vec2 operator-(const Vec2& other) const {
        return Vec2(x_ - other.x_, y_ - other.y_);
    }
    Vec2 operator*(float scalar) const {
        return Vec2(x_ * scalar, y_ * scalar);
    }
    Vec2 operator/(float scalar) const {
        if (scalar == 0.0f) return Vec2(0.0f, 0.0f); // Avoid division by zero
        return Vec2(x_ / scalar, y_ / scalar);
    }
    Vec2 operator+=(const Vec2& other) {
        x_ += other.x_;
        y_ += other.y_;
        return *this;
    }
    Vec2 operator-=(const Vec2& other) {
        x_ -= other.x_;
        y_ -= other.y_;
        return *this;
    }
    Vec2 operator*=(float scalar) {
        x_ *= scalar;
        y_ *= scalar;
        return *this;
    }
    Vec2 operator/=(float scalar) {
        if (scalar == 0.0f) return Vec2(0.0f, 0.0f); // Avoid division by zero
        x_ /= scalar;
        y_ /= scalar;
        return *this;
    }

private:
    float x_;
    float y_;
};

class Vertex {
public:
    Vertex(model::NodeId id, Vec2 position, Vec2 velocity = {0.f, 0.f}) : id_(id), position_(position), velocity_(velocity) {}
    
    model::NodeId id() const { return id_; }
    
    Vec2 position_;
    Vec2 velocity_;
private:
    model::NodeId id_;
};

class SpringElectricalEmbedding {
public:
    SpringElectricalEmbedding(
            const std::vector<Vertex>& vertices, 
            float repulsion_constant = 1.0f, 
            float attraction_constant = 0.1f, 
            float timestep = 1.0f, 
            float spring_length = 10.0f,
            float damping = 0.9f,
            int iterations = 100)
        : vertices_(vertices), 
          repulsion_constant_(repulsion_constant), 
          attraction_constant_(attraction_constant), 
          timestep_(timestep), 
          spring_length_(spring_length), 
          damping_(damping), 
          iterations_(iterations) {}

    void do_layout(const allocator::GenerationalIndexArray<model::Edge>& edges); // Method to perform the layout algorithm
    void do_stable_layout(const allocator::GenerationalIndexArray<model::Edge>& edges); // Method to perform a stable layout
    const std::vector<Vertex>& vertices() const { return vertices_; } // Get the list of vertices
    float repulsion_constant() const { return repulsion_constant_; } // Get the repulsion constant
    float attraction_constant() const { return attraction_constant_; } // Get the attraction constant
    float timestep() const { return timestep_; } // Get the time step for the simulation
    float spring_length() const { return spring_length_; } // Get the desired length of the springs (edges)
    float damping() const { return damping_; } // Get the damping factor to reduce oscillations
    int iterations() const { return iterations_; } // Get the number of iterations for stable layout
private:
    std::vector<Vertex> vertices_; // List of vertices in the graph
    float repulsion_constant_; // Constant for repulsion force between nodes
    float attraction_constant_; // Constant for attraction force between connected nodes (spring constant)
    float timestep_; // Time step for the simulation
    float spring_length_; // Desired length of the springs (edges) between connected nodes
    float damping_; // Damping factor to reduce oscillations
    int iterations_; // Number of iterations for stable layout
};

}

#endif
