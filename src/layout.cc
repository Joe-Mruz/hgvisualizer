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
#include "layout.h"
#include "model.h"
#include "allocator.h"
#include <iostream>

// Implement the spring-electrical embedding algorithm
// to adjust the positions of vertices based on repulsion and attraction forces.
// Precondition: The NodeIds of the edges must match the IDs of the vertices.
void layout::SpringElectricalEmbedding::do_layout(const allocator::GenerationalIndexArray<model::Edge>& edges) {
    for (auto& v : vertices_) {
        Vec2 force{0, 0};
        
        // Repulsion
        for (const auto& u : vertices_) {
            if (u.id() == v.id()) continue;
            Vec2 delta = v.position_ - u.position_;
            float dist = std::max(delta.length(), 0.01f);
            force += delta.normalized() * (repulsion_constant_ / (dist * dist));
        }
        
        // Attraction (springs)
        for (const auto& edge : edges) {
            if (edge.from() == v.id() || edge.to() == v.id()) {
                Vertex& other = (edge.from() == v.id()) ? vertices_[edge.to()] : vertices_[edge.from()];
                Vec2 delta = other.position_ - v.position_;
                float dist = delta.length();
                force += delta.normalized() * (dist - spring_length_) * attraction_constant_;
            }
        }

        v.velocity_ += (force * timestep_) * damping_;
        v.position_ += v.velocity_ * timestep_;
    }
}

// Perform a stable layout by running the layout algorithm multiple times
// to allow the system to converge to a stable state.
// This method resets the velocities of the vertices to avoid building up oscillations.
void layout::SpringElectricalEmbedding::do_stable_layout(const allocator::GenerationalIndexArray<model::Edge>& edges) {
    for (size_t i = 0; i < iterations_; ++i) {
        do_layout(edges);
    }

    // Reset to avoid building up oscillations
    for (auto& vertex : vertices_) {
        vertex.velocity_ = {0.f, 0.f};
    }
}