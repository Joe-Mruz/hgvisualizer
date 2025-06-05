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
#include "layout_cuda.h"
#include <iostream>

// Select the layout algorithm to use - uncomment one of the following lines
#define LAYOUT_CUDA
//#define LAYOUT_BH
//#define LAYOUT_SIMPLE

// Perform a stable layout by running the layout algorithm multiple times
// to allow the system to converge to a stable state.
// This method resets the velocities of the vertices to avoid building up oscillations.
void layout::SpringElectricalEmbedding::do_layout(const allocator::GenerationalIndexArray<model::Edge>& edges) {
    for (size_t i = 0; i < iterations_; ++i) {
#if defined(LAYOUT_CUDA)
        do_cuda_layout(edges);
#elif defined(LAYOUT_BH)
        do_bh_layout(edges);
#elif defined(LAYOUT_SIMPLE)
        do_simple_layout(edges);
#else
        std::cerr << "No layout algorithm defined. Please define one of LAYOUT_CUDA, LAYOUT_BH, or LAYOUT_SIMPLE." << std::endl;
        return;
#endif
    }

    // Reset to avoid building up oscillations
    for (auto& vertex : vertices_) {
        vertex.velocity_ = {0.f, 0.f};
    }
}

// Implement the spring-electrical embedding algorithm
// to adjust the positions of vertices based on repulsion and attraction forces.
// This is the naive O(n^2) implementation of the layout algorithm.
// Precondition: The NodeIds of the edges must match the IDs of the vertices.
void layout::SpringElectricalEmbedding::do_simple_layout(const allocator::GenerationalIndexArray<model::Edge>& edges) {
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

// Implement the Barnes-Hut approximation for the spring-electrical embedding layout algorithm.
// This method uses a quadtree to efficiently compute repulsion forces between vertices.
namespace {
using layout::Vec2;

// --- Quadtree for Barnes-Hut approximation ---
struct Quad {
    float x, y, w, h;
    std::vector<layout::Vertex*> points;
    std::unique_ptr<Quad> nw, ne, sw, se;
    bool divided = false;

    Quad(float x, float y, float w, float h) : x(x), y(y), w(w), h(h) {}

    bool contains(const layout::Vertex& v) const {
        return v.position_.x() >= x && v.position_.x() < x + w &&
               v.position_.y() >= y && v.position_.y() < y + h;
    }

    void subdivide() {
        float hw = w / 2, hh = h / 2;
        nw = std::make_unique<Quad>(x, y, hw, hh);
        ne = std::make_unique<Quad>(x + hw, y, hw, hh);
        sw = std::make_unique<Quad>(x, y + hh, hw, hh);
        se = std::make_unique<Quad>(x + hw, y + hh, hw, hh);
        divided = true;
    }

    void insert(layout::Vertex* v, int maxPoints = 1) {
        if (!contains(*v)) return;
        if (points.size() < maxPoints) {
            points.push_back(v);
        } else {
            if (!divided) subdivide();
            nw->insert(v, maxPoints);
            ne->insert(v, maxPoints);
            sw->insert(v, maxPoints);
            se->insert(v, maxPoints);
        }
    }

    // Compute total mass and center of mass for Barnes-Hut
    void computeMass(float& mass, Vec2& center) const {
        if (points.empty() && !divided) return;
        if (!divided) {
            mass = static_cast<float>(points.size());
            center = {0, 0};
            for (auto* v : points) center += v->position_;
            if (mass > 0) center /= mass;
        } else {
            float m = 0; Vec2 c{0, 0};
            float m1 = 0, m2 = 0, m3 = 0, m4 = 0;
            Vec2 c1, c2, c3, c4;
            nw->computeMass(m1, c1); ne->computeMass(m2, c2);
            sw->computeMass(m3, c3); se->computeMass(m4, c4);
            m = m1 + m2 + m3 + m4;
            c = (c1 * m1 + c2 * m2 + c3 * m3 + c4 * m4);
            if (m > 0) c /= m;
            mass = m; center = c;
        }
    }

    // Barnes-Hut repulsion force calculation
    void applyRepulsion(layout::Vertex& v, float theta, float repulsion_constant, Vec2& force) const {
        float mass = 0; Vec2 center{0, 0};
        computeMass(mass, center);
        if (mass == 0 || (points.size() == 1 && points[0]->id() == v.id())) return;
        Vec2 delta = v.position_ - center;

        // Note: When two nodes (or a node and a center of mass) are extremely close, 
        // the repulsive force (which is proportional to 1/(dist*dist)) becomes 
        // enormous, causing a node to "explode" away. Clamp the minimum distance to a reasonable value 
        // proportional to the graph's scale. Or we can add a softening factor.
        float dist = std::max(delta.length(), 20.0f); // Avoid division by zero or too small distances - tune as needed
        float s = w; // width of this quad
        if (!divided || (s / dist) < theta) {
            // Treat as a single mass
            force += delta.normalized() * (repulsion_constant * mass / (dist * dist));
        } else {
            nw->applyRepulsion(v, theta, repulsion_constant, force);
            ne->applyRepulsion(v, theta, repulsion_constant, force);
            sw->applyRepulsion(v, theta, repulsion_constant, force);
            se->applyRepulsion(v, theta, repulsion_constant, force);
        }
    }
};
}

// Implement the Barnes-Hut layout algorithm for the spring-electrical embedding.
// This method uses a quadtree to efficiently compute repulsion forces between vertices.
// Precondition: The NodeIds of the edges must match the IDs of the vertices.
void layout::SpringElectricalEmbedding::do_bh_layout(const allocator::GenerationalIndexArray<model::Edge>& edges) {
    // 1. Build quadtree
    float minX = vertices_[0].position_.x(), maxX = minX;
    float minY = vertices_[0].position_.y(), maxY = minY;
    for (const auto& v : vertices_) {
        minX = std::min(minX, v.position_.x());
        maxX = std::max(maxX, v.position_.x());
        minY = std::min(minY, v.position_.y());
        maxY = std::max(maxY, v.position_.y());
    }
    float w = std::max(maxX - minX, maxY - minY) + 1.0f;
    Quad root(minX - 0.5f, minY - 0.5f, w, w);
    for (auto& v : vertices_) root.insert(&v);

    // 2. Compute forces
    const float theta = .7f; // Barnes-Hut threshold
    for (auto& v : vertices_) {
        Vec2 force{0, 0};

        // Repulsion (Barnes-Hut)
        root.applyRepulsion(v, theta, repulsion_constant_, force);

        // Attraction (springs)
        for (const auto& edge : edges) {
            if (edge.from() == v.id() || edge.to() == v.id()) {
                Vertex& other = (edge.from() == v.id()) ? vertices_[edge.to()] : vertices_[edge.from()];
                Vec2 delta = other.position_ - v.position_;
                float dist = std::max(delta.length(), 1.0f); // Clamp to minimum distance
                force += delta.normalized() * (dist - spring_length_) * attraction_constant_;
            }
        }

        v.velocity_ += (force * timestep_) * damping_;
        v.position_ += v.velocity_ * timestep_;
    }
}

// Perform the layout using CUDA for better performance on large graphs.
// It uses a custom CUDA kernel to compute forces based on the positions of vertices and edges.
// Precondition: The NodeIds of the edges must match the IDs of the vertices.
void layout::SpringElectricalEmbedding::do_cuda_layout(const allocator::GenerationalIndexArray<model::Edge>& edges) {
    using namespace layout_cuda;

    int n = static_cast<int>(vertices_.size());
    int e = static_cast<int>(edges.size());

    // Prepare positions array
    std::vector<CudaVec2> h_positions(n);
    for (int i = 0; i < n; ++i) {
        h_positions[i].x = vertices_[i].position_.x();
        h_positions[i].y = vertices_[i].position_.y();
    }

    // Prepare edges array
    std::vector<CudaEdge> h_edges(e);
    int edge_idx = 0;
    for (const auto& edge : edges) {
        h_edges[edge_idx].from = edge.from();
        h_edges[edge_idx].to = edge.to();
        ++edge_idx;
    }

    // Output forces array
    std::vector<CudaVec2> h_forces(n);

    // Call CUDA wrapper to compute forces
    layout_cuda::compute_forces(
        h_positions.data(),
        h_forces.data(),
        n,
        repulsion_constant_,
        h_edges.data(),
        e,
        spring_length_,
        attraction_constant_
    );

    // Update vertex velocities and positions
    for (int i = 0; i < n; ++i) {
        Vec2 force{h_forces[i].x, h_forces[i].y};
        vertices_[i].velocity_ += (force * timestep_) * damping_;
        vertices_[i].position_ += vertices_[i].velocity_ * timestep_;
    }
}