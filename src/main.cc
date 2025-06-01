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
#include "model.h"
#include "layout.h"
#include "ui.h"

int main() {
    constexpr size_t NODE_COUNT = 256;
    constexpr float SPRING_LENGTH = 5.0f;
    constexpr int UPDATES_PER_FRAME = 10; // higher values run faster, but rendering is less granular and the layout processing may lag behind
    constexpr int LAYOUT_ITERATIONS_PER_FRAME = 1; // higher values allow faster stable layout convergence, but is expensive and may cause stuttered rendering

    std::vector<model::Node> nodes;
    for (size_t i = 0; i < NODE_COUNT; ++i) {
        nodes.emplace_back(model::Node{i});
    }

    std::vector<layout::Vertex> vertices;

    // Initialize vertices in a grid layout for better initial distribution
    int num_cols = std::sqrt(NODE_COUNT);
    for (size_t i = 0; i < NODE_COUNT; ++i) {
        int row = i / num_cols;
        int col = i % num_cols;
        vertices.emplace_back(layout::Vertex{
            nodes[i].id(), 
            layout::Vec2(static_cast<float>(col * SPRING_LENGTH), static_cast<float>(row * SPRING_LENGTH))}); // Initial positions
    }

    auto hgraph_layout = std::make_unique<layout::SpringElectricalEmbedding>(
        std::move(vertices),
        100000.0f, // repulsion constant
        0.1f, // attraction constant
        1.0f, // timestep
        SPRING_LENGTH, // spring length
        0.01f,  // damping factor
        10 // iterations for stable layout
    );

    auto hgraph_model = std::make_unique<model::Model>(
        nodes,
        15, // edge threshold for creating new edges
        5, // edge retainment threshold
        0, // edge retainment decay
        5 // edge retainment floor
    );

    #define HGRAPH_SIMPLE_MODEL
    //#define HGRAPH_FULLY_CONNECTED

    // Create a simple model with edges between nodes
    #ifdef HGRAPH_SIMPLE_MODEL
    for (size_t i = 0; i < NODE_COUNT - 1; ++i) {
        const auto edge = model::Edge{i, i + 1};
        const auto edge_id = hgraph_model->edges_.add(edge);

        hgraph_model->nodes_[i].edges_.add(edge_id);
        hgraph_model->nodes_[i + 1].edges_.add(edge_id); // Also add the reverse edge for undirected graph
    }
    // Connect the last node to the first to make it circular
    const auto edge = model::Edge{NODE_COUNT - 1, 0};
    const auto edge_id = hgraph_model->edges_.add(edge);
    hgraph_model->nodes_[NODE_COUNT - 1].edges_.add(edge_id);
    hgraph_model->nodes_[0].edges_.add(edge_id); // Also add the reverse edge for undirected graph
    #endif

    // Create a fully connected graph
    // Note: this can sometimes cause layout issues due to high contention with the initial positioning
    #ifdef HGRAPH_FULLY_CONNECTED
    for (size_t i = 0; i < NODE_COUNT; ++i) {
        for (size_t j = 0; j < NODE_COUNT; ++j) {
            if (i == j) continue; // Skip self-loops

            const auto edge = model::Edge{i, j};
            const auto edge_id = hgraph_model->edges_.add(edge);

            hgraph_model->nodes_[i].edges_.add(edge_id);
            hgraph_model->nodes_[j].edges_.add(edge_id); // Also add the reverse edge for undirected graph
        }
    }
    #endif

    ui::run(hgraph_model, hgraph_layout, UPDATES_PER_FRAME, LAYOUT_ITERATIONS_PER_FRAME);

    return 0;
}