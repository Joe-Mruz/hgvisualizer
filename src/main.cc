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
#include "config.h"

int main() {
    config::Properties properties {};
    properties.load("res/app.config");
    config::Config config(std::move(properties));

    const size_t NODE_COUNT = config.model_node_count(); // Number of nodes in the graph

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
            layout::Vec2 {
                static_cast<float>(col * config.layout_spring_length() * 2), 
                static_cast<float>(row * config.layout_spring_length() * 2)
            }}); // Initial positions
    }

    auto hgraph_layout = std::make_unique<layout::SpringElectricalEmbedding>(
        std::move(vertices),
        config.layout_repulsion_constant(),
        config.layout_attraction_constant(),
        config.layout_timestep(),
        config.layout_spring_length(),
        config.layout_damping_factor(),         // damping factor - .01 is a good value for the O(n^2) layout, .1 is good for the Barnes-Hut layout
        config.layout_stablization_iterations() // iterations for stable layout - 5 is a good value for the O(n^2) layout, 1 is good for the Barnes-Hut layout
    );

    auto hgraph_model = std::make_unique<model::Model>(
        nodes,
        config.model_node_max_edges(),
        config.model_node_min_edges(),
        config.model_node_min_edges_decay(), // reduces the node min edges after every n updates
        config.model_node_min_edges_floor()  // decay stops when the threshold reaches this value
    );

    // Select the initial positionings for the nodes and edges in the graph.
    // Uncomment one of the following to select the initial model for the graph.
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

    ui::run(
        hgraph_model, 
        hgraph_layout, 
        config.model_updates_per_frame(), // higher values run faster, but rendering is less granular and the layout processing may lag behind
        config.layout_iterations_per_frame()); // higher values allow faster stable layout convergence, but is expensive and may cause stuttered rendering

    return 0;
}