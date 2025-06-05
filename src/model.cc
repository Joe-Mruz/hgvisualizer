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
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include "model.h"
#include <random>

using namespace model;

static thread_local std::mt19937 rng{std::random_device{}()};

// Remove an edge if it there are at least n edges connected to the node
// Returns true if an edge was removed, false otherwise
bool Model::apply_rule_one(Node& node) {
    // Prevent removing too many edges that would isolate the node
    // TODO: Consider making this per node, or even per edge - what impacts would that have?
    if (node.edges_.size() <= node_min_edges_) {
        return false; // Not enough edges to remove
    }

    // Collect GenerationalIndex keys (edge indices)
    std::vector<allocator::GenerationalIndex> edge_indices;
    for (const auto& edge_idx : node.edges_) {
        edge_indices.push_back(edge_idx);
    }

    // Shuffle the indices
    std::shuffle(edge_indices.begin(), edge_indices.end(), rng);

    // Iterating over the natural order of edges introduces a bias that results in certain edges being removed more frequently than others.
    // To avoid this, we shuffle the edge indices before iterating.
    // This ensures that we randomly select edges to remove, which helps maintain a more balanced graph structure.
    for (auto& edge_id: edge_indices) {
        auto edge = edges_.get(edge_id);
        if (!edge) {
            throw; // Edge not found, this should not happen
        }
        
        // Check if the edge connects to another node with more than one edge
        auto other_node_id = edge->other(node.id());
        auto& other_node = nodes_[other_node_id];

        if (other_node.edges_.size() > node_min_edges_) {
            // If the other node has more than the minimum allowed edges, remove this edge
            // Note: This invalidates the iterator, so we need to break out after removing
            node.remove_edge(edge_id); 
            other_node.remove_edge(edge_id);
            edges_.remove(edge_id);
            return true; // Successfully removed an edge
        }
    }
    // No edge removed
    return false;
}

// Create an edge to a second tier neighbor
// Returns true if a new edge was created, false otherwise
bool Model::apply_rule_two(Node& node) {
    if (node.edges_.size() < 1 || node.edges_.size() >= node_max_edges_) {
        return false; // Not enough edges to create a new one
    }

    // Collect GenerationalIndex keys (edge indices)
    std::vector<allocator::GenerationalIndex> edge_indices;
    for (const auto& edge_idx : node.edges_) {
        edge_indices.push_back(edge_idx);
    }

    // Shuffle the indices
    std::shuffle(edge_indices.begin(), edge_indices.end(), rng);

    // Iterating over the natural order of edges introduces a bias that results in certain edges being removed more frequently than others.
    // To avoid this, we shuffle the edge indices before iterating.
    // This ensures that we randomly select edges to remove, which helps maintain a more balanced graph structure.
    for(auto& edge_id: edge_indices) {
        auto edge = edges_.get(edge_id);
        if (!edge) {
            throw; // Edge not found, this should not happen
        }
        
        // Check if the edge connects to another node with more than one edge
        auto other_node_id = edge->other(node.id());
        auto& other_node = nodes_[other_node_id];

        if (other_node.edges_.size() > 1) {

            std::vector<allocator::GenerationalIndex> other_node_edge_indices;
            for (const auto& edge_idx : other_node.edges_) {
                other_node_edge_indices.push_back(edge_idx);
            }

            // Shuffle the indices
            std::shuffle(other_node_edge_indices.begin(), other_node_edge_indices.end(), rng);

            // Find a second tier neighbor by iterating the edges of the other node
            // that does not connect to the current node and is not the current node itself
            for (auto& second_tier_edge_id: other_node_edge_indices) {
                auto second_tier_edge = edges_.get(second_tier_edge_id);
                if (!second_tier_edge) {
                    throw; // Edge not found, this should not happen
                }

                // Check if the second tier edge connects to a node that is not the current node
                if (second_tier_edge->connects(node.id())) {
                    continue;
                }

                NodeId second_tier_neighbor_id = second_tier_edge->other(other_node.id());
                
                // Check if the second tier neighbor is already connected to the current node
                // or if it is the current node itself.
                if (nodes_connected(node.id(), second_tier_neighbor_id) || second_tier_neighbor_id == node.id()) {
                    continue;
                }

                if (nodes_[second_tier_neighbor_id].edges_.size() >= node_max_edges_) {
                    continue; // The second tier neighbor already has too many edges
                }

                Edge new_edge {node.id(), second_tier_neighbor_id};
                EdgeId new_edge_id = edges_.add(new_edge);
                node.edges_.add(new_edge_id);
                nodes_[second_tier_neighbor_id].edges_.add(new_edge_id);
                return true; // Successfully created a new edge
            }
        }
    }
    // No second tier neighbor found or no new edge created - the local region is fully connected
    return false;
}

// Update the model by applying rules to a random node
void Model::update() {
    constexpr size_t MAX_ITERATIONS = 10; // Limit iterations to prevent infinite loops for the terminal states

    for (int i {0}; i < MAX_ITERATIONS; ++i) {
        // Randomly pick a node and try to apply rules
        std::uniform_int_distribution<int> nodes_dist(0, nodes_.size() - 1);
        size_t node_index = nodes_dist(rng);
        Node& node = nodes_[node_index];

        // Randomly select which rule to try first
        std::uniform_int_distribution<int> dist(0, 1);
        bool try_rule_one_first = dist(rng);

        bool rule_applied = false;
        if (try_rule_one_first) {
            rule_applied = apply_rule_one(node);
            if (!rule_applied) {
                rule_applied = apply_rule_two(node);
            }
        } else {
            rule_applied = apply_rule_two(node);
            if (!rule_applied) {
                rule_applied = apply_rule_one(node);
            }
        }

        // If a rule was applied, we can stop
        if (rule_applied) {
            break;
        }
    }

    // Increment the update count and apply decay to the edge retainment threshold
    if (node_min_edges_decay_ <= 0) {
        return; // No decay applied, exit early
    }
    update_count_++;
    if (update_count_ % node_min_edges_decay_ == 0) {
        node_min_edges_ = std::max(node_min_edges_floor_, node_min_edges_ - 1);
        update_count_ = 0;
    }
}
