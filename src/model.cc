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
#include <unordered_set>

namespace model {

static thread_local std::mt19937 rng{std::random_device{}()};

MatchResult Rule::matches(const NodeId node_id, const NodeArray& nodes, const EdgeArray& edges) const {
    MatchResult result {};
    
    // Find all edges connected to the node and store their ID in the result
    std::array<NodeId, MAX_MATCHED_EDGES> other_nodes {};
    size_t other_node_count = 0;
    size_t matched_edges = 0;
    for (const auto& edge_entry : edges) {
        Edge edge = edge_entry.value();

        if (edge.connects(node_id)) {
            result.add_edge_index(edge_entry.index());
            ++matched_edges;

            if (other_node_count >= MAX_MATCHED_EDGES) {
                break;
            }

            other_nodes[other_node_count++] = edge.other(node_id);
        }
    }

    switch (lhs_.compare()) {
        case FilterCompareType::Equal: matched_edges == lhs_.value() ? result.success() : result.fail(); break;
        case FilterCompareType::NotEqual: matched_edges != lhs_.value() ? result.success() : result.fail(); break;
        case FilterCompareType::LessThan: matched_edges < lhs_.value() ? result.success() : result.fail(); break;
        case FilterCompareType::GreaterThan: matched_edges > lhs_.value() ? result.success() : result.fail(); break;
        case FilterCompareType::LessThanOrEqual: matched_edges <= lhs_.value() ? result.success() : result.fail(); break;
        case FilterCompareType::GreaterThanOrEqual: matched_edges >= lhs_.value() ? result.success() : result.fail(); break;
        default: 
            // Unsupported comparison type, fail the match
            result.fail();
    }

    return result;
}

void Model::update() {
    if (terminal_state_) {
        // No more rules can be applied, do nothing
        return;
    }

    this->time_++;

    // Shuffle the nodes and attempt to apply rules in random order. Stop when one of the rules is successfully applied.
    std::vector<NodeId> shuffled_nodes;
    for (const auto& node : nodes_) {
        shuffled_nodes.push_back(node.value().id());
    }
    std::shuffle(shuffled_nodes.begin(), shuffled_nodes.end(), rng);

    std::vector<Rule> shuffled_rules(rules_);
    std::shuffle(shuffled_rules.begin(), shuffled_rules.end(), rng);

    bool rule_applied = false;
    size_t rule_misses = 0;

    for (const auto& node : shuffled_nodes) {
        // Process each rule for the current node
        for (const auto& rule : shuffled_rules) {
            if (apply_rule(rule, node)) {
                // If the rule was successfully applied, break out of the loop
                rule_applied = true;
                break;
            }
        }

        if (rule_applied) {
            break;
        } else {
            rule_misses++;
        }
    }

    // If no rules have been applied, this is a terminal state, give up
    if (!rule_applied) {
        terminal_state_ = true;
    }

    // Calculate the percentage of nodes that did not have any rules applied
    if (shuffled_nodes.size() == 0) {
        rule_miss_percentage_ = 0;
    } else {
        rule_miss_percentage_ = static_cast<float>(rule_misses) / static_cast<float>(shuffled_nodes.size()) * 100.0f;
    }
}

void Model::reset() {
    // Clear all nodes and edges, reset time and terminal state
    edges_.clear();
    nodes_.clear();
    time_ = 1;
    terminal_state_ = false;

    // Add a single initial node
    add_node(0);
}

bool Model::apply_rule(Rule rule, NodeId this_node_id) {

    const MatchResult match_result = rule.matches(this_node_id, nodes_, edges_);

    if (!match_result.is_success()) {
        // The rule does not match the current node, skip it
        return false;
    }

    // Apply the rule to the model
    bool rule_applied = false;
    switch (rule.rhs().op()) {
        case OperationType::Expand: rule_applied = apply_rule_expand(rule, this_node_id, match_result); break;
        case OperationType::Merge: rule_applied = apply_rule_merge(rule, this_node_id, match_result); break;
        case OperationType::Split: rule_applied = apply_rule_split(rule, this_node_id, match_result); break;
        case OperationType::Integrate: rule_applied = apply_rule_integrate(rule, this_node_id, match_result); break;
        case OperationType::Decay: rule_applied = apply_rule_decay(rule, this_node_id, match_result); break;
        case OperationType::ExpandAll: rule_applied = apply_rule_expand_all(rule, this_node_id, match_result); break;
        default: rule_applied = false; // Unsupported operation type
    }

    return rule_applied;
}

bool Model::apply_rule_expand(Rule rule, NodeId this_node_id, MatchResult match_result) {
    // For the specified value of the RHS, split an edge into two edges, creating a new node in the middle.
    // If there are not enough edges to split, create new nodes and connect them to this_node_id.
    size_t splits = rule.rhs().value();
    size_t performed = 0;

    for (size_t i = 0; i < match_result.edge_count() && performed < splits; ++i) {
        EdgeIndex edge_idx = match_result.edge_indexes()[i];
        Edge edge = edges_.get(edge_idx).value();

        // Remove the original edge
        edges_.remove(edge_idx);

        // Create a new node in the middle
        Node new_node = add_node(this_node_id);

        // Add two new edges: (edge.from(), new_node) and (new_node, edge.to())
        edges_.add(Edge{edge.from(), new_node.id()});
        edges_.add(Edge{new_node.id(), edge.to()});

        ++performed;
    }

    // If performed < splits, create new nodes and connect them to this_node_id
    while (performed < splits) {
        Node new_node = add_node(this_node_id);
        edges_.add(Edge{this_node_id, new_node.id()});
        ++performed;
    }

    return performed > 0;
}

bool Model::apply_rule_expand_all(Rule rule, NodeId this_node_id, MatchResult match_result) {
    // For the specified value of the RHS, expand all edges connected to this_node_id.
    return apply_rule_expand(Rule {rule.lhs(), Operation {OperationType::Expand, match_result.edge_count()}}, this_node_id, match_result);
}

bool Model::apply_rule_merge(Rule rule, NodeId this_node_id, MatchResult match_result) {
    // For the specified value of the RHS, merge up to that many neighbor nodes into this_node_id.
    // All edges of merged nodes are redirected to this_node_id, and the merged nodes are removed.
    size_t merges = rule.rhs().value();
    size_t merged = 0;

    // Collect up to 'merges' unique neighbor node IDs from match_result
    std::array<NodeId, MAX_MATCHED_EDGES> neighbor_ids{};
    size_t neighbor_count = 0;
    for (size_t i = 0; i < match_result.edge_count() && neighbor_count < merges && neighbor_count < MAX_MATCHED_EDGES; ++i) {
        Edge edge = edges_.get(match_result.edge_indexes()[i]).value();
        NodeId other = edge.other(this_node_id);
        // Avoid duplicates and self
        bool already_added = (other == this_node_id);
        for (size_t j = 0; j < neighbor_count; ++j) {
            if (neighbor_ids[j] == other) {
                already_added = true;
                break;
            }
        }
        if (!already_added) {
            neighbor_ids[neighbor_count++] = other;
        }
    }

    std::vector<Edge> edges_to_add;

    // For each neighbor to merge
    for (size_t i = 0; i < neighbor_count; ++i) {
        NodeId merge_id = neighbor_ids[i];

        for (const auto& edge_entry : edges_) {
            const Edge& edge = edge_entry.value();
            if (edge.connects(merge_id) && edge.other(merge_id) != this_node_id) {
                Edge new_edge{this_node_id, edge.other(merge_id)};
                edges_to_add.push_back(new_edge);
            }
        }

        // Remove all edges connected to merge_id
        for (auto it = edges_.begin(); it != edges_.end();) {
            Edge edge = (*it).value();
            if (edge.connects(merge_id)) {
                it.erase();
            } else {
                ++it;
            }
        }

        // Remove the merged node
        // Note: this only works because the node ID is the same as its index
        allocator::GenerationalIndex merge_idx = nodes_.begin_all()[merge_id].index();
        nodes_.remove(merge_idx);

        ++merged;
    }

    // Now add the new edges
    for (const Edge& e : edges_to_add) {
        // Avoid adding duplicate edges and dangling edges.
        // We may end up with duplicates or dangling edges if there is more than one merged node and those nodes are connected to each other. 
        bool dangling = nodes_.begin_all()[e.from()].index().alive() == false || nodes_.begin_all()[e.to()].index().alive() == false;
        if (!contains_edge(e) && !dangling) {
            edges_.add(e);
        }
    }

    return merged > 0;
}

bool Model::apply_rule_split(Rule rule, NodeId this_node_id, MatchResult match_result) {
    // For the specified value of the RHS, clone this_node_id into that many new nodes.
    // All edges of this_node_id are duplicated for each new node.
    size_t splits = rule.rhs().value();
    if (splits == 0) return false;

    // Collect all edges connected to this_node_id
    std::vector<Edge> connected_edges;
    for (const auto& edge_entry : edges_) {
        Edge edge = edge_entry.value();
        if (edge.connects(this_node_id)) {
            connected_edges.push_back(edge);
        }
    }

    bool any_split = false;
    std::vector<NodeId> new_node_ids;
    for (size_t i = 0; i < splits; ++i) {
        // Create a new node (clone)
        Node new_node = add_node(this_node_id);
        new_node_ids.push_back(new_node.id());

        // Duplicate all edges for the new node
        for (const Edge& edge : connected_edges) {
            if (edge.from() == this_node_id) {
                edges_.add(Edge{new_node.id(), edge.to()});
            } else if (edge.to() == this_node_id) {
                edges_.add(Edge{edge.from(), new_node.id()});
            }
        }

        any_split = true;
    }

    for (NodeId new_id : new_node_ids) {
        // Connect the new nodes to this_node_id
        edges_.add(Edge{this_node_id, new_id});
    }

    return any_split;
}

bool Model::apply_rule_integrate(Rule rule, NodeId this_node_id, MatchResult match_result) {
    // For the specified value of the RHS, create an edge to a second tier neighbor.
    size_t new_edge_count = rule.rhs().value();
    size_t performed = 0;

    // 1. Use other nodes in the match_result edges as the "first-tier" set
    std::array<NodeId, MAX_MATCHED_EDGES> first_tier_neighbors{};
    size_t first_tier_count = 0;
    for (size_t i = 0; i < match_result.edge_count(); ++i) {
        Edge edge = edges_.get(match_result.edge_indexes()[i]).value();
        NodeId other = edge.other(this_node_id);
        if (first_tier_count < MAX_MATCHED_EDGES) {
            first_tier_neighbors[first_tier_count++] = other;
        }
    }

    // 2. Find unique second-tier neighbors (neighbors of neighbors, not in first-tier or self)
    using EdgePair = std::pair<EdgeIndex, NodeId>;
    std::array<EdgePair, MAX_MATCHED_EDGES> second_tier_edges{};
    size_t second_tier_count = 0;
    for (size_t n = 0; n < first_tier_count; ++n) {
        NodeId neighbor = first_tier_neighbors[n];
        for (const auto& edge_entry : edges_) {
            const Edge& edge = edge_entry.value();
            if (edge.connects(neighbor)) {
                NodeId other = edge.other(neighbor);
                // Check not self and not in first_tier_neighbors
                bool is_first_tier = false;
                for (size_t k = 0; k < first_tier_count; ++k) {
                    if (first_tier_neighbors[k] == other) {
                        is_first_tier = true;
                        break;
                    }
                }
                // Check not already in second_tier_edges
                bool already_added = false;
                for (size_t k = 0; k < second_tier_count; ++k) {
                    if (second_tier_edges[k].second == other) {
                        already_added = true;
                        break;
                    }
                }
                if (other != this_node_id && !is_first_tier && !already_added && second_tier_count < MAX_MATCHED_EDGES) {
                    second_tier_edges[second_tier_count++] = std::make_pair(edge_entry.index(), other);
                }

                if (second_tier_count >= MAX_MATCHED_EDGES) {
                    break; // Stop if we reached the maximum number of second-tier edges
                }
            }
        }
    }

    // 3. For up to new_edge_count, create an edge from this_node_id to the second-tier neighbor
    for (size_t i = 0; i < second_tier_count && performed < new_edge_count; ++i) {
        EdgeIndex edge_idx = second_tier_edges[i].first;
        NodeId second_tier_id = second_tier_edges[i].second;
        Edge new_edge{this_node_id, second_tier_id};
        if (!contains_edge(new_edge)) {
            edges_.add(new_edge);
        }

        ++performed;
    }

    return performed > 0;
}

bool Model::apply_rule_decay(Rule rule, NodeId this_node_id, MatchResult match_result) {
    // For the specified value of the RHS, remove up to that many edges connected to this_node_id.
    size_t decays = rule.rhs().value();
    size_t removed = 0;

    // Directly remove up to 'decays' edges from match_result
    for (size_t i = 0; i < match_result.edge_count() && removed < decays; ++i) {
        edges_.remove(match_result.edge_indexes()[i]);
        ++removed;
    }

    return removed > 0;
}

}