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
#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <algorithm>
#include <array>
#include "allocator.h"

// This module defines the model for a graph structure, including nodes and edges.
// It provides functionality for adding, removing, and updating nodes and edges,
// as well as applying rules for edge creation and retention based on the number of edges connected to nodes.
namespace model {

using NodeId = size_t;
using NodeIndex = allocator::GenerationalIndex;
using EdgeIndex = allocator::GenerationalIndex;
using ModelTime = size_t;

class Edge {
public:
   Edge (NodeId from, NodeId to) : from_(from), to_(to) {}

   NodeId from() const { return from_; }
   NodeId to() const { return to_; }
   NodeId other(NodeId node_id) const {return (node_id == from_) ? to_ : from_;}
   bool connects(NodeId node_id) const { return (node_id == from_ || node_id == to_); }
   bool operator==(const Edge& other) const {
      return (from_ == other.from_ && to_ == other.to_) || (from_ == other.to_ && to_ == other.from_);
   }
   bool operator!=(const Edge& other) const {
      return !(*this == other);
   }
private:
   NodeId from_;
   NodeId to_;
};

class Node {
public:
   Node() = default;
   Node(NodeId id, NodeId parent_id = 0, ModelTime creation_time = 0) : id_(id), parent_id_(parent_id), creation_time_(creation_time) {}

   NodeId id() const { return id_; }
   ModelTime creation_time() const { return creation_time_; }
   NodeId parent_id() const { return parent_id_; }

private:
   NodeId id_;
   NodeId parent_id_ = 0; // Node ID that this node was created from, if any
   ModelTime creation_time_ = 0; // Time when this node was created
};

enum class FilterType {
   Edge,
};

enum class FilterCompareType {
   Equal,
   NotEqual,
   LessThan,
   GreaterThan,
   LessThanOrEqual,
   GreaterThanOrEqual,
};

enum class OperationType {
   Expand,
   Merge,
   Split,
   Integrate,
   Decay,
   ExpandAll,
};

using NodeArray = allocator::GenerationalIndexArray<Node>;
using EdgeArray = allocator::GenerationalIndexArray<Edge>;

// Represents an operation to be performed on a node
// based on the result of a rule.
class Operation {
public:
   Operation(OperationType op, size_t value=0) : op_(op), value_(value) {}

   OperationType op() const { return op_; }
   size_t value() const { return value_; }

private:
   OperationType op_;
   size_t value_;
};

// Represents a filter to be applied to a node
// to determine if a rule should be applied.
class Filter {
public:
   Filter(FilterType filter, FilterCompareType compare, size_t value) : filter_(filter), compare_(compare), value_(value) {}

   FilterType filter() const { return filter_; }
   FilterCompareType compare() const { return compare_; }
   size_t value() const { return value_; }

private:
   FilterType filter_;
   FilterCompareType compare_;
   size_t value_;
};

// Represents the result of a rule match.
// This includes whether the match was successful
// and any edges that were matched.
constexpr size_t MAX_MATCHED_EDGES = 100;
class MatchResult {
public:
   MatchResult() = default;
   void fail() { success_ = false; }
   void success() { success_ = true; }
   bool is_success() const { return success_; }
   const std::array<EdgeIndex, MAX_MATCHED_EDGES>& edge_indexes() const { return edge_indexes_; }
   size_t edge_count() const { return edge_count_; }

   void add_edge_index(EdgeIndex edge_index) {
      if (edge_count_ < MAX_MATCHED_EDGES) {
         edge_indexes_[edge_count_++] = edge_index;
      }
   }
private:
   bool success_ = false;
   std::array<EdgeIndex, MAX_MATCHED_EDGES> edge_indexes_;
   size_t edge_count_ = 0;
};

// Represents a rule that can be applied to a node.
// A rule consists of a filter and an operation.
// The filter is used to determine if the rule should be applied,
// and the operation is the action to be performed if the rule matches.
class Rule {
public:
   Rule(Filter lhs, Operation rhs) : lhs_(lhs), rhs_(rhs) {}

   Filter lhs() const { return lhs_; }
   Operation rhs() const { return rhs_; }
   MatchResult matches(const NodeId node_id, const NodeArray& nodes, const EdgeArray& edges) const;

private:
   Filter lhs_;
   Operation rhs_;
};

// Represents the model of a graph structure.
// This includes nodes, edges, and rules for modifying the graph.
class Model {
public:
   Model(std::vector<Rule> rules, EdgeArray edges, NodeArray nodes)
      : rules_(std::move(rules)), edges_(std::move(edges)), nodes_(std::move(nodes)) {
         add_node(0); // Start with a single node
   }

   const std::vector<Rule>& rules() const { return rules_; }
   const EdgeArray& edges() const { return edges_; }
   const NodeArray& nodes() const { return nodes_; }
   size_t node_count() const { return nodes_.size(); }
   size_t edge_count() const { return edges_.size(); }
   ModelTime time() const { return time_; }
   bool terminal_state() const { return terminal_state_; }
   float rule_miss_percentage() const { return rule_miss_percentage_; }

   bool contains_edge(Edge edge) const {
      for (const auto& entry : edges_) {
         if (entry.value() == edge) {
            return true;
         }
      }
      return false;
   }

   void update();
   void reset();
private:
   Node add_node(NodeId parent_id) {
      // We want to ensure the node ID is equivalent to its index in the array. 
      // This allows for certain optimizations with edge lookups.
      auto idx = nodes_.add(Node{});
      auto node = Node{idx.index(), parent_id, time_};
      nodes_.set(idx, node);
      return node;
   }

   bool apply_rule(Rule rule, NodeId this_node_id);
   bool apply_rule_expand(Rule rule, NodeId this_node_id, MatchResult match_result);
   bool apply_rule_collapse(Rule rule, NodeId this_node_id, MatchResult match_result, bool retain_original_edges = false);
   bool apply_rule_integrate(Rule rule, NodeId this_node_id, MatchResult match_result);
   bool apply_rule_decay(Rule rule, NodeId this_node_id, MatchResult match_result);
   bool apply_rule_expand_all(Rule rule, NodeId this_node_id, MatchResult match_result);
   bool apply_rule_merge(Rule rule, NodeId this_node_id, MatchResult match_result);
   bool apply_rule_split(Rule rule, NodeId this_node_id, MatchResult match_result);

   std::vector<Rule> rules_;
   EdgeArray edges_;
   NodeArray nodes_;
   ModelTime time_ = 1; // Start at time 1 so the initial node has a creation time of 1
   bool terminal_state_ = false; // True if no rules can be applied
   float rule_miss_percentage_ = 0.0f; // Percentage of nodes that did not have any rules applied
};

}

#endif
