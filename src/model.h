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

#include <cstdint>
#include <vector>
#include <algorithm>
#include "allocator.h"

// This module defines the model for a graph structure, including nodes and edges.
// It provides functionality for adding, removing, and updating nodes and edges,
// as well as applying rules for edge creation and retention based on the number of edges connected to nodes.
namespace model {

using NodeId = size_t;
using EdgeId = allocator::GenerationalIndex;

// The Node class represents a node (vertex) in the graph.
class Node {
public:
   Node (NodeId id, allocator::GenerationalIndexArray<EdgeId> edges = {}) : id_(id), edges_(std::move(edges)){}
   
   NodeId id() const { return id_; }
   bool remove_edge(EdgeId edge_id) {
      auto it = std::find(edges_.begin(), edges_.end(), edge_id);
      if (it != edges_.end()) {
         it.erase();
         return true;
      }
      return false;
   }

   allocator::GenerationalIndexArray<EdgeId> edges_;
private:
   NodeId id_;
};

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

// The Model class represents the entire graph structure, containing nodes and edges.
class Model {
public:
   Model (std::vector<Node> nodes,
          int node_max_edges = 20,
          int node_min_edges = 4, 
          int node_min_edges_decay = 1000,
          int node_min_edges_floor = 4) 
            : nodes_(std::move(nodes)),
              node_max_edges_(node_max_edges),
              node_min_edges_(node_min_edges),
              node_min_edges_decay_(node_min_edges_decay),
              node_min_edges_floor_(node_min_edges_floor) {};
   
   void update ();
   int node_max_edges() const { return node_max_edges_; }
   int node_min_edges() const { return node_min_edges_; }
   int node_min_edges_decay() const { return node_min_edges_decay_; }
   int node_min_edges_floor() const { return node_min_edges_floor_; }
   size_t node_count() const { return nodes_.size(); }
   size_t edge_count() const { return edges_.size(); }

   std::vector<Node> nodes_;
   allocator::GenerationalIndexArray<Edge> edges_;
private:
   bool apply_rule_one(Node& node);
   bool apply_rule_two(Node& node);

   bool nodes_connected(NodeId node_id1, NodeId node_id2) const {
      for (const auto& edge_id : nodes_[node_id1].edges_) {
         auto edge = edges_.get(edge_id);
         if (edge && edge->connects(node_id2)) {
            return true;
         }
      }
      return false;
   }

   int node_max_edges_; // Threshold for edge creation - maximum number of edges a node can have
   int node_min_edges_; // Threshold for edge retention - how many edges a node must have at minimum
   int node_min_edges_decay_; // Decay the retainment threshold after n updates
   int node_min_edges_floor_; // Minimum edge retainment threshold - decay stops when the threshold reaches this value
   int update_count_ {0}; // Count of updates to apply decay
};

}

#endif
