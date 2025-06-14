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
#include "layout_cuda.h"

using namespace layout_cuda;

namespace {
// CUDA kernel for O(n^2) force-directed layout (repulsion + attraction)
__global__ void kernel_compute_forces(
    const CudaVec2* positions,
    CudaVec2* forces,
    int n,
    float repulsion_constant,
    const CudaEdge* edges,
    int edge_count,
    float spring_length,
    float attraction_constant) {

    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    CudaVec2 force(0, 0);
    CudaVec2 vi = positions[i];

    // Repulsion
    for (int j = 0; j < n; ++j) {
        if (i == j) continue;
        CudaVec2 vj = positions[j];
        CudaVec2 delta = vi - vj;

        // Note: When two nodes (or a node and a center of mass) are extremely close, 
        // the repulsive force (which is proportional to 1/(dist*dist)) becomes 
        // enormous, causing a node to "explode" away. Clamp the minimum distance to 
        // a reasonable value proportional to the graph's scale. Or we can add a softening factor.
        float dist = fmaxf(delta.length(), 20.0f); // Clamp to minimum distance -- tune as needed
        force += delta.normalized() * (repulsion_constant / (dist * dist));
    }

    // Attraction (springs)
    for (int e = 0; e < edge_count; ++e) {
        if (edges[e].from == i || edges[e].to == i) {
            int other = (edges[e].from == i) ? edges[e].to : edges[e].from;
            CudaVec2 vj = positions[other];
            CudaVec2 delta = vj - vi;
            float dist = fmaxf(delta.length(), 1.0f); // Clamp to minimum distance
            force += delta.normalized() * (dist - spring_length) * attraction_constant;
        }
    }

    forces[i] = force;

    // Cap the maximum force to prevent exploding nodes
    float max_force = 1000.0f; // Tune as needed
    float force_len = force.length();
    if (force_len > max_force) {
        forces[i] = force.normalized() * max_force;
    } else {
        forces[i] = force;
    }
}
}

// Host-side wrapper
void layout_cuda::compute_forces(
    const CudaVec2* h_positions,
    CudaVec2* h_forces,
    int n,
    float repulsion_constant,
    const CudaEdge* h_edges,
    int edge_count,
    float spring_length,
    float attraction_constant) {
        
    CudaVec2* d_positions = nullptr;
    CudaVec2* d_forces = nullptr;
    CudaEdge* d_edges = nullptr;
    cudaMalloc(&d_positions, n * sizeof(CudaVec2));
    cudaMalloc(&d_forces, n * sizeof(CudaVec2));
    cudaMalloc(&d_edges, edge_count * sizeof(CudaEdge));
    cudaMemcpy(d_positions, h_positions, n * sizeof(CudaVec2), cudaMemcpyHostToDevice);
    cudaMemcpy(d_edges, h_edges, edge_count * sizeof(CudaEdge), cudaMemcpyHostToDevice);

    int blockSize = 256;
    int gridSize = (n + blockSize - 1) / blockSize;
    kernel_compute_forces<<<gridSize, blockSize>>>(
        d_positions, d_forces, n, repulsion_constant,
        d_edges, edge_count, spring_length, attraction_constant
    );

    cudaMemcpy(h_forces, d_forces, n * sizeof(CudaVec2), cudaMemcpyDeviceToHost);
    cudaFree(d_positions);
    cudaFree(d_forces);
    cudaFree(d_edges);
}