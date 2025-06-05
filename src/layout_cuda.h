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
#ifndef LAYOUT_CUDA_H
#define LAYOUT_CUDA_H

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cmath>

// This file defines the CUDA interface for the spring-electrical embedding layout algorithm.
// It includes a simple 2D vector struct for CUDA, an edge structure, and a function to compute forces.
// The compute_forces function is a CUDA kernel that calculates the forces acting on each vertex based on their positions and edges.
namespace layout_cuda {

// Simple 2D vector struct for CUDA
struct CudaVec2 {
    float x, y;
    __device__ CudaVec2() : x(0), y(0) {}
    __device__ CudaVec2(float x_, float y_) : x(x_), y(y_) {}
    __device__ CudaVec2 operator+(const CudaVec2& o) const { return CudaVec2(x + o.x, y + o.y); }
    __device__ CudaVec2 operator-(const CudaVec2& o) const { return CudaVec2(x - o.x, y - o.y); }
    __device__ CudaVec2& operator+=(const CudaVec2& o) { x += o.x; y += o.y; return *this; }
    __device__ CudaVec2 operator*(float s) const { return CudaVec2(x * s, y * s); }
    __device__ float length() const { return sqrtf(x * x + y * y); }
    __device__ CudaVec2 normalized() const {
        float len = length();
        return (len > 1e-6f) ? CudaVec2(x / len, y / len) : CudaVec2(0, 0);
    }
};

// Edge structure for CUDA
struct CudaEdge {
    int from;
    int to;
};

void compute_forces(
    const CudaVec2* h_positions,
    CudaVec2* h_forces,
    int n,
    float repulsion_constant,
    const CudaEdge* h_edges,
    int edge_count,
    float spring_length,
    float attraction_constant
);

}

#endif