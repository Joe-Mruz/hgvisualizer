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
#ifndef UI_H
#define UI_H

#include <memory>
#include "model.h"
#include "layout.h"

// This module provides the user interface for the HGraph application.
// It handles rendering the graph, processing user input, and managing the layout of the graph.
namespace ui {

void run(const std::unique_ptr<model::Model>& model, 
         const std::unique_ptr<layout::SpringElectricalEmbedding>& layout, 
         const int updates_per_frame = 1,
         const int layout_iterations_per_frame = 1);
}

#endif