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
    config::Rules rules {};
    rules.load("res/rules.config");

    layout::VertexArray vertices;

    auto hgraph_layout = std::make_unique<layout::SpringElectricalEmbedding>(
        std::move(vertices),
        config.layout_repulsion_constant(),
        config.layout_attraction_constant(),
        config.layout_timestep(),
        config.layout_spring_length(),
        config.layout_damping_factor(),
        config.layout_stablization_iterations()
    );

    auto nodes = std::make_unique<model::NodeArray>();
    auto edges = std::make_unique<model::EdgeArray>();

    auto hgraph_model = std::make_unique<model::Model>(
        rules.rules(),
        *edges,
        *nodes
    );

    ui::run(
        hgraph_model, 
        hgraph_layout, 
        config.model_updates_per_frame() // higher values run faster, but rendering is less granular and the layout processing may lag behind
    );

    return 0;
}