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
#include "ui.h"
#include "model.h"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <iostream>
#include <format>
#include <array>

namespace {

    constexpr int   WINDOW_WIDTH  {1440};
    constexpr int   WINDOW_HEIGHT {1080};
    constexpr float VERTEX_RADIUS {3.0f};
    constexpr const char* FONT_PATH  {"res/PixelSplitter-Bold.ttf"};
    constexpr const char* WINDOW_TITLE {"HGVisualizer - Hypergraph Visualization Tool"};

    bool            paused      {false};
    bool            do_updates  {true};
    bool            do_layout   {true};
    bool            dragging    {false};
    int             mouse_x     {0};
    int             mouse_y     {0};
    int             cycle       {0};
    layout::Vec2    drag_offset {0.0f, 0.0f};
    float           zoom        {1.0f};

    void draw_vertex(SDL_Renderer* renderer, layout::Vec2 position) {

        SDL_SetRenderDrawColor(renderer, 180, 200, 210, 255);

        SDL_FRect rect;
        rect.x = static_cast<int>(position.x() - VERTEX_RADIUS);
        rect.y = static_cast<int>(position.y() - VERTEX_RADIUS);
        rect.w = static_cast<int>(VERTEX_RADIUS * 2);
        rect.h = static_cast<int>(VERTEX_RADIUS * 2);

        SDL_RenderFillRect(renderer, &rect);
    }

    void draw_edge(SDL_Renderer* renderer, layout::Vec2 from, layout::Vec2 to) {
        SDL_SetRenderDrawColor(renderer, 86, 190, 179, 100);
        SDL_RenderLine(renderer, from.x(), from.y(), to.x(), to.y());
    }
}

// Run the UI loop, handling events and rendering the model and layout.
// This function initializes SDL, creates a window and renderer, and enters the main loop.
void ui::run(const std::unique_ptr<model::Model>& model, 
        const std::unique_ptr<layout::SpringElectricalEmbedding>& layout, 
        const int updates_per_frame, 
        const int layout_iterations_per_frame)
{
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return;
    }

    if (!TTF_Init()) {
        std::cerr << "TTF_Init() Error: " << SDL_GetError() << std::endl;
        return;
    }

    SDL_Window* window = SDL_CreateWindow(WINDOW_TITLE, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    if (!window) {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, NULL);
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return;
    }

    // Enable alpha blending for the renderer
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    TTF_Font* font = TTF_OpenFont(FONT_PATH, 12);
    if (font == NULL) {
        std::cerr << "TTF_OpenFont() Error: " << SDL_GetError() << std::endl;
        return;
    }

    bool running = true;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_EVENT_QUIT) {
                running = false;
            }
            if (event.type == SDL_EVENT_KEY_DOWN) {
                if (event.key.key == SDLK_RETURN) {
                    paused = !paused;
                }
                if (event.key.key == SDLK_SPACE) {
                    do_updates = !do_updates;
                }
                if (event.key.key == SDLK_TAB) {
                    do_layout = !do_layout;
                }   
            }
            if (event.type == SDL_EVENT_MOUSE_WHEEL) {
                // 1. Get mouse position in screen coordinates
                float mx, my;
                SDL_GetMouseState(&mx, &my);

                // 2. Calculate centroid and screen center as above
                layout::Vec2 centroid;
                for (const auto& vertex : layout->vertices()) {
                    centroid += vertex.position_;
                }
                centroid /= static_cast<float>(layout->vertices().size());
                layout::Vec2 screen_center(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f);

                // 3. Compute the offset before zoom
                layout::Vec2 offset = screen_center - centroid * zoom + drag_offset;

                // 4. Compute world position under mouse before zoom
                layout::Vec2 mouse_screen(static_cast<float>(mx), static_cast<float>(my));
                layout::Vec2 mouse_world_before = (mouse_screen - offset) / zoom;

                // 5. Change zoom
                float old_zoom = zoom;
                if (event.wheel.y > 0) {
                    zoom *= 1.1f;
                } else if (event.wheel.y < 0) {
                    zoom /= 1.1f;
                }
                if (zoom < 0.1f) zoom = 0.1f;
                if (zoom > 10.0f) zoom = 10.0f;

                // 6. Compute new offset so mouse_world_before stays under the mouse
                offset = screen_center - centroid * zoom + drag_offset;
                layout::Vec2 mouse_world_after = (mouse_screen - offset) / zoom;
                layout::Vec2 world_delta = mouse_world_after - mouse_world_before;
                drag_offset += world_delta * zoom;
            }
            if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN) {
                if (event.button.button == SDL_BUTTON_LEFT) {
                    dragging = true;
                }
            }
            if (event.type == SDL_EVENT_MOUSE_BUTTON_UP) {
                if (event.button.button == SDL_BUTTON_LEFT) {
                    dragging = false;
                }
            }
            if (event.type == SDL_EVENT_MOUSE_MOTION) {
                int prev_mouse_x {mouse_x};
                int prev_mouse_y {mouse_y};
                mouse_x = event.motion.x;
                mouse_y = event.motion.y;

                if (dragging) {
                    drag_offset += {
                        static_cast<float>(mouse_x - prev_mouse_x), 
                        static_cast<float>(mouse_y - prev_mouse_y)
                    };
                }
            }
        }
        SDL_Delay(16); // ~60 FPS

        if (!paused) {
            if (cycle % 1 == 0) {
                if (do_updates) {
                    for (int i {0}; i < updates_per_frame; ++i) {
                        model->update();
                    }
                }
                if (do_layout) {
                    for (int i {0}; i < layout_iterations_per_frame; ++i) {
                        layout->do_stable_layout(model->edges_);
                    }
                }
            }
            cycle++;
        }

        // Clear the window
        SDL_SetRenderDrawColor(SDL_GetRenderer(window), 0, 0, 0, 255);
        SDL_RenderClear(SDL_GetRenderer(window));

        // Calculate screen center by averaging of the layout vertices positions
        layout::Vec2 centroid;
        for (const auto& vertex : layout->vertices()) {
            centroid += vertex.position_;
        }
        centroid /= static_cast<float>(layout->vertices().size());

        layout::Vec2 screen_center(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f);
        layout::Vec2 offset = screen_center - centroid * zoom + drag_offset;

        // Draw edges
        for (const auto& vertex : layout->vertices()) {
            auto& node = model->nodes_[vertex.id()];

            // for each neighbor, draw an edge
            for (const auto& edge_id : node.edges_) {
                auto edge = model->edges_.get(edge_id);
                if (edge) {
                    auto other_id = edge->other(vertex.id());
                    auto other = layout->vertices()[other_id];
                    draw_edge(renderer, vertex.position_ * zoom + offset, other.position_ * zoom + offset);
                }
            }
        }

        // Draw vertices
        for (const auto& vertex : layout->vertices()) {
            auto& node = model->nodes_[vertex.id()];
            draw_vertex(renderer, vertex.position_ * zoom + offset);
        }

        // Draw debug information
        std::array<std::string, 17> debug_texts = {
            std::format("Offset (MDRAG): x={:.2f} y={:.2f}", offset.x(), offset.y()),
            std::format("Update (SPACE): {}", do_updates ? "ON" : "OFF"),
            std::format("Layout (TAB): {}", do_layout ? "ON" : "OFF"),
            std::format("Paused (ENTER): {}", paused ? "YES" : "NO"),
            std::format("Zoom (MSCROLL): {:.2f}", zoom),
            std::format("Max Edges Per Node: {}", model->edge_threshold()),
            std::format("Edge Retainment: {}", model->edge_retainment_threshold()),
            std::format("Retainment Decay: {}", model->edge_retainment_decay()),
            std::format("Retainment Floor: {}", model->edge_retainment_floor()),
            std::format("Nodes: {}", model->node_count()),
            std::format("Edges: {}", model->edge_count()),
            std::format("Repulsion: {:.2f}", layout->repulsion_constant()),
            std::format("Attraction: {:.2f}", layout->attraction_constant()),
            std::format("Timestep: {:.2f}", layout->timestep()),
            std::format("Spring Length: {:.2f}", layout->spring_length()),
            std::format("Damping: {:.2f}", layout->damping()),
            std::format("Iterations: {}", layout->iterations())
        };

        SDL_Color color = {150, 150, 150, 255};
        int y = 10;
        const int line_height = 16;

        for (const auto& text : debug_texts) {
            SDL_Surface* text_surface = TTF_RenderText_Solid(font, text.c_str(), text.length(), color);
            if (text_surface) {
                SDL_Texture* text_texture = SDL_CreateTextureFromSurface(renderer, text_surface);
                if (text_texture) {
                    SDL_FRect dstrect = {10.f, static_cast<float>(y), static_cast<float>(text_surface->w), static_cast<float>(text_surface->h)};
                    SDL_RenderTexture(renderer, text_texture, NULL, &dstrect);
                    SDL_DestroyTexture(text_texture);
                }
                SDL_DestroySurface(text_surface);
            }
            y += line_height;
        }

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
}