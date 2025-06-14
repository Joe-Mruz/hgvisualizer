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

    constexpr int   WINDOW_WIDTH        {1440};
    constexpr int   WINDOW_HEIGHT       {1080};
    constexpr float VERTEX_RADIUS       {3.0f};
    constexpr const char* FONT_PATH     {"res/PixelSplitter-Bold.ttf"};
    constexpr const char* WINDOW_TITLE  {"HGVisualizer - Hypergraph Visualization Tool"};

    bool                paused                      {false};
    bool                do_updates                  {true};
    bool                draw_nodes                  {false};
    bool                dragging                    {false};
    int                 mouse_x                     {0};
    int                 mouse_y                     {0};
    int                 cycle                       {0};
    layout::Vec2        drag_offset                 {0.0f, 0.0f};
    float               zoom                        {1.0f};
    layout::Vec2        smoothed_centroid           {0.0f, 0.0f};
    bool                centroid_initialized        {false};
    layout::Vertex      hovered_vertex              {0, {0.0f, 0.0f}};
    bool                highlight_vertex_enabled    {false};
    model::ModelTime    last_update_time            {0};

    // Helper function to check if a point is inside a circle (used for vertex hover detection)
    bool is_point_in_rect(int px, int py, layout::Vec2 center, float radius) {
        float dx = px - center.x();
        float dy = py - center.y();
        return (dx * dx + dy * dy) <= (radius * radius);
    }

    void draw_vertex(SDL_Renderer* renderer, layout::Vec2 position, SDL_Color color = {180, 200, 210, 255}) {

        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);

        SDL_FRect rect;
        rect.x = static_cast<int>(position.x() - VERTEX_RADIUS);
        rect.y = static_cast<int>(position.y() - VERTEX_RADIUS);
        rect.w = static_cast<int>(VERTEX_RADIUS * 2);
        rect.h = static_cast<int>(VERTEX_RADIUS * 2);

        SDL_RenderFillRect(renderer, &rect);
    }

    void draw_edge(SDL_Renderer* renderer, layout::Vec2 from, layout::Vec2 to, SDL_Color color = {86, 190, 179, 100}) {
        SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
        SDL_RenderLine(renderer, from.x(), from.y(), to.x(), to.y());
    }

    std::string format_rule(const model::Rule& rule) {
        std::string filter;
        switch (rule.lhs().filter()) {
            case model::FilterType::Edge: filter = "Edges"; break;
            default: filter = "Unknown"; break;
        }

        std::string compare;
        switch (rule.lhs().compare()) {
            case model::FilterCompareType::Equal: compare = "=="; break;
            case model::FilterCompareType::NotEqual: compare = "!="; break;
            case model::FilterCompareType::LessThan: compare = "<"; break;
            case model::FilterCompareType::LessThanOrEqual: compare = "<="; break;
            case model::FilterCompareType::GreaterThan: compare = ">"; break;
            case model::FilterCompareType::GreaterThanOrEqual: compare = ">="; break;
            default: compare = "Unknown"; break;
        }

        std::string operation;
        switch (rule.rhs().op()) {
            case model::OperationType::Expand: operation = "Expand"; break;
            case model::OperationType::Merge: operation = "Merge"; break;
            case model::OperationType::Split: operation = "Split"; break;
            case model::OperationType::Integrate: operation = "Integrate"; break;
            case model::OperationType::Decay: operation = "Decay"; break;
            case model::OperationType::ExpandAll: operation = "ExpandAll"; break;
            default: operation = "Unknown"; break;
        }

        return std::format("{} {} {} -> {} {}", filter, compare, rule.lhs().value(), operation, rule.rhs().value());
    }

    void draw_rules(SDL_Renderer* renderer, TTF_Font* font, const std::vector<model::Rule>& rules, layout::Vec2 position) {
        SDL_Color text_color = {150, 150, 150, 255};
        std::string rules_header = "---Rules---";
        SDL_Surface* surface = TTF_RenderText_Solid(font, rules_header.c_str(), rules_header.length(), text_color);
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FRect dst_rect {
            static_cast<float>(position.x()), 
            static_cast<float>(position.y()), 
            static_cast<float>(surface->w), 
            static_cast<float>(surface->h)
        };
        SDL_RenderTexture(renderer, texture, nullptr, &dst_rect);
        int y_offset = surface->h;

        SDL_DestroySurface(surface);
        SDL_DestroyTexture(texture);

        for (const auto& rule : rules) {
            std::string rule_text = format_rule(rule);

            SDL_Surface* surface = TTF_RenderText_Solid(font, rule_text.c_str(), rule_text.length(), text_color);
            if (!surface) {
                std::cerr << "TTF_RenderText_Solid Error: " << SDL_GetError() << std::endl;
                continue;
            }

            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
            if (!texture) {
                std::cerr << "SDL_CreateTextureFromSurface Error: " << SDL_GetError() << std::endl;
                SDL_DestroySurface(surface);
                continue;
            }

            SDL_FRect dst_rect {
                static_cast<float>(position.x()),
                static_cast<float>(position.y()) + y_offset,
                static_cast<float>(surface->w),
                static_cast<float>(surface->h)
            };

            SDL_RenderTexture(renderer, texture, NULL, &dst_rect);

            y_offset += surface->h;

            SDL_DestroyTexture(texture);
            SDL_DestroySurface(surface);
        }
    }
}

// Run the UI loop, handling events and rendering the model and layout.
// This function initializes SDL, creates a window and renderer, and enters the main loop.
void ui::run(
        const std::unique_ptr<model::Model>& model, 
        const std::unique_ptr<layout::SpringElectricalEmbedding>& layout, 
        const int updates_per_frame) {

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
                if (event.key.key == SDLK_SPACE) {
                    paused = !paused;
                }
                if (event.key.key == SDLK_TAB) {
                    do_updates = !do_updates;
                }
                if (event.key.key == SDLK_N) {
                    draw_nodes = !draw_nodes;
                }
                if (event.key.key == SDLK_H) {
                    highlight_vertex_enabled = !highlight_vertex_enabled;
                }
                if (event.key.key == SDLK_ESCAPE) {
                    model->reset();
                    layout->reset();
                    cycle = 0;
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
            if (do_updates && cycle % 2 == 0) {
                for (int i {0}; i < updates_per_frame; ++i) {
                    model->update();
                    layout->update(*model);
                }
            }

            layout->do_layout(*model);
        }

        // Clear the window
        SDL_SetRenderDrawColor(SDL_GetRenderer(window), 0, 0, 0, 255);
        SDL_RenderClear(SDL_GetRenderer(window));

        // Calculate screen center by averaging of the layout vertices positions
        layout::Vec2 centroid;
        for (const auto& vertex : layout->vertices()) {
            if (!vertex.alive_) { 
                continue; // Skip dead vertices
            }
            centroid += vertex.position_;
        }
        centroid /= static_cast<float>(layout->vertices().size());

        // Smooth the centroid
        float alpha = 0.15f; // Smoothing factor (0 = no movement, 1 = instant snap)
        if (!centroid_initialized) {
            smoothed_centroid = centroid;
            centroid_initialized = true;
        } else {
            smoothed_centroid = smoothed_centroid * (1.0f - alpha) + centroid * alpha;
        }

        layout::Vec2 screen_center(WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f);
        layout::Vec2 offset = screen_center - smoothed_centroid * zoom + drag_offset;

        constexpr SDL_Color highlight_color = {255, 100, 100, 255};
        constexpr SDL_Color default_vertex_color = {180, 200, 210, 255};
        constexpr SDL_Color default_edge_color = {86, 190, 179, 100};
        constexpr SDL_Color new_edge_color = {80, 255, 80, 255};

        // Draw edges
        for (const auto& edge : model->edges()) {
            auto from_vertex = layout->vertices()[edge.value().from()];
            auto to_vertex = layout->vertices()[edge.value().to()];

            SDL_Color edge_color = default_edge_color;
            if ((hovered_vertex.id() == from_vertex.id() || hovered_vertex.id() == to_vertex.id()) && highlight_vertex_enabled) {
                edge_color = highlight_color; // Highlight edge if hovered vertex is connected
            }

            if (from_vertex.created_time_ > last_update_time || to_vertex.created_time_ > last_update_time) {
                edge_color = new_edge_color; // Highlight new edges
            }

            draw_edge(renderer, from_vertex.position_ * zoom + offset, to_vertex.position_ * zoom + offset, edge_color);
        }

        // Draw vertices
        if (draw_nodes) {
            for (const auto& vertex : layout->vertices()) {
                if (!vertex.alive_) { 
                    continue; // Skip dead vertices
                }

                // Check if the vertex is hovered
                if (is_point_in_rect(mouse_x, mouse_y, vertex.position_ * zoom + offset, VERTEX_RADIUS)) {
                    hovered_vertex = vertex;
                }

                SDL_Color vertex_color = default_vertex_color;
                if (hovered_vertex.id() == vertex.id()) {
                    vertex_color = highlight_color; // Highlight hovered vertex
                }

                draw_vertex(renderer, vertex.position_ * zoom + offset, vertex_color);
            }
        }

        last_update_time = model->time();

        if (model->terminal_state()) {
            // Draw text "Terminal State" in the top of the screen
            SDL_Color color = {255, 100, 100, 255};
            const std::string text = "Terminal State (ESC to reset)";
            SDL_Surface* text_surface = TTF_RenderText_Solid(font, text.c_str(), text.length(), color);
            if (text_surface) {
                SDL_Texture* text_texture = SDL_CreateTextureFromSurface(renderer, text_surface);
                if (text_texture) {
                    SDL_FRect dstrect = {300.f, 10.f, static_cast<float>(text_surface->w), static_cast<float>(text_surface->h)};
                    SDL_RenderTexture(renderer, text_texture, NULL, &dstrect);
                    SDL_DestroyTexture(text_texture);
                }
                SDL_DestroySurface(text_surface);
            }
        }

        // Draw debug information
        std::array<std::string, 18> debug_texts = {
            std::string("---Control---"),
            std::format("Offset (MDRAG): x={:.2f} y={:.2f}", offset.x(), offset.y()),
            std::format("Zoom (MWHEEL): {:.2f}", zoom),
            std::format("Update (TAB): {}", do_updates ? "ON" : "OFF"),
            std::format("Paused (SPACE): {}", paused ? "YES" : "NO"),
            std::format("Show Nodes (N): {}", draw_nodes ? "YES" : "NO"),
            std::format("Highlight Vertex (H): {}", highlight_vertex_enabled ? "YES" : "NO"),

            std::string("---Layout---"),
            std::format("Repulsion: {:.2f}", layout->repulsion_constant()),
            std::format("Attraction: {:.2f}", layout->attraction_constant()),
            std::format("Timestep: {:.2f}", layout->timestep()),
            std::format("Spring Length: {:.2f}", layout->spring_length()),
            std::format("Damping: {:.2f}", layout->damping()),
            std::format("Iterations: {}", layout->iterations()),

            std::string("---Model---"),
            std::format("Nodes: {}", model->node_count()),
            std::format("Edges: {}", model->edge_count()),
            std::format("Rule Misses: {:.0f}%", model->rule_miss_percentage()),
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

        draw_rules(renderer, font, model->rules(), {10.0f, static_cast<float>(y)});

        SDL_RenderPresent(renderer);
        cycle++;
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
}