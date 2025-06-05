# HGVisualizer

**HGVisualizer** is an interactive C++ simulation and visualization tool for evolving hypergraphs, inspired by the [Wolfram Physics Project](https://www.wolframphysics.org/). It features a dynamic rule-based graph model and a real-time spring-electrical embedding layout for clear and engaging visualization.

[![HGVisualizer Demo](https://img.youtube.com/vi/2NnOHmyoMic/0.jpg)](https://youtu.be/2NnOHmyoMic)

---

## What Are Hypergraphs and the Wolfram Physics Project?

A **hypergraph** is a generalization of a graph where an edge (called a "hyperedge") can connect any number of nodes, not just two. The [Wolfram Physics Project](https://www.wolframphysics.org/) explores the idea that the universe can be modeled as a hypergraph, with simple local rules driving the evolution of the entire structure. In this project, we use a simplified (pairwise) version of these ideas to experiment with emergent complexity and graph dynamics.

---

## Project Features

- **Rule-based Graph Evolution:**  
  The graph evolves by applying local rules to nodes and edges, mimicking the approach of the Wolfram Physics Project.
- **Spring-Electrical Embedding Layout:**  
  The graph is rendered using a force-directed layout, where nodes repel each other and edges act as springs, resulting in a readable and aesthetically pleasing visualization.
- **Interactive Visualization:**  
  Pan, zoom, and observe the evolution of the graph in real time.
- **Configurable Parameters:**  
  Easily adjust the number of nodes, edge thresholds, layout constants, and more.

---

## The Two Evolution Rules

1. **Edge Removal Rule:**  
   - If a node has more than a minimum number of edges, randomly select one of its edges.
   - If the other node connected by this edge also has more than the minimum number of edges, remove the edge.
   - This prevents nodes from becoming isolated and maintains a balanced structure.

2. **Edge Creation Rule:**  
   - If a node has at least one edge but fewer than a maximum threshold, randomly select one of its neighbors.
   - Then, randomly select a neighbor of that neighbor (a "second-tier neighbor") that is not already directly connected.
   - If such a node exists and does not exceed the edge threshold, create a new edge between the original node and this second-tier neighbor.
   - This encourages the formation of new connections and increases graph complexity.

---

## Significance of Edge Creation and Removal

- **Edge Creation** reduces the overall path distance between nodes, effectively "pulling" nodes closer together in the network. This provides an analogue for the **force of gravity** in the model, as it increases connectivity and can lead to clustering or condensation of the graph structure.
- **Edge Removal** increases the overall path distance between nodes, effectively "pushing" nodes apart in the network. This acts as an analogue for an **expanding universe**, where the removal of connections can be seen as the creation of "new space" between regions of the graph, causing them to drift apart.
- **Node Count is Conserved:**  
  Throughout the simulation, the number of nodes remains constant. Only the connections (edges) between nodes are created or removed, reflecting a universe where the fundamental entities are preserved but their relationships evolve.
- **Terminal States:**  
  During the evolution, certain regions of the graph may reach a **terminal state**—a configuration in which no further rules can be applied to the nodes in that region. These terminal states can be interpreted as analogues of **black holes** or **singularities** in the model, representing areas where the local structure has become "frozen" and isolated from further evolution. This highlights how simple local rules can give rise to complex and emergent phenomena reminiscent of features in our universe.
- **Event Horizons:**  
  As the graph evolves, it is possible for parts of the graph to become disconnected from the rest, forming isolated subgraphs. These disconnected regions can be thought of as analogues of **event horizons**—boundaries beyond which information or influence cannot propagate to the rest of the network. In cosmology, the cosmic event horizon is defined by the limits imposed by the speed of light and the expansion of the universe, beyond which events cannot affect an observer. Similarly, in the simulation, once a region becomes disconnected, it is causally isolated from the rest of the graph, and its evolution proceeds independently. This provides a conceptual parallel to the cosmic event horizon and highlights how local rules and edge dynamics can lead to emergent boundaries within the network.
- **Maximum Entropy:**  
  The simulation can also explore states of **maximum entropy**, where the graph reaches a highly disordered or randomized configuration. In such a state, the connections between nodes are as uniformly distributed as possible, with no large-scale structure or clustering. This can be seen as an analogue to a universe in thermal equilibrium, where all distinctions between regions are erased and the system has reached its highest possible disorder.

---

## Spring-Electrical Embedding Layout

To make the evolving graph readable, HGVisualizer uses a **spring-electrical embedding** algorithm:

- **Repulsion:** All nodes repel each other, preventing overlap and spreading the graph out.
- **Attraction:** Edges act as springs, pulling connected nodes together toward a preferred distance.
- **Damping:** Motion is damped to prevent oscillations and help the layout stabilize.
- **Iterations:** The layout is updated over multiple iterations per frame for smooth convergence.

This approach is based on the method described in the [Wolfram Language documentation](https://reference.wolfram.com/language/ref/method/SpringElectricalEmbedding.html).

---

## Building and Running

1. **Dependencies:**  
   - C++20 compiler  
   - [SDL3](https://github.com/libsdl-org/SDL)  
   - [SDL3_ttf](https://github.com/libsdl-org/SDL_ttf)
   - [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) (for GPU-accelerated layout)  

2. **Build:**  
   Use CMake:
   ```sh
   cmake -S . -B build
   cmake --build build
   ```

3. **Run:**  
   ```
   ./build/HGVisualizerApp

---

## Configuration File

HGVisualizer supports a configuration file (`app.config`) that allows you to easily set simulation and layout parameters without recompiling the code. The config file should be placed in the `res` directory and uses a simple `name=value` format, with one property per line.

Example: `<executable directory>/res/app.config`

### Parameter Descriptions

- **model.node_count**  
  The total number of nodes in the graph.

- **model.updates_per_frame**  
  How many times the graph evolution rules are applied per frame.

- **model.node_max_edges**  
  The maximum number of edges a node can have.

- **model.node_min_edges**  
  The minimum number of edges a node should maintain.

- **model.node_min_edges_decay**  
  How often (in frames) the minimum edge count is reduced (0 means no decay).

- **model.node_min_edges_floor**  
  The lowest value the minimum edge count can decay to.

- **layout.spring_length**  
  The preferred distance between connected nodes (spring rest length).

- **layout.iterations_per_frame**  
  How many times the layout algorithm runs per frame (higher = more stable, but slower).

- **layout.repulsion_constant**  
  The strength of the repulsive force between all nodes.

- **layout.attraction_constant**  
  The strength of the attractive (spring) force along edges.

- **layout.timestep**  
  The time step for each layout iteration (affects movement speed).

- **layout.damping_factor**  
  Damping applied to node velocities to reduce oscillations.

- **layout.stabilization_iterations**  
  Number of extra layout iterations to help the graph stabilize after changes.
   ```
---

## Controls

- **Mouse Drag:** Pan the graph
- **Mouse Wheel:** Zoom in/out (centered on mouse)
- **Space:** Toggle updates (rule application)
- **Tab:** Toggle layout updates
- **Enter:** Pause/resume simulation

---

## License

This project is licensed under the GNU General Public License v3.0.

---

## Credits

- Inspired by [Wolfram Physics Project](https://www.wolframphysics.org/)
- Developed by Joe Mruz

---

## Work in Progress

HGVisualizer is an active work in progress. Recent updates include:

- **Configuration file support:** You can now easily adjust simulation and visualization parameters by editing `res/app.config` without recompiling.
- **CUDA acceleration:** GPU-accelerated layout is now supported via the NVIDIA CUDA Toolkit for improved performance on large graphs.

Planned features:

- Support for user-defined/custom rules to experiment with new behaviors.
- More interactive controls to reset the graph or modify parameters during runtime.
- Further improvements to usability and performance.

Feedback and contributions are welcome!