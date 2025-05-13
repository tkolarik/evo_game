# Vehicle Evolution Simulator

A 2D vehicle evolution simulator built with Rust, the Bevy game engine, and Rapier2D for physics.
Organisms (vehicles) with procedurally generated chassis and wheels evolve over generations to achieve better locomotion, primarily by maximizing horizontal travel distance.

## Features

*   **Evolutionary Algorithm:** Uses concepts like selection (tournament), crossover (single-point, uniform), and mutation to evolve vehicle designs.
*   **Physics-Based Simulation:** Leverages Rapier2D for realistic 2D physics simulation of vehicles.
*   **Bevy Game Engine:** Utilizes Bevy for rendering, entity-component system management, and plugin architecture.
*   **Interactive Controls:** Allows for pausing, running generations in batches, visualizing individual organisms, and resetting the simulation.
*   **Phylogeny Tracking:** Can generate a `.dot` file to visualize the lineage of individuals.

## Building and Running

1.  **Install Rust:** If you don't have Rust installed, get it from [rustup.rs](https://rustup.rs/).
2.  **Clone the Repository:**
    ```bash
    git clone <your-repository-url>
    cd evo_game
    ```
3.  **Run the Simulator:**
    ```bash
    cargo run
    ```
    Or, for a release build (recommended for performance):
    ```bash
    cargo run --release
    ```

## Controls

The simulation can be controlled via keyboard shortcuts and a UI panel.

### Keyboard Controls:

*   **`F`**: Toggle continuous fast mode (evolves generations without visualization) / Pause.
*   **`V`**: Visualize the best individual of the current generation / Pause.
*   **`B`**: Visualize the all-time best individual recorded / Pause.
*   **`P`**: Export the current phylogeny tree to `phylogeny.dot` (can be visualized with Graphviz).
*   **`R`**: Reset the entire simulation to generation 0.
*   **`Space`**: Pause the simulation if running, or resume to continuous fast mode if paused.

### UI Panel:

The UI panel provides buttons for most of the keyboard controls, plus:

*   **Run 1 Gen / Run 5 Gens / Run 10 Gens**: Evolve a specific number of generations and then pause.
*   **Simulation Speed Slider**: Adjust the speed of the visualization playback (does not affect headless simulation speed).

## Project Structure

*   `src/main.rs`: Main application setup, Bevy plugins, UI, and simulation mode management.
*   `src/organism.rs`: Defines the genetic structure (genes, chromosome) of vehicles.
*   `src/simulation.rs`: Handles the headless physics simulation using Rapier2D to evaluate individual fitness.
*   `src/evolution.rs`: Implements the evolutionary algorithm (population, selection, crossover, mutation).
*   `src/visualization.rs`: Contains Bevy systems for setting up the camera and rendering vehicles for visualization.
*   `src/phylogeny.rs`: Logic for tracking and exporting the evolutionary lineage of individuals.

## Dependencies

*   `bevy`: Game engine.
*   `bevy_rapier2d`: 2D physics integration for Bevy.
*   `bevy_egui`: Egui integration for Bevy (used for the UI panel).
*   `rand`, `rand_distr`: For random number generation, crucial for mutation and initial population.
*   `rapier2d`: Core 2D physics engine.
*   `dot`: For generating Graphviz DOT files (used in phylogeny).