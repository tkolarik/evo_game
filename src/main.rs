mod organism;
mod simulation;
mod evolution;
mod visualization;
mod phylogeny;

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use bevy_egui::{egui, EguiPlugin, EguiContexts};

use simulation::{PhysicsWorld as HeadlessPhysicsWorld, SimulationConfig, GROUND_Y_POSITION};
use evolution::EvolutionEngine;
use visualization::*;
use phylogeny::write_phylogeny_to_dot_file;

// --- Bevy App Setup ---

// Resource to manage the main evolution engine
#[derive(Resource)]
struct EvoResource(HeadlessPhysicsWorld, EvolutionEngine);

// Resource to manage batch running of generations
#[derive(Resource, Default)]
struct BatchRunConfig {
    target_generations_this_batch: Option<usize>,
    generations_completed_this_batch: usize,
}

// Enum to define the simulation mode
#[derive(States, Debug, Clone, PartialEq, Eq, Hash, Default)]
enum SimulationMode {
    #[default]
    Paused, // Initial state, or when explicitly paused
    RunningFast, // Non-visual, fast generation processing
    Visualizing, // Visualizing one individual
    // SteppingGeneration, // This mode was defined but not fully used, can be added later if needed
}

// Resource for simulation speed control
#[derive(Resource)]
pub struct SimulationSpeed(pub f32);

impl Default for SimulationSpeed {
    fn default() -> Self {
        SimulationSpeed(1.0)
    }
}

fn main() {
    println!("Vehicle Evolution Simulator Initializing...");

    let sim_config = SimulationConfig::default(); 
    let headless_physics_world = HeadlessPhysicsWorld::new(&sim_config);
    let evolution_engine = EvolutionEngine::new(sim_config.clone());

    App::new()
        .insert_resource(ClearColor(Color::rgb(0.1, 0.1, 0.15)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Vehicle Evolution Simulator".into(),
                resolution: (1280.0, 720.0).into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0))
        .add_plugins(RapierDebugRenderPlugin::default()) // Optional: for debugging colliders
        .add_plugins(EguiPlugin) // Add EguiPlugin
        
        // Initialize States
        .add_state::<SimulationMode>()

        // Resources
        .insert_resource(sim_config) // SimulationConfig is also a Bevy resource
        .insert_resource(EvoResource(headless_physics_world, evolution_engine))
        .init_resource::<CurrentVehicleChromosome>()
        .init_resource::<VisualizationState>()
        .init_resource::<SimulationSpeed>() // Initialize simulation speed resource
        .init_resource::<BatchRunConfig>() // Initialize batch run config

        // Startup Systems
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_ground_visualization) // Add a visual ground

        // Systems for controlling simulation flow and visualization
        .add_systems(Update, keyboard_controls)
        .add_systems(Update, run_fast_generations.run_if(in_state(SimulationMode::RunningFast)))
        
        // Systems for visualization mode
        .add_systems(OnEnter(SimulationMode::Visualizing), setup_current_vehicle_for_visualization)
        .add_systems(Update, step_vehicle_visualization_simulation.run_if(in_state(SimulationMode::Visualizing)))
        .add_systems(OnExit(SimulationMode::Visualizing), despawn_vehicle_parts)
        
        // UI System (placeholder)
        .add_systems(Update, ui_system_info_panel)
        .add_systems(Update, apply_simulation_speed) // System to apply speed to Time<Virtual>
        
        .run();
}

fn setup_ground_visualization(mut commands: Commands) {
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.3, 0.5, 0.3),
                custom_size: Some(Vec2::new(2000.0, 20.0)),
                ..default()
            },
            transform: Transform::from_xyz(0.0, GROUND_Y_POSITION * 100.0 - 10.0 , 0.0),
            ..default()
        },
        RigidBody::Fixed,
        Collider::cuboid(1000.0, 10.0),
    ));
}

fn keyboard_controls(
    mut next_sim_mode: ResMut<NextState<SimulationMode>>,
    current_sim_mode: Res<State<SimulationMode>>,
    keyboard_input: Res<Input<KeyCode>>,
    mut evo_res: ResMut<EvoResource>,
    config: Res<SimulationConfig>,
    mut vis_chromosome: ResMut<CurrentVehicleChromosome>,
    mut vis_state: ResMut<VisualizationState>,
    mut batch_run_config: ResMut<BatchRunConfig>, // Added for resetting on manual mode changes
) {
    if keyboard_input.just_pressed(KeyCode::F) {
        if *current_sim_mode.get() != SimulationMode::RunningFast {
            println!("Switching to RunningFast mode (continuous).");
            batch_run_config.target_generations_this_batch = None; // Ensure continuous mode
            batch_run_config.generations_completed_this_batch = 0;
            next_sim_mode.set(SimulationMode::RunningFast);
        } else {
            println!("Switching to Paused mode from RunningFast.");
            next_sim_mode.set(SimulationMode::Paused);
        }
    }

    if keyboard_input.just_pressed(KeyCode::V) { // Visualize best of current generation
        if *current_sim_mode.get() != SimulationMode::Visualizing {
            if let Some(best_current) = evo_res.1.population.individuals.first() {
                println!("Preparing to visualize best of current generation (ID: {}).", best_current.id);
                vis_chromosome.0 = Some(best_current.chromosome.clone());
                vis_state.is_simulating_current_vehicle = true; // Start simulation immediately
                vis_state.current_simulation_time = 0.0;
                next_sim_mode.set(SimulationMode::Visualizing);
            } else {
                println!("No individuals in population to visualize.");
            }
        } else {
             println!("Switching to Paused mode from Visualizing.");
            next_sim_mode.set(SimulationMode::Paused);
        }
    }
    
    if keyboard_input.just_pressed(KeyCode::B) { // Visualize all-time best
        if *current_sim_mode.get() != SimulationMode::Visualizing {
            if let Some(all_time_best) = &evo_res.1.all_time_best_individual {
                vis_chromosome.0 = Some(all_time_best.chromosome.clone());
                vis_state.is_simulating_current_vehicle = true; 
                vis_state.current_simulation_time = 0.0;
                next_sim_mode.set(SimulationMode::Visualizing);
            } else {
                println!("No all-time best individual recorded yet.");
            }
        } else {
            next_sim_mode.set(SimulationMode::Paused);
        }
    }

    if keyboard_input.just_pressed(KeyCode::P) { // Phylogeny export
        println!("Exporting phylogeny to phylogeny.dot...");
        if let Err(e) = write_phylogeny_to_dot_file(&evo_res.1, &config, "phylogeny.dot") {
            eprintln!("Failed to write phylogeny file: {}", e);
        }
    }

    if keyboard_input.just_pressed(KeyCode::R) { // Reset Simulation
        println!("Resetting simulation...");
        evo_res.1.reset();
        next_sim_mode.set(SimulationMode::Paused);
        vis_chromosome.0 = None;
        vis_state.is_simulating_current_vehicle = false;
        batch_run_config.target_generations_this_batch = None; // Reset batch config
        batch_run_config.generations_completed_this_batch = 0;
    }

    if keyboard_input.just_pressed(KeyCode::Space) { 
        match *current_sim_mode.get() {
            SimulationMode::Paused => { 
                println!("Resuming to RunningFast mode (continuous) from Paused.");
                batch_run_config.target_generations_this_batch = None; // Ensure continuous mode
                batch_run_config.generations_completed_this_batch = 0;
                next_sim_mode.set(SimulationMode::RunningFast);
            }
            _ => {
                println!("Pausing simulation.");
                next_sim_mode.set(SimulationMode::Paused);
            }
        }
    }
}

fn run_fast_generations(
    mut evo_res: ResMut<EvoResource>,
    config: Res<SimulationConfig>,
    mut next_sim_mode: ResMut<NextState<SimulationMode>>,
    mut batch_run_config: ResMut<BatchRunConfig>,
) {
    let EvoResource(ref mut physics_world, ref mut engine) = *evo_res;

    // If next_sim_mode is already trying to switch to Paused, respect that and don't run.
    if next_sim_mode.0 == Some(SimulationMode::Paused) {
        // If we were in a batch run, this pause might be premature (e.g. user pressed Space).
        // We should clear the batch target so it doesn't resume automatically if F is pressed later.
        batch_run_config.target_generations_this_batch = None;
        batch_run_config.generations_completed_this_batch = 0;
        return;
    }

    if engine.population.generation_count >= config.num_generations {
        println!("Target total generations ({}) reached. Switching to Paused.", config.num_generations);
        next_sim_mode.set(SimulationMode::Paused);
        batch_run_config.target_generations_this_batch = None;
        batch_run_config.generations_completed_this_batch = 0;
        return;
    }

    engine.evolve_generation(physics_world);
    println!("Completed generation {} (fast mode).", engine.population.generation_count);

    if let Some(target_batch_gens) = batch_run_config.target_generations_this_batch {
        batch_run_config.generations_completed_this_batch += 1;
        if batch_run_config.generations_completed_this_batch >= target_batch_gens || engine.population.generation_count >= config.num_generations {
            println!(
                "Batch run completed ({} / {} generations for this batch). Total gens: {}. Switching to Paused.",
                batch_run_config.generations_completed_this_batch,
                target_batch_gens,
                engine.population.generation_count
            );
            next_sim_mode.set(SimulationMode::Paused);
            batch_run_config.target_generations_this_batch = None; // Reset for next time
            batch_run_config.generations_completed_this_batch = 0;
        } else {
            // Continue batch: next_sim_mode remains RunningFast implicitly
            // (or we could explicitly set it if there was a chance it got changed)
        }
    } else {
        // This is continuous fast mode (no batch target)
        // The system will run again next frame if still in RunningFast mode.
        // If config.num_generations is hit, the check at the start of the function handles pausing.
        // No explicit change to next_sim_mode here unless total generations are met.
    }
}

// System to set up the vehicle for visualization. It needs to run once when entering Visualizing state.
fn setup_current_vehicle_for_visualization(
    mut commands: Commands, 
    vis_chromosome: Res<CurrentVehicleChromosome>,
    config: Res<SimulationConfig>,
    vehicle_parts_query: Query<Entity, With<VehiclePart>>,
    mut rapier_config: ResMut<RapierConfiguration> // Changed to use RapierConfiguration
) {
    for entity in vehicle_parts_query.iter() {
        commands.entity(entity).despawn_recursive();
    }
    // Reset Rapier context if necessary (e.g. clear bodies, though despawning should handle most)
    // This might not be needed if OnExit(Visualizing) and despawn_vehicle_parts works perfectly
    // and RapierPhysicsPlugin correctly handles removed entities.

    if let Some(chromosome) = &vis_chromosome.0 {
        let chassis_genes = &chromosome.chassis;
        let initial_chassis_y = config.initial_height_above_ground + chassis_genes.height / 2.0;

        // Ensure gravity is set for the visual simulation
        rapier_config.gravity = Vec2::new(0.0, config.gravity); // Corrected

        let chassis_entity = commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.7, 0.7, 0.8),
                    custom_size: Some(Vec2::new(chassis_genes.width, chassis_genes.height)),
                    ..default()
                },
                transform: Transform::from_xyz(0.0, initial_chassis_y, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(chassis_genes.width / 2.0, chassis_genes.height / 2.0),
            ActiveEvents::COLLISION_EVENTS, // For debugging if needed
            Name::new("Chassis"),
            VehiclePart,
        )).id();

        for wheel_gene in &chromosome.wheels {
            if wheel_gene.active {
                let wheel_x_abs = wheel_gene.get_x_position(chassis_genes.width);
                let wheel_y_abs = wheel_gene.get_y_position(chassis_genes.height);

                let wheel_entity = commands.spawn((
                    SpriteBundle {
                        sprite: Sprite {
                            color: Color::rgb(0.5, 0.5, 0.6),
                            custom_size: Some(Vec2::new(wheel_gene.radius * 2.0, wheel_gene.radius * 2.0)),
                            ..default()
                        },
                        transform: Transform::from_xyz(wheel_x_abs, wheel_y_abs, 0.1), // Positioned relative to chassis later via joint
                        ..default()
                    },
                    RigidBody::Dynamic,
                    Collider::ball(wheel_gene.radius),
                    Friction::coefficient(wheel_gene.friction_coefficient),
                    Restitution::coefficient(0.1),
                    Name::new("Wheel"),
                    VehiclePart,
                )).id();
                
                let motor_target_vel = if wheel_gene.motor_torque.abs() > 0.01 { 
                                           -wheel_gene.motor_torque.signum() * 30.0 // Increased target vel for more power
                                       } else { 0.0 };
                let motor_stiffness = wheel_gene.motor_torque.abs() * 0.5; // Adjusted stiffness
                let motor_damping = wheel_gene.motor_torque.abs() * 0.05;  // Adjusted damping

                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(Vec2::new(wheel_x_abs, wheel_y_abs)) // Anchor on chassis
                    .local_anchor2(Vec2::new(0.0, 0.0)) // Anchor on wheel center
                    .motor_velocity(motor_target_vel, motor_damping) // factor is damping for velocity motor
                    .motor_max_force(wheel_gene.motor_torque.abs() * 200.0) // Increased max force
                    .motor_model(MotorModel::ForceBased); // Try ForceBased model
                
                commands.entity(chassis_entity).insert(ImpulseJoint::new(wheel_entity, joint)); // Corrected Bevy component insertion
            }
        }
        println!("Spawned full vehicle for visualization.");
    } else {
        println!("No chromosome to visualize.");
    }
}

// System to apply simulation speed
fn apply_simulation_speed(sim_speed: Res<SimulationSpeed>, mut time: ResMut<Time<Virtual>>) {
    time.set_relative_speed(sim_speed.0);
}

// A simple UI panel to display info (requires a Bevy UI library like bevy_egui or native Bevy UI)
fn ui_system_info_panel(
    mut contexts: EguiContexts, // If using bevy_egui - needs to be added to App
    mut evo_res: ResMut<EvoResource>, // Changed from Res to ResMut
    sim_mode: Res<State<SimulationMode>>,
    config: Res<SimulationConfig>,
    mut vis_state: ResMut<VisualizationState>, // Changed from Res to ResMut
    mut vis_chromosome: ResMut<CurrentVehicleChromosome>,
    mut next_sim_mode: ResMut<NextState<SimulationMode>>,
    mut sim_speed: ResMut<SimulationSpeed>, // For simulation speed control via UI
    mut batch_run_config: ResMut<BatchRunConfig>,
) {
    egui::Window::new("Simulation Info & Controls").show(contexts.ctx_mut(), |ui| {
        ui.label(format!("Current Mode: {:?}", sim_mode.get()));
        ui.separator();
        ui.label(format!("Generation: {} / {}", evo_res.1.population.generation_count, config.num_generations));
        if let Some(best) = evo_res.1.population.individuals.first() {
            ui.label(format!("Best Fitness (Current Gen): {:.2}", best.fitness));
        }
        if let Some(best_overall) = &evo_res.1.all_time_best_individual {
            ui.label(format!("All-Time Best Fitness: {:.2} (Gen {})", best_overall.fitness, best_overall.generation));
        }
        let avg_fitness = if !evo_res.1.population.individuals.is_empty() {
            evo_res.1.population.individuals.iter().map(|ind| ind.fitness).sum::<f32>() / evo_res.1.population.individuals.len() as f32
        } else { 0.0 };
        ui.label(format!("Avg Fitness (Current Gen): {:.2}", avg_fitness));
        ui.separator();
        ui.heading("Controls (Keyboard):");
        ui.label("F - Toggle Fast Mode / Pause");
        ui.label("V - Visualize Best of Current Gen / Pause");
        ui.label("B - Visualize All-Time Best / Pause");
        ui.label("P - Export Phylogeny (.dot file)");
        ui.label("R - Reset Simulation");
        ui.label("Space - Pause / Resume to Fast Mode");
        ui.separator();
        ui.heading("Controls (UI Buttons):");
        if ui.button(if *sim_mode.get() == SimulationMode::RunningFast  && batch_run_config.target_generations_this_batch.is_none() { "Pause Fast Mode" } else { "Run Fast (Continuous)" }).clicked() {
            if *sim_mode.get() == SimulationMode::RunningFast && batch_run_config.target_generations_this_batch.is_none() {
                next_sim_mode.set(SimulationMode::Paused);
            } else {
                batch_run_config.target_generations_this_batch = None;
                batch_run_config.generations_completed_this_batch = 0;
                next_sim_mode.set(SimulationMode::RunningFast);
            }
        }
        if ui.button(if *sim_mode.get() == SimulationMode::Visualizing && vis_chromosome.0.as_ref().map_or(false, |c| evo_res.1.population.individuals.first().map_or(false, |bc| bc.chromosome == *c)) { "Stop Visualizing" } else { "Visualize Best of Gen" }).clicked() {
            if *sim_mode.get() == SimulationMode::Visualizing {
                next_sim_mode.set(SimulationMode::Paused);
            } else {
                 if let Some(best_current) = evo_res.1.population.individuals.first() {
                    vis_chromosome.0 = Some(best_current.chromosome.clone());
                    vis_state.is_simulating_current_vehicle = true; // Directly use ResMut
                    vis_state.current_simulation_time = 0.0; // Directly use ResMut
                    next_sim_mode.set(SimulationMode::Visualizing);
                } else {
                    println!("No individuals in current gen to visualize.");
                }
            }
        }
        if ui.button(if *sim_mode.get() == SimulationMode::Visualizing && vis_chromosome.0.as_ref().map_or(false, |c| evo_res.1.all_time_best_individual.as_ref().map_or(false, |bat| bat.chromosome == *c)) { "Stop Visualizing" } else { "Visualize All-Time Best" }).clicked() {
            if *sim_mode.get() == SimulationMode::Visualizing {
                next_sim_mode.set(SimulationMode::Paused);
            } else {
                 if let Some(all_time_best) = &evo_res.1.all_time_best_individual {
                    vis_chromosome.0 = Some(all_time_best.chromosome.clone());
                    vis_state.is_simulating_current_vehicle = true; // Directly use ResMut
                    vis_state.current_simulation_time = 0.0; // Directly use ResMut
                    next_sim_mode.set(SimulationMode::Visualizing);
                } else {
                    println!("No all-time best recorded to visualize.");
                }
            }
        }
        if ui.button("Export Phylogeny Tree (.dot)").clicked() {
             if let Err(e) = write_phylogeny_to_dot_file(&evo_res.1, &config, "phylogeny.dot") {
                eprintln!("Failed to write phylogeny file: {}", e);
            }
        }
        if ui.button("Reset Simulation").clicked() {
            evo_res.1.reset(); // evo_res is ResMut, .1.reset() is fine
            next_sim_mode.set(SimulationMode::Paused);
            vis_chromosome.0 = None;
            vis_state.is_simulating_current_vehicle = false; // Directly use ResMut
            batch_run_config.target_generations_this_batch = None;
            batch_run_config.generations_completed_this_batch = 0;
        }
        ui.separator();

        ui.label("Run Specific Number of Generations:");
        let run_batch = |gens: usize, batch_config: &mut BatchRunConfig, next_mode: &mut NextState<SimulationMode>| {
            batch_config.target_generations_this_batch = Some(gens);
            batch_config.generations_completed_this_batch = 0;
            next_mode.set(SimulationMode::RunningFast);
        };

        if ui.button("Run 1 Gen").clicked() {
            run_batch(1, &mut batch_run_config, &mut next_sim_mode);
        }
        if ui.button("Run 5 Gens").clicked() {
            run_batch(5, &mut batch_run_config, &mut next_sim_mode);
        }
        if ui.button("Run 10 Gens").clicked() {
            run_batch(10, &mut batch_run_config, &mut next_sim_mode);
        }

        ui.separator();
        ui.label("Simulation Speed (Visualized Mode Only):");
        // Use bevy_egui::egui::Slider
        if ui.add(egui::Slider::new(&mut sim_speed.0, 0.1..=5.0).text("Speed Factor")).changed() {
            // The apply_simulation_speed system will handle this.
        }
        ui.label(format!("Current Speed Factor: {:.2}", sim_speed.0));

    });
} 