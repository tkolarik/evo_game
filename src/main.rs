mod organism;
mod simulation;
mod evolution;
mod visualization;
mod phylogeny;

use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use bevy_egui::{egui, EguiPlugin, EguiContexts};

use simulation::{PhysicsWorld as HeadlessPhysicsWorld, SimulationConfig};
use evolution::EvolutionEngine;
use visualization::*;
use phylogeny::write_phylogeny_to_dot_file;

// --- Collision Groups ---
const GROUP_VEHICLE: u32 = 1 << 0; // 0b00000001
const GROUP_GROUND: u32 = 1 << 1;  // 0b00000010
// Add more groups as needed, e.g., GROUP_OBSTACLE = 1 << 2;

const VEHICLE_FILTER: u32 = GROUP_GROUND; // Vehicles collide with ground
const GROUND_FILTER: u32 = GROUP_VEHICLE;   // Ground collides with vehicles

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

// Add this resource to store the distance data
#[derive(Resource, Default)]
pub struct DistanceComparisonData {
    pub simulated: Option<f32>,
    pub visualized: Option<f32>,
}

// Resource to control simulation state for visualization
#[derive(Resource)]
pub struct VisualizationState {
    pub is_simulating_current_vehicle: bool,
    pub current_simulation_time: f32,
    pub show_best_overall: bool, // Flag to show all-time best vs current gen best
    pub is_physics_paused: bool, // New flag: true to pause Rapier physics during visualization
    pub single_step_requested: bool, // New flag: true to advance physics by one tick when paused
    pub use_debug_vehicle: bool, // New flag: true to use a hardcoded debug vehicle
    // For distance tracking and comparison
    pub initial_chassis_position: Option<Vec2>,
    pub expected_fitness: Option<f32>,
    pub auto_stop_visualization: bool, // New flag: true to automatically stop visualization after sim_duration_secs
}

impl Default for VisualizationState {
    fn default() -> Self {
        Self {
            is_simulating_current_vehicle: false,
            current_simulation_time: 0.0,
            show_best_overall: false,
            is_physics_paused: false, // Default to not paused
            single_step_requested: false, // Default to no single step requested
            use_debug_vehicle: false, // Default to not using debug vehicle
            initial_chassis_position: None,
            expected_fitness: None,
            auto_stop_visualization: true, // Default to auto-stopping for fair comparison
        }
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
        .init_resource::<DistanceComparisonData>()

        // Configure physics to use fixed timestep for better stability
        .insert_resource(Time::<Fixed>::from_seconds(1.0 / 60.0))

        // Startup Systems
        .add_systems(Startup, setup_graphics)
        .add_systems(Startup, setup_ground_visualization) // Add a visual ground
        .add_systems(Startup, configure_physics) // Configure physics to match headless simulation

        // Systems for controlling simulation flow and visualization
        .add_systems(Update, keyboard_controls)
        .add_systems(Update, run_fast_generations.run_if(in_state(SimulationMode::RunningFast)))
        
        // Systems for visualization mode
        .add_systems(OnEnter(SimulationMode::Visualizing), setup_current_vehicle_for_visualization)
        .add_systems(Update, (
            step_vehicle_visualization_simulation,
            camera_follow_vehicle,
            control_visualization_physics_pause,
            record_initial_chassis_position,
            distance_comparison_ui,
        ).run_if(in_state(SimulationMode::Visualizing)))
        .add_systems(OnExit(SimulationMode::Visualizing), (
            despawn_vehicle_parts,
            ensure_physics_active_on_exit_visualization,
        ))
        
        // UI System (placeholder)
        .add_systems(Update, ui_system_info_panel)
        .add_systems(Update, apply_simulation_speed) // System to apply speed to Time<Virtual>
        
        .run();
}

fn setup_ground_visualization(mut commands: Commands, config: Res<SimulationConfig>) {
    // Create a more noticeable ground that matches the screenshot
    commands.spawn((
        SpriteBundle {
            sprite: Sprite {
                color: Color::rgb(0.3, 0.6, 0.3),
                custom_size: Some(Vec2::new(4000.0, 1000.0)),
                ..default()
            },
            // This transform will be used by the RigidBody as well.
            // To make top surface at Y = -0.15, and sprite/collider half-height is 500.0,
            // center must be at Y = -0.15 - 500.0 = -500.15.
            // The Z is for visual layering; physics is 2D.
            transform: Transform::from_xyz(0.0, -500.15, -0.1),
            ..default()
        },
        RigidBody::Fixed,
        Collider::cuboid(2000.0, 500.0), // Half-height is 500.0, matches sprite half-height
        Friction::coefficient(config.ground_friction),
        Restitution::coefficient(0.0),
        CollisionGroups::new(Group::from_bits_truncate(GROUP_GROUND), Group::from_bits_truncate(GROUND_FILTER)),
        Name::new("Ground"),
    ));
    println!("Ground created with friction coefficient: {}", config.ground_friction);
}

fn keyboard_controls(
    mut next_sim_mode: ResMut<NextState<SimulationMode>>,
    current_sim_mode: Res<State<SimulationMode>>,
    keyboard_input: Res<Input<KeyCode>>,
    mut evo_res: ResMut<EvoResource>,
    config: Res<SimulationConfig>,
    mut vis_chromosome: ResMut<CurrentVehicleChromosome>,
    mut vis_state: ResMut<VisualizationState>,
    mut batch_run_config: ResMut<BatchRunConfig>,
) {
    let shift_pressed = keyboard_input.pressed(KeyCode::ShiftLeft) || keyboard_input.pressed(KeyCode::ShiftRight);

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

    if keyboard_input.just_pressed(KeyCode::V) {
        if shift_pressed { // Shift + V: Load best of current gen and PAUSE
            if *current_sim_mode.get() == SimulationMode::Paused {
                if let Some(best_current) = evo_res.1.population.individuals.first() {
                    println!("Shift+V: Loading best of current generation (ID: {}) and PAUSING physics.", best_current.id);
                    vis_chromosome.0 = Some(best_current.chromosome.clone());
                    vis_state.is_simulating_current_vehicle = true;
                    vis_state.current_simulation_time = 0.0;
                    vis_state.is_physics_paused = true; // Explicitly pause physics
                    vis_state.single_step_requested = false; // Clear any pending step
                    vis_state.use_debug_vehicle = false; // Ensure debug vehicle is off
                    next_sim_mode.set(SimulationMode::Visualizing);
                } else {
                    println!("Shift+V: No individuals in population to visualize.");
                }
            } else {
                println!("Shift+V: Can only load paused visualization from main Paused mode.");
            }
        } else { // Just V: Visualize best of current generation (and play)
            if *current_sim_mode.get() != SimulationMode::Visualizing {
                if let Some(best_current) = evo_res.1.population.individuals.first() {
                    println!("V: Preparing to visualize best of current generation (ID: {}).", best_current.id);
                    vis_chromosome.0 = Some(best_current.chromosome.clone());
                    vis_state.is_simulating_current_vehicle = true;
                    vis_state.current_simulation_time = 0.0;
                    vis_state.use_debug_vehicle = false; // Ensure debug vehicle is off
                    // vis_state.is_physics_paused will be false by default via setup_current_vehicle_for_visualization
                    next_sim_mode.set(SimulationMode::Visualizing);
                } else {
                    println!("V: No individuals in population to visualize.");
                }
            } else {
                 println!("V: Switching to Paused mode from Visualizing.");
                next_sim_mode.set(SimulationMode::Paused);
            }
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
            _ => { // Includes Visualizing and RunningFast
                println!("Pausing simulation via Space bar (next_sim_mode set to Paused).");
                // If in visualizing, this will exit visualizing. If user wants to pause *within* visualizing, use K.
                next_sim_mode.set(SimulationMode::Paused);
            }
        }
    }

    if keyboard_input.just_pressed(KeyCode::K) { // New keybind for pausing visualization physics
        if *current_sim_mode.get() == SimulationMode::Visualizing {
            vis_state.is_physics_paused = !vis_state.is_physics_paused;
            if vis_state.is_physics_paused {
                println!("Visualization physics PAUSED.");
            } else {
                println!("Visualization physics RESUMED.");
                vis_state.single_step_requested = false; // Ensure no pending single step when resuming play
            }
        }
    }

    if keyboard_input.just_pressed(KeyCode::N) { // New keybind for single physics step
        if *current_sim_mode.get() == SimulationMode::Visualizing && vis_state.is_physics_paused {
            vis_state.single_step_requested = true;
            println!("Visualization physics: Single step requested.");
        } else if *current_sim_mode.get() == SimulationMode::Visualizing && !vis_state.is_physics_paused {
            println!("Press 'K' to pause visualization physics before single stepping with 'N'.");
        } else {
            // N does nothing if not in visualizing mode
        }
    }

    if keyboard_input.just_pressed(KeyCode::D) { // Keybind for Debug Vehicle Visualization
        if *current_sim_mode.get() == SimulationMode::Visualizing && vis_state.use_debug_vehicle {
            // If already visualizing debug vehicle, switch to Paused mode
            println!("D: Exiting debug vehicle visualization. Switching to Paused mode.");
            vis_state.use_debug_vehicle = false;
            vis_state.is_simulating_current_vehicle = false; // Ensure regular simulation stops
            next_sim_mode.set(SimulationMode::Paused);
        } else if *current_sim_mode.get() == SimulationMode::Paused { // Allow starting debug from Paused
            // If paused, switch to visualizing debug vehicle
            println!("D: Enabling debug vehicle visualization. STARTING PAUSED - PRESS 'N' TO STEP.");
            vis_chromosome.0 = None; // Clear any selected chromosome
            vis_state.use_debug_vehicle = true;
            vis_state.is_simulating_current_vehicle = true; // Enable simulation logic for visualization
            vis_state.current_simulation_time = 0.0;
            vis_state.is_physics_paused = true; // Start paused
            vis_state.single_step_requested = false;
            next_sim_mode.set(SimulationMode::Visualizing);
        } else if *current_sim_mode.get() == SimulationMode::Visualizing && !vis_state.use_debug_vehicle {
            // If in normal visualization, switch to debug visualization (paused)
            println!("D: Switching from normal to DEBUG vehicle visualization. STARTING PAUSED - PRESS 'N' TO STEP.");
            vis_chromosome.0 = None; // Clear any selected chromosome
            vis_state.use_debug_vehicle = true;
            vis_state.is_simulating_current_vehicle = true; // Re-ensure this, though should be true
            vis_state.current_simulation_time = 0.0; // Reset time
            vis_state.is_physics_paused = true; // Pause physics
            vis_state.single_step_requested = false;
            // next_sim_mode.set(SimulationMode::Visualizing); // Already in this mode, but ensures OnEnter logic if it were different
        } else {
            println!("D: Press D from Paused mode to start debug vehicle, or again if already in debug viz to exit. Or from normal viz to switch to debug viz.");
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
    mut rapier_config: ResMut<RapierConfiguration>,
    mut vis_state: ResMut<VisualizationState>,
    evo_res: Res<EvoResource>,
) {
    vis_state.current_simulation_time = 0.0; // Ensure sim time is reset for all visualizations
    // Reset visualization-specific pause state (is_physics_paused is handled by keyboard_controls for debug mode)
    if !vis_state.use_debug_vehicle {
        vis_state.is_physics_paused = false; 
    }
    // Ensure physics pipeline is active when starting a new visualization
    rapier_config.physics_pipeline_active = true;

    // Reset the initial chassis position - will be set after spawning
    vis_state.initial_chassis_position = None;
    
    // Set the expected fitness value from the individual being visualized if available
    if let Some(chromosome) = &vis_chromosome.0 {
        // Find the individual with this chromosome to get its fitness
        vis_state.expected_fitness = None; // Reset first
        
        if !vis_state.use_debug_vehicle {
            // Search current generation population
            for individual in &evo_res.1.population.individuals {
                if individual.chromosome == *chromosome {
                    vis_state.expected_fitness = Some(individual.fitness);
                    println!("Expected fitness from simulation: {:.2}", individual.fitness);
                    break;
                }
            }
            
            // If not found in current gen, check all-time best
            if vis_state.expected_fitness.is_none() {
                if let Some(best) = &evo_res.1.all_time_best_individual {
                    if best.chromosome == *chromosome {
                        vis_state.expected_fitness = Some(best.fitness);
                        println!("Expected fitness from all-time best: {:.2}", best.fitness);
                    }
                }
            }
        }
    } else {
        vis_state.expected_fitness = None;
    }

    for entity in vehicle_parts_query.iter() {
        commands.entity(entity).despawn_recursive();
    }

    // Ensure gravity is set for the visual simulation
    rapier_config.gravity = Vec2::new(0.0, config.gravity);

    if vis_state.use_debug_vehicle {
        println!("Spawning DEBUG vehicle.");
        // Build a vehicle from hard-coded parameter for debugging
        let chassis_width = 2.0;
        let chassis_height = 1.0;
        let chassis_half_height = chassis_height / 2.0;
        let debug_wheel_radius = 0.4;
        let debug_wheel_density = 1.0;
        let debug_wheel_friction = 1.0;
        let debug_motor_torque = 0.00000125; // Adjusted from 0.0005 to compensate for scaling changes

        // Lower the chassis so the wheel can make contact with the ground
        let initial_chassis_x = 0.0;  // Center horizontally
        let initial_chassis_y = 0.2 + chassis_half_height;  // Just above the ground to ensure wheel contact
        
        // Position the wheel on the left side, closer to the ground
        let wheel_x_rel_to_chassis_center = -0.6; // Left side of chassis
        
        let debug_wheels_data = vec![
            wheel_x_rel_to_chassis_center,  // Left wheel position
        ];

        // Create chassis
        let chassis_entity = commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.7, 0.7, 0.8),
                    custom_size: Some(Vec2::new(chassis_width, chassis_height)),
                    ..default()
                },
                transform: Transform::from_xyz(initial_chassis_x, initial_chassis_y, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(chassis_width / 2.0, chassis_half_height),
            ColliderMassProperties::Density(1.0),
            Friction::coefficient(0.7),
            // Stronger damping for stability
            Damping { linear_damping: 5.0, angular_damping: 5.0 },
            // Add velocity caps for stability
            Velocity { linvel: Vec2::ZERO, angvel: 0.0 },
            // Add velocity limits to prevent explosion
            Ccd::enabled(), // Enable Continuous Collision Detection for stability
            CollisionGroups::new(Group::from_bits_truncate(GROUP_VEHICLE), Group::from_bits_truncate(VEHICLE_FILTER)),
            ActiveEvents::COLLISION_EVENTS,
            Name::new("DebugChassis"),
            VehiclePart,
        )).id();

        // Create the debug wheel and attach with a revolute joint
        for &wheel_x_rel_to_chassis_center in &debug_wheels_data {
            let wheel_color = Color::rgb(0.5, 0.5, 0.5); // Neutral color for zero torque

            // Calculate wheel position to be touching the ground
            // Position the bottom of the wheel slightly below the ground level
            let wheel_world_x = initial_chassis_x + wheel_x_rel_to_chassis_center;
            let wheel_world_y = initial_chassis_y - chassis_half_height - debug_wheel_radius; // Wheel center Y to match joint setup
            
            println!("Spawning wheel at position: ({}, {})", wheel_world_x, wheel_world_y);

            let wheel_entity = commands.spawn((
                SpriteBundle {
                    sprite: Sprite {
                        color: wheel_color,
                        custom_size: Some(Vec2::new(debug_wheel_radius * 2.0, debug_wheel_radius * 2.0)),
                        ..default()
                    },
                    transform: Transform::from_xyz(wheel_world_x, wheel_world_y, 0.1),
                    ..default()
                },
                RigidBody::Dynamic,
                Collider::ball(debug_wheel_radius),
                ColliderMassProperties::Density(debug_wheel_density),
                Friction::coefficient(debug_wheel_friction),
                // Stronger damping for the wheel
                Damping { linear_damping: 5.0, angular_damping: 5.0 },
                // Add velocity caps for stability
                Velocity { linvel: Vec2::ZERO, angvel: 0.0 },
                // Add velocity limits to prevent explosion
                Ccd::enabled(), // Enable CCD for wheel too
                CollisionGroups::new(Group::from_bits_truncate(GROUP_VEHICLE), Group::from_bits_truncate(VEHICLE_FILTER)),
                ExternalImpulse::default(),
                Name::new("DebugWheel"),
                VehiclePart,
                VisualizedWheel { motor_gene_torque: debug_motor_torque },
            )).id();

            // Use a revolute joint to allow wheel rotation
            let revolute_joint = RevoluteJointBuilder::new()
                .local_anchor1(Vec2::new(wheel_x_rel_to_chassis_center, -chassis_half_height - debug_wheel_radius))
                .local_anchor2(Vec2::ZERO)
                .motor_model(MotorModel::ForceBased)
                .motor_velocity(if debug_motor_torque > 0.0 { f32::MAX } else { f32::MIN }, 0.0)
                .motor_max_force(debug_motor_torque.abs());
                
            commands.entity(chassis_entity).insert(ImpulseJoint::new(wheel_entity, revolute_joint.build()));
            
            // Add a visual indicator of wheel rotation (a line from center to edge)
            commands.entity(wheel_entity).with_children(|parent| {
                parent.spawn((
                    SpriteBundle {
                        sprite: Sprite {
                            color: Color::BLACK,
                            custom_size: Some(Vec2::new(debug_wheel_radius, debug_wheel_radius * 0.15)),
                            anchor: bevy::sprite::Anchor::CenterLeft,
                            ..default()
                        },
                        transform: Transform::from_xyz(0.0, 0.0, 0.01),
                        ..default()
                    },
                    Name::new("WheelRotationIndicator"),
                ));
            });
        }
        println!("Spawned DEBUG vehicle with non-overlapping wheel placement.");

    } else if let Some(chromosome) = &vis_chromosome.0 { // Original logic for chromosome-based vehicle
        println!("Spawning vehicle from chromosome.");
        let chassis_genes = &chromosome.chassis;
        let initial_chassis_x = 0.0;
        let initial_chassis_y = config.initial_height_above_ground + chassis_genes.height / 2.0;

        let chassis_entity = commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.7, 0.7, 0.8),
                    custom_size: Some(Vec2::new(chassis_genes.width, chassis_genes.height)),
                    ..default()
                },
                transform: Transform::from_xyz(initial_chassis_x, initial_chassis_y, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(chassis_genes.width / 2.0, chassis_genes.height / 2.0),
            ColliderMassProperties::Density(chassis_genes.density),
            Friction::coefficient(0.7), 
            Damping { linear_damping: 5.0, angular_damping: 5.0 },
            CollisionGroups::new(Group::from_bits_truncate(GROUP_VEHICLE), Group::from_bits_truncate(VEHICLE_FILTER)),
            ActiveEvents::COLLISION_EVENTS, 
            Name::new("Chassis"),
            Ccd::enabled(), // Enable CCD for chassis
            VehiclePart,
        )).id();

        for wheel_gene in &chromosome.wheels {
            if wheel_gene.active {
                let wheel_x_abs = wheel_gene.get_x_position(chassis_genes.width);
                let wheel_y_abs = wheel_gene.get_y_position(chassis_genes.height);
                
                let wheel_color = if wheel_gene.motor_torque > 0.0 { Color::rgb(0.8, 0.3, 0.3) } 
                                  else if wheel_gene.motor_torque < 0.0 { Color::rgb(0.3, 0.3, 0.8) } 
                                  else { Color::rgb(0.5, 0.5, 0.5) };

                let wheel_entity = commands.spawn((
                    SpriteBundle {
                        sprite: Sprite {
                            color: wheel_color,
                            custom_size: Some(Vec2::new(wheel_gene.radius * 2.0, wheel_gene.radius * 2.0)),
                            ..default()
                        },
                        transform: Transform::from_xyz(initial_chassis_x + wheel_x_abs, initial_chassis_y + wheel_y_abs, 0.1),
                        ..default()
                    },
                    RigidBody::Dynamic,
                    Collider::ball(wheel_gene.radius),
                    ColliderMassProperties::Density(wheel_gene.density), // Added density for gene wheels too
                    Friction::coefficient(wheel_gene.friction_coefficient),
                    Restitution::coefficient(0.1),
                    Damping { linear_damping: 5.0, angular_damping: 5.0 },
                    CollisionGroups::new(Group::from_bits_truncate(GROUP_VEHICLE), Group::from_bits_truncate(VEHICLE_FILTER)),
                    ExternalImpulse::default(), 
                    Name::new("Wheel"),
                    Ccd::enabled(), // Enable CCD for wheels
                    VehiclePart,
                    VisualizedWheel { motor_gene_torque: wheel_gene.motor_torque },
                )).id();
                
                let mut joint_builder = RevoluteJointBuilder::new()
                    .local_anchor1(Vec2::new(wheel_x_abs, wheel_y_abs))
                    .local_anchor2(Vec2::new(0.0, 0.0));
                
                if wheel_gene.motor_torque != 0.0 {
                    joint_builder = joint_builder
                        .motor_model(MotorModel::ForceBased)
                        .motor_velocity(if wheel_gene.motor_torque > 0.0 { f32::MAX } else { f32::MIN }, 0.0)
                        .motor_max_force(wheel_gene.motor_torque.abs());
                }
                
                commands.entity(chassis_entity).insert(ImpulseJoint::new(wheel_entity, joint_builder.build()));
                
                commands.entity(wheel_entity).with_children(|parent| {
                    parent.spawn((
                        SpriteBundle {
                            sprite: Sprite {
                                color: Color::BLACK,
                                custom_size: Some(Vec2::new(wheel_gene.radius, wheel_gene.radius * 0.15)),
                                anchor: bevy::sprite::Anchor::CenterLeft,
                                ..default()
                            },
                            transform: Transform::from_xyz(0.0, 0.0, 0.01), 
                            ..default()
                        },
                        Name::new("WheelRotationIndicator"),
                    ));
                });
            }
        }
        println!("Spawned full vehicle for visualization with proper torque handling and improved visibility.");
    } else {
        if !vis_state.use_debug_vehicle { // Only print if not in debug mode expecting no chromosome
            println!("No chromosome to visualize and not in debug mode.");
        }
    }
}

// System to apply simulation speed
fn apply_simulation_speed(sim_speed: Res<SimulationSpeed>, mut time: ResMut<Time<Virtual>>) {
    time.set_relative_speed(sim_speed.0);
}

// System to control physics pause during visualization
fn control_visualization_physics_pause(
    mut vis_state: ResMut<VisualizationState>,
    mut rapier_config: ResMut<RapierConfiguration>,
    mut clear_color: ResMut<ClearColor>,
) {
    clear_color.0 = Color::rgb(0.1, 0.1, 0.15);
    
    if vis_state.is_physics_paused {
        if vis_state.single_step_requested {
            // Activate physics for this frame only
            rapier_config.physics_pipeline_active = true;
            
            // Clear the flag after activating physics for one frame
            vis_state.single_step_requested = false;
            
            println!("Physics step executed");
        } else {
            // Keep physics inactive when paused and not stepping
            rapier_config.physics_pipeline_active = false;
        }
    } else {
        // Physics always active when not paused
        rapier_config.physics_pipeline_active = true;
    }
}

// System to ensure physics is active when exiting visualization mode
fn ensure_physics_active_on_exit_visualization(mut rapier_config: ResMut<RapierConfiguration>) {
    rapier_config.physics_pipeline_active = true;
    println!("Ensured Rapier physics is active upon exiting visualization mode.");
}

// A simple UI panel to display info (requires a Bevy UI library like bevy_egui or native Bevy UI)
fn ui_system_info_panel(
    mut contexts: EguiContexts,
    mut evo_res: ResMut<EvoResource>,
    sim_mode: Res<State<SimulationMode>>,
    config: Res<SimulationConfig>,
    mut vis_state: ResMut<VisualizationState>,
    mut vis_chromosome: ResMut<CurrentVehicleChromosome>,
    mut next_sim_mode: ResMut<NextState<SimulationMode>>,
    mut sim_speed: ResMut<SimulationSpeed>,
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
        
        // Add visualization time display and auto-stop control
        if *sim_mode.get() == SimulationMode::Visualizing {
            ui.label(format!("Visualization Time: {:.2}/{:.2}s", vis_state.current_simulation_time, config.sim_duration_secs));
            
            if ui.checkbox(&mut vis_state.auto_stop_visualization, "Auto-stop at Simulation Duration").clicked() {
                println!("Auto-stop visualization setting changed to: {}", vis_state.auto_stop_visualization);
            }
        }
        
        ui.separator();
        ui.heading("Controls (Keyboard):");
        ui.label("F - Toggle Fast Mode / Pause");
        ui.label("V - Visualize Best of Current Gen / Pause Visualization");
        ui.label("Shift+V - Load Best of Gen & PAUSE Visualization");
        ui.label("B - Visualize All-Time Best / Pause Visualization");
        ui.label("P - Export Phylogeny (.dot file)");
        ui.label("R - Reset Simulation");
        ui.label("Space - Pause / Resume to Fast Mode");
        ui.label("K - Toggle Visualization Physics Pause");
        ui.label("N - Advance One Tick (when viz physics paused)");
        ui.label("D - Toggle Debug Vehicle Visualization");
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
                    vis_state.is_simulating_current_vehicle = true;
                    vis_state.current_simulation_time = 0.0;
                    vis_state.use_debug_vehicle = false; // Ensure debug vehicle is off
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
                    vis_state.is_simulating_current_vehicle = true;
                    vis_state.current_simulation_time = 0.0;
                    vis_state.use_debug_vehicle = false; // Ensure debug vehicle is off
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
            evo_res.1.reset();
            next_sim_mode.set(SimulationMode::Paused);
            vis_chromosome.0 = None;
            vis_state.is_simulating_current_vehicle = false;
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
        if ui.add(egui::Slider::new(&mut sim_speed.0, 0.1..=5.0).text("Speed Factor")).changed() {
        }
        ui.label(format!("Current Speed Factor: {:.2}", sim_speed.0));

    });
}

// Configure Rapier physics settings to match headless simulation
fn configure_physics(
    mut rapier_config: ResMut<RapierConfiguration>,
    config: Res<SimulationConfig>,
) {
    rapier_config.gravity = Vec2::new(0.0, config.gravity);
    
    rapier_config.physics_pipeline_active = true;

    println!("Physics configured with gravity: (0, {})", config.gravity);
}

// New system to record initial chassis position after spawning
fn record_initial_chassis_position(
    mut vis_state: ResMut<VisualizationState>,
    chassis_query: Query<(&Transform, &Name), With<VehiclePart>>,
) {
    // Only run once to capture the initial position
    if vis_state.is_simulating_current_vehicle && vis_state.initial_chassis_position.is_none() {
        for (transform, name) in chassis_query.iter() {
            if name.as_str().contains("Chassis") {
                let chassis_pos = Vec2::new(transform.translation.x, transform.translation.y);
                vis_state.initial_chassis_position = Some(chassis_pos);
                println!("Recorded initial chassis position: ({:.2}, {:.2})", chassis_pos.x, chassis_pos.y);
                break;
            }
        }
    }
}

// Update the distance comparison UI to be more prominent
fn distance_comparison_ui(
    mut contexts: EguiContexts,
    distance_data: Res<DistanceComparisonData>,
    sim_mode: Res<State<SimulationMode>>,
) {
    if *sim_mode.get() == SimulationMode::Visualizing { 
        if let (Some(simulated), Some(visualized)) = (distance_data.simulated, distance_data.visualized) {
            egui::Window::new("SIMULATION VS VISUALIZATION COMPARISON")
                .anchor(egui::Align2::CENTER_TOP, [0.0, 20.0])
                .show(contexts.ctx_mut(), |ui| {
                    ui.heading("Distance Comparison");
                    ui.add_space(5.0);
                    
                    ui.label(format!("Simulated Distance: {:.2}", simulated));
                    ui.label(format!("Visualized Distance: {:.2}", visualized));
                    
                    let delta = visualized - simulated;
                    let delta_percent = if simulated != 0.0 { (delta / simulated) * 100.0 } else { 0.0 };
                    
                    ui.separator();
                    ui.label(format!("Delta: {:.2} ({:.1}%)", delta, delta_percent));
                    
                    if delta.abs() / simulated > 0.1 && simulated > 1.0 {
                        ui.colored_label(egui::Color32::from_rgb(255, 100, 100), 
                            "⚠️ WARNING: Large discrepancy between simulation and visualization!");
                    }
                });
        }
    }
}

// System to step vehicle visualization simulation - Update this function
fn step_vehicle_visualization_simulation(
    mut vis_state: ResMut<VisualizationState>,
    config: Res<SimulationConfig>,
    time: Res<Time<Fixed>>,
    mut wheel_query: Query<(&VisualizedWheel, &mut ExternalImpulse, &Transform)>,
    chassis_query: Query<(Entity, &Transform), (With<VehiclePart>, Without<VisualizedWheel>)>, // Modified to get Entity + Transform
    all_visualized_wheels_query: Query<(Entity, &Name, &Transform), With<VisualizedWheel>>, // Added &Transform for logging
    _next_sim_mode: ResMut<NextState<SimulationMode>>,
    mut distance_data: ResMut<DistanceComparisonData>,
) {
    // Only log if physics is not paused (includes both normal running and single step)
    if vis_state.is_simulating_current_vehicle && (!vis_state.is_physics_paused || vis_state.single_step_requested) {
        println!("--- step_vehicle_visualization_simulation SYSTEM CALLED ---"); // DEBUG PRINT

        // Log all existing wheels and chassis transforms at the start of this system's execution
        println!("=== PHYSICS STATE ===");
        
        // Log chassis position
        for (entity, transform) in chassis_query.iter() {
            println!("Chassis entity {entity:?}, Transform: {}", transform.translation);
        }
        
        let collected_wheels: Vec<_> = all_visualized_wheels_query.iter().collect();
        println!("Logging all collected wheels ({}): ", collected_wheels.len());
        for (entity, name, transform) in collected_wheels.iter() {
            println!("Wheel entity {entity:?} with name: {}, Transform: {}", name, transform.translation);
        }
        println!("=== END PHYSICS STATE ===");
    }
    
    if vis_state.is_simulating_current_vehicle {
        // Only increment simulation time if physics is not paused or we're doing a single step
        if !vis_state.is_physics_paused || vis_state.single_step_requested {
            // Torque application to wheels is now handled by joint motors, so this loop is removed.
            // for (wheel, mut impulse, _transform) in wheel_query.iter_mut() {
            //     // Apply impulse scaled by time delta
            //     let scaled_torque = wheel.motor_gene_torque * time.delta_seconds();
            //     
            //     // Clear any existing impulse to prevent accumulation
            //     impulse.torque_impulse = 0.0;
            //     
            //     // Apply the torque directly - no clamping, rely on gene limits instead
            //     impulse.torque_impulse += scaled_torque;
            // }
            
            // Increment simulation time if physics is running
            vis_state.current_simulation_time += time.delta_seconds();
            
            // Check if we should auto-stop the visualization
            if vis_state.auto_stop_visualization && vis_state.current_simulation_time >= config.sim_duration_secs {
                // Stop the visualization and display comparison
                println!("AUTO-STOP: Visualization reached {} seconds (matching simulation duration)", config.sim_duration_secs);
                
                // Calculate and update distance data
                let mut chassis_pos = None;
                for (_, transform) in chassis_query.iter() {
                    chassis_pos = Some(Vec2::new(transform.translation.x, transform.translation.y));
                    break;
                }
                
                if let (Some(initial_pos), Some(current_pos), Some(expected)) = (vis_state.initial_chassis_position, chassis_pos, vis_state.expected_fitness) {
                    let visualized_distance = current_pos.x - initial_pos.x;
                    let delta = visualized_distance - expected;
                    let delta_percent = if expected != 0.0 { (delta / expected) * 100.0 } else { 0.0 };
                    
                    println!("----------------------------------------------------");
                    println!("SIMULATION VS VISUALIZATION COMPARISON");
                    println!("Simulated Distance: {:.2}", expected);
                    println!("Visualized Distance: {:.2}", visualized_distance);
                    println!("Delta: {:.2} ({:.1}%)", delta, delta_percent);
                    println!("----------------------------------------------------");
                    
                    // Update the distance comparison data resource for UI display
                    distance_data.simulated = Some(expected);
                    distance_data.visualized = Some(visualized_distance);
                }
                
                // Pause the physics but stay in visualization mode
                vis_state.is_physics_paused = true;
            }
        }
    }
} 