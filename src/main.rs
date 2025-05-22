mod organism;
mod simulation;
mod evolution;
mod visualization;
mod phylogeny;
mod physics; // New physics module for shared definitions

use bevy::{
    prelude::*,
    window::PresentMode,
};
use bevy_rapier2d::prelude::*;
use bevy_egui::{egui, EguiPlugin, EguiContexts};

use simulation::{PhysicsWorld as HeadlessPhysicsWorld, SimulationConfig};
use evolution::EvolutionEngine;
use visualization::*;
use phylogeny::write_phylogeny_to_dot_file;
use crate::organism::get_test_chromosome; // For logging test chromosome
use std::fs::File; // For logging
use std::io::Write; // For logging
use std::sync::Mutex; // For sharing File handle in resource
use physics::{PhysicsParameters, GroundDefinition, VehicleDefinition};
use std::env; // Added for command-line arguments

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

// Resource to manage logging state for the test chromosome
#[derive(Resource)]
struct TestChromosomeLogState {
    is_logging: bool,
    log_file: Option<Mutex<File>>,
    step_count: u32,
}

impl Default for TestChromosomeLogState {
    fn default() -> Self {
        Self {
            is_logging: false,
            log_file: None,
            step_count: 0,
        }
    }
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
    pub chassis_entity_for_logging: Option<Entity>, // Added to store chassis entity for logging
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
            chassis_entity_for_logging: None, // Initialize as None
        }
    }
}

// Add the struct definition for PhysicsSteps
#[derive(Resource, Default)]
struct PhysicsSteps(u32);

// New system to stabilize the simulation with pre-steps when a vehicle is spawned
fn run_simulation_presteps(
    vis_state: Res<VisualizationState>,
    mut physics_steps: ResMut<PhysicsSteps>, // Changed from mut commands: Commands
) {
    // Only run when we're actively simulating a vehicle and in the first frame
    if vis_state.is_simulating_current_vehicle && vis_state.current_simulation_time < 0.1 {
        // Force joint stabilization through Bevy Rapier ECS
        // println!("Requesting 2 additional physics pre-steps for stabilization."); // COMMENTED OUT
        // physics_steps.0 += 2; // COMMENTED OUT // Request 2 additional steps (total 3 with the normal step)
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.contains(&String::from("--test-headless-eval")) {
        println!("[Focused Test] Running single headless evaluation...");

        // Ensure log file is clean for this focused test
        if std::fs::remove_file("headless_physics_log.txt").is_ok() {
            println!("[Focused Test] Removed existing headless_physics_log.txt");
        }

        let sim_config = SimulationConfig::default();
        let mut headless_physics_world = HeadlessPhysicsWorld::new(&sim_config);
        let test_chromosome = get_test_chromosome();

        println!("[Focused Test] Evaluating chromosome: {:#?}", test_chromosome);
        let fitness = headless_physics_world.evaluate_fitness(&test_chromosome, &sim_config);
        println!("[Focused Test] Fitness calculated: {}", fitness);
        println!("[Focused Test] Check headless_physics_log.txt for detailed physics state.");
        return; // Exit after the focused test
    }

    println!("Vehicle Evolution Simulator Initializing...");

    let sim_config = SimulationConfig::default(); 
    let headless_physics_world = HeadlessPhysicsWorld::new(&sim_config);
    let evolution_engine = EvolutionEngine::new(sim_config.clone());

    // Calculate the initial chromosome once
    let initial_chromosome = get_test_chromosome(); 

    App::new()
        .insert_resource(ClearColor(Color::rgb(0.1, 0.1, 0.15)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Vehicle Evolution Simulator".into(),
                resolution: (1280.0, 720.0).into(),
                present_mode: PresentMode::AutoVsync,
                ..default()
            }),
            ..default()
        }))
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(1.0))
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(EguiPlugin)
        
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
        .init_resource::<TestChromosomeLogState>() // Initialize logging state
        .insert_resource(PhysicsSteps::default())

        // Configure physics to use fixed timestep for better stability
        .insert_resource(Time::<Fixed>::from_seconds(1.0 / 60.0))

        // Startup Systems
        .add_systems(Startup, (
            setup_graphics,
            setup_ground_visualization, // Add a visual ground
            configure_physics,
            configure_rapier_params, // Add the new system
        ))

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
        
        // Add our stabilization pre-step system
        .add_systems(FixedUpdate, run_simulation_presteps)
        
        // Add the new logging system
        .add_systems(
            FixedUpdate,
            (
                handle_test_chassis_logging,
            ),
        )
        
        .run();
}

fn setup_ground_visualization(mut commands: Commands, config: Res<SimulationConfig>) {
    // Create ground using shared physics module
    let physics_params = PhysicsParameters {
        gravity: config.gravity,
        ground_friction: config.ground_friction,
        ground_restitution: 0.0,
        erp: 0.01, // Unified value from testing
        joint_erp: 0.01, // Unified value from testing
        solver_iterations: 4,
    };
    
    let ground_def = GroundDefinition::new(&physics_params);
    ground_def.spawn_for_bevy(&mut commands);
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
    mut test_log_state: ResMut<TestChromosomeLogState>, // Added for logging
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
    mut vis_state: ResMut<VisualizationState>,
    current_vehicle: Res<CurrentVehicleChromosome>,
    config: Res<SimulationConfig>,
    mut rapier_context: ResMut<RapierContext>,
    mut test_log_state: ResMut<TestChromosomeLogState>, // Re-add for logging
) {
    // Set the same ERP value used in the headless simulation for consistency
    rapier_context.integration_parameters.erp = 0.01;
    rapier_context.integration_parameters.joint_erp = 0.01;
    
    // Reset visualization specific state
    vis_state.current_simulation_time = 0.0;
    vis_state.is_physics_paused = false;
    
    // Reset logging state
    test_log_state.is_logging = false;
    test_log_state.log_file = None;
    test_log_state.step_count = 0;
    
    if let Some(chromosome) = &current_vehicle.0 {
        println!("Setting up vehicle for visualization");
        
        // Check if this is the test chromosome for logging
        let test_chromosome = get_test_chromosome();
        if *chromosome == test_chromosome {
            test_log_state.is_logging = true;
            match File::create("visual_physics_log.txt") {
                Ok(file) => {
                    test_log_state.log_file = Some(Mutex::new(file));
                    if let Some(mutex_file) = &test_log_state.log_file {
                        if let Ok(mut f) = mutex_file.lock() {
                            writeln!(f, "VISUALIZATION LOG").unwrap_or_default();
                            writeln!(f, "Chassis target: width={}, height={}, density={}", 
                                chromosome.chassis.width, chromosome.chassis.height, chromosome.chassis.density).unwrap_or_default();
                            for (i, wheel) in chromosome.wheels.iter().enumerate() {
                                if wheel.active {
                                    writeln!(f, "Wheel {} target: radius={}, density={}, torque={}, friction={}", 
                                        i, wheel.radius, wheel.density, wheel.motor_torque, wheel.friction_coefficient).unwrap_or_default();
                                }
                            }
                        }
                    }
                },
                Err(e) => {
                    eprintln!("Failed to create visual_physics_log.txt: {}", e);
                    test_log_state.is_logging = false;
                }
            }
        }
        
        // Create vehicle using shared physics module
        let vehicle_def = VehicleDefinition::new(chromosome, config.initial_height_above_ground);
        
        // Store initial chassis position for distance tracking
        let (initial_x, initial_y) = vehicle_def.get_initial_chassis_position();
        vis_state.initial_chassis_position = Some(Vec2::new(initial_x, initial_y));
        
        // Spawn chassis
        let chassis_entity = vehicle_def.spawn_chassis_for_bevy(&mut commands);
        vis_state.chassis_entity_for_logging = Some(chassis_entity);
        
        // Spawn wheels
        for wheel_idx in 0..chromosome.wheels.len() {
            vehicle_def.spawn_wheel_for_bevy(&mut commands, chassis_entity, wheel_idx);
        }
        
        // Set state to simulating and reset simulation time
        vis_state.is_simulating_current_vehicle = true;
        vis_state.current_simulation_time = 0.0;
        
        // Debug message to confirm simulation start
        println!("Vehicle visualization setup complete, simulation starting");
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
    mut test_log_state: ResMut<TestChromosomeLogState>, // Added for logging
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
    // Set gravity to match SimulationConfig
    rapier_config.gravity = Vec2::new(0.0, config.gravity);
    rapier_config.physics_pipeline_active = true;
    // rapier_config.velocity_solver_iterations = 8; // REMOVED - Does not exist on this type
    
    println!("Physics configured: gravity=({}, {}), Rapier default solver iterations used", rapier_config.gravity.x, rapier_config.gravity.y);
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
    mut wheel_query: Query<(&VisualizedWheel, &mut ExternalImpulse, &Transform, &Velocity, Entity), With<VisualizedWheel>>, // Added Velocity and Entity
    chassis_query: Query<(Entity, &Transform), (With<VehiclePart>, Without<VisualizedWheel>)>, // Modified to get Entity + Transform
    _all_visualized_wheels_query: Query<(Entity, &Name, &Transform), With<VisualizedWheel>>, // Keep for now, maybe remove later if unused
    _next_sim_mode: ResMut<NextState<SimulationMode>>,
    mut distance_data: ResMut<DistanceComparisonData>,
    mut test_log_state: ResMut<TestChromosomeLogState>, // Added for logging
    // For querying body states for logging:
    chassis_logging_query: Query<(Entity, &Transform, &Velocity), (With<RigidBody>, With<ReadMassProperties>, With<Name>, With<VehiclePart>, Without<VisualizedWheel>)>, // More specific for Chassis
    // Removed wheel_logging_query as wheel_query now has Velocity and Entity
) {
    // Only log if physics is not paused (includes both normal running and single step)
    if vis_state.is_simulating_current_vehicle && (!vis_state.is_physics_paused || vis_state.single_step_requested) {
        //println!("--- step_vehicle_visualization_simulation SYSTEM CALLED ---"); // DEBUG PRINT

        // Log physics state if it's the test chromosome run
        if test_log_state.is_logging {
            let current_step_for_log = test_log_state.step_count; // Read step_count first

            if let Some(mutex_file) = &test_log_state.log_file {
                if let Ok(mut file) = mutex_file.lock() {
                    writeln!(file, "--- Step {} ---", current_step_for_log).unwrap_or_default();
                    // test_log_state.step_count += 1; // Moved increment outside lock

                    // Find and log chassis using stored Entity ID
                    let mut chassis_logged = false;
                    if let Some(chassis_entity_id) = vis_state.chassis_entity_for_logging {
                        // Try to get chassis data using a more focused query
                        if let Ok((entity, transform, velocity)) = chassis_logging_query.get(chassis_entity_id) {
                            writeln!(file, "Chassis: entity={:?}, pos=({:.4}, {:.4}), rot={:.4}, linvel=({:.4}, {:.4}), angvel={:.4}",
                                entity, transform.translation.x, transform.translation.y, transform.rotation.to_euler(EulerRot::ZYX).0,
                                velocity.linvel.x, velocity.linvel.y, velocity.angvel).unwrap_or_default();
                            chassis_logged = true;
                        } else {
                             writeln!(file, "Chassis: Entity ID {:?} NOT FOUND in chassis_logging_query", chassis_entity_id).unwrap_or_default();
                        }
                    } else {
                         writeln!(file, "Chassis: Entity ID for logging NOT SET in VisualizationState").unwrap_or_default();
                    }
                    if !chassis_logged && vis_state.chassis_entity_for_logging.is_some() { /* Redundant check, but fine */ }
                    else if !chassis_logged { writeln!(file, "Chassis: NOT FOUND (general fallback)").unwrap_or_default(); }

                    // Log wheels using the main wheel_query
                    let mut wheel_log_idx = 0;
                    for (_visual_wheel_data, _impulse, transform, velocity, entity) in wheel_query.iter() {
                        writeln!(file, "Wheel {}: entity={:?}, pos=({:.4}, {:.4}), rot={:.4}, linvel=({:.4}, {:.4}), angvel={:.4}",
                            wheel_log_idx, entity, transform.translation.x, transform.translation.y, transform.rotation.to_euler(EulerRot::ZYX).0,
                            velocity.linvel.x, velocity.linvel.y, velocity.angvel).unwrap_or_default();
                        wheel_log_idx +=1;
                    }
                }
            } else if test_log_state.is_logging { // Log file was None, but logging was intended
                eprintln!("Error: Test chromosome logging active, but log_file is None in step_vehicle_visualization_simulation.");
            }
        }
        if test_log_state.is_logging { // Increment step_count after file operations
            test_log_state.step_count += 1;
        }
    }
    
    if vis_state.is_simulating_current_vehicle {
        // Only increment simulation time if physics is not paused or we're doing a single step
        if !vis_state.is_physics_paused || vis_state.single_step_requested {
            // Torque application to wheels is now handled by joint motors in Bevy Rapier for visualized entities.
            // The VisualizedWheel component's motor_gene_torque is used when setting up the joint's motor.
            // No explicit impulse application per step is needed here for the motors to function.

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

// New system to configure Rapier parameters to match headless simulation
fn configure_rapier_params(mut rapier_context: ResMut<RapierContext>) {
    // Access the underlying Rapier configuration
    let context = rapier_context.as_mut();
    
    // Set parameters to match headless simulation
    context.integration_parameters.erp = 0.01; // Changed to 0.01
    context.integration_parameters.joint_erp = 0.01; // Changed to 0.01
    // num_solver_iterations is handled by RapierConfiguration or defaults
    
    // Log that we've configured integration parameters
    println!("Rapier integration parameters configured: erp=0.01, joint_erp=0.01");
}

fn handle_test_chassis_logging(
    mut test_log_state: ResMut<TestChromosomeLogState>,
    _rapier_context: Res<RapierContext>,
    vis_state: Res<VisualizationState>,
    chassis_query: Query<(&Transform, &Velocity), (With<VehiclePart>, Without<VisualizedWheel>)>,
    _time: Res<Time>,
) {
    if test_log_state.is_logging {
        // Increment step count
        test_log_state.step_count += 1;
        
        // Log to file
        if let Some(mutex_file) = &test_log_state.log_file {
            if let Ok(mut file) = mutex_file.lock() {
                writeln!(file, "--- Step {} ---", test_log_state.step_count - 1).unwrap_or_default();
                
                let mut found_chassis = false;
                
                // Try to find the chassis using the entity ID
                if let Some(chassis_entity) = vis_state.chassis_entity_for_logging {
                    if let Ok((transform, velocity)) = chassis_query.get(chassis_entity) {
                        let chassis_pos = transform.translation;
                        let chassis_rot = transform.rotation.to_euler(EulerRot::XYZ).2; // Extract Euler Z rotation (2D)
                        
                        // Log chassis state
                        writeln!(file, "Chassis: pos=({:.4}, {:.4}), rot={:.4}, linvel=({:.4}, {:.4}), angvel={:.4}", 
                            chassis_pos.x, chassis_pos.y, chassis_rot,
                            velocity.linvel.x, velocity.linvel.y, velocity.angvel).unwrap_or_default();
                        
                        found_chassis = true;
                    }
                }
                
                // Fallback: if we couldn't find the chassis by entity ID, try to find any chassis
                if !found_chassis {
                    for (transform, velocity) in chassis_query.iter() {
                        let chassis_pos = transform.translation;
                        let chassis_rot = transform.rotation.to_euler(EulerRot::XYZ).2; // Extract Euler Z rotation (2D)
                        
                        // Log chassis state
                        writeln!(file, "Chassis (fallback): pos=({:.4}, {:.4}), rot={:.4}, linvel=({:.4}, {:.4}), angvel={:.4}", 
                            chassis_pos.x, chassis_pos.y, chassis_rot,
                            velocity.linvel.x, velocity.linvel.y, velocity.angvel).unwrap_or_default();
                        
                        found_chassis = true;
                        break; // Only log the first chassis
                    }
                }
                
                if !found_chassis {
                    writeln!(file, "Chassis: NOT FOUND").unwrap_or_default();
                }
            }
        }
    }
} 