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

use simulation::{PhysicsWorld as HeadlessPhysicsWorld, SimulationConfig, BatchRunConfig};
use evolution::EvolutionEngine;
use visualization::*; // Import all visualization components
use phylogeny::write_phylogeny_to_dot_file;
use crate::organism::get_test_chromosome; // For logging test chromosome
use std::fs::File; // For logging
use std::io::Write; // For logging
use std::sync::Mutex; // For sharing File handle in resource
use physics::{GroundDefinition, VehicleDefinition, GROUP_VEHICLE, GROUP_GROUND, VEHICLE_FILTER, GROUND_FILTER};
use std::env; // Added for command-line arguments

// --- Collision Groups ---

// Resource to manage the main evolution engine
#[derive(Resource)]
struct EvoResource(HeadlessPhysicsWorld, EvolutionEngine);

// Resource to manage batch running of generations
// REMOVED: BatchRunConfig definition - now imported from simulation.rs

// Resource to manage logging state for the test chromosome
#[derive(Resource)]
struct TestChromosomeLogState {
    is_logging: bool,
    log_file: Option<Mutex<File>>, // For regular physics state
    debug_log_file: Option<Mutex<File>>, // For detailed query debugging
    step_count: u32,
}

impl Default for TestChromosomeLogState {
    fn default() -> Self {
        Self {
            is_logging: false,
            log_file: None,
            debug_log_file: None, // Initialize as None
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
// (Using VisualizationState from visualization.rs)

// Add the struct definition for PhysicsSteps
#[derive(Resource, Default)]
struct PhysicsSteps(u32);

// New system to stabilize the simulation with pre-steps when a vehicle is spawned
fn run_simulation_presteps(
    vis_state: Res<VisualizationState>,
    _physics_steps: ResMut<PhysicsSteps>, // MODIFIED: Prefixed with _ as it's unused now
) {
    // Only run when we're actively simulating a vehicle and in the first frame
    if vis_state.is_simulating_current_vehicle && vis_state.current_simulation_time < 0.1 {
        // Force joint stabilization through Bevy Rapier ECS
        // println!("Requesting 2 additional physics pre-steps for stabilization."); // COMMENTED OUT
        // physics_steps.0 += 2; // COMMENTED OUT // Request 2 additional steps (total 3 with the normal step)
    }
}

// DIAGNOSTIC: System to check if Velocity components are present and have values
fn diagnostic_check_velocity_components(
    chassis_query: Query<(Entity, &Name, Option<&Velocity>, &Transform), With<VehiclePart>>,
    vis_state: Res<VisualizationState>,
) {
    // Only run diagnostics for first few seconds
    if !vis_state.is_simulating_current_vehicle || vis_state.current_simulation_time > 0.2 {
        return;
    }
    
    println!("[DIAGNOSTIC VELOCITY CHECK] Time: {:.3}s", vis_state.current_simulation_time);
    
    for (entity, name, velocity, transform) in chassis_query.iter() {
        match velocity {
            Some(vel) => {
                println!("  Entity {:?} '{}': HAS Velocity - linvel=({:.4}, {:.4}), angvel={:.4}, pos=({:.2}, {:.2})",
                    entity, name.as_str(), vel.linvel.x, vel.linvel.y, vel.angvel,
                    transform.translation.x, transform.translation.y);
            }
            None => {
                println!("  Entity {:?} '{}': NO Velocity component! pos=({:.2}, {:.2})",
                    entity, name.as_str(), transform.translation.x, transform.translation.y);
            }
        }
    }
}

// DIAGNOSTIC: System to monitor ground collision and check entities
fn diagnostic_monitor_ground_collision(
    ground_query: Query<(Entity, &Name, &Transform, &CollisionGroups), With<Collider>>,
    vehicle_query: Query<(Entity, &Name, &Transform, &CollisionGroups), (With<VehiclePart>, With<Collider>)>,
    vis_state: Res<VisualizationState>,
) {
    // Only run once early in simulation
    if !vis_state.is_simulating_current_vehicle || vis_state.current_simulation_time > 0.1 {
        return;
    }
    
    println!("[DIAGNOSTIC COLLISION CHECK] Time: {:.3}s", vis_state.current_simulation_time);
    
    // Check ground entity
    let mut ground_found = false;
    let mut ground_groups: Option<&CollisionGroups> = None;
    for (entity, name, transform, groups) in ground_query.iter() {
        if name.as_str() == "Ground" {
            ground_found = true;
            ground_groups = Some(groups);
            println!("  Ground entity {:?}: pos.y={:.2}, memberships={:?}, filters={:?}", 
                entity, transform.translation.y, groups.memberships, groups.filters);
            break;
        }
    }
    
    if !ground_found {
        println!("  WARNING: Ground entity NOT FOUND!");
        return;
    }
    
    // Check vehicle collision groups and compatibility
    println!("  Vehicle parts:");
    for (entity, name, transform, groups) in vehicle_query.iter() {
        println!("    {:?} '{}': pos.y={:.2}, memberships={:?}, filters={:?}",
            entity, name.as_str(), transform.translation.y, 
            groups.memberships, groups.filters);
            
        // Check collision compatibility
        if let Some(ground_groups) = ground_groups {
            let vehicle_can_collide_with_ground = groups.filters.intersects(ground_groups.memberships);
            let ground_can_collide_with_vehicle = ground_groups.filters.intersects(groups.memberships);
            let collision_possible = vehicle_can_collide_with_ground && ground_can_collide_with_vehicle;
            
            println!("      Collision analysis: vehicle->ground={}, ground->vehicle={}, possible={}",
                vehicle_can_collide_with_ground, ground_can_collide_with_vehicle, collision_possible);
                
            if !collision_possible {
                println!("      ❌ COLLISION GROUPS INCOMPATIBLE!");
            }
        }
            
        // Check if falling through ground
        if transform.translation.y < -1.0 {
            println!("    ⚠️ WARNING: Entity is below ground level!");
        }
    }
    
    // Log expected collision group values
    println!("  Expected collision groups:");
    println!("    GROUP_VEHICLE={}, VEHICLE_FILTER={}", physics::GROUP_VEHICLE, physics::VEHICLE_FILTER);
    println!("    GROUP_GROUND={}, GROUND_FILTER={}", physics::GROUP_GROUND, physics::GROUND_FILTER);
}

// DIAGNOSTIC: System to verify ground entity exists after startup
fn diagnostic_verify_ground_entity(
    ground_query: Query<(Entity, &Name, &Transform, &CollisionGroups, &Collider), With<Collider>>,
) {
    println!("[DIAGNOSTIC GROUND VERIFICATION] Checking for ground entity...");
    
    let mut ground_found = false;
    for (entity, name, transform, groups, collider) in ground_query.iter() {
        if name.as_str() == "Ground" {
            ground_found = true;
            println!("  ✓ Ground entity {:?} found!", entity);
            println!("    Position: ({:.3}, {:.3}, {:.3})", 
                transform.translation.x, transform.translation.y, transform.translation.z);
            println!("    Collision groups: memberships={:?}, filters={:?}", 
                groups.memberships, groups.filters);
            println!("    Collider type: {:?}", collider);
            break;
        }
    }
    
    if !ground_found {
        println!("  ❌ CRITICAL: Ground entity NOT FOUND!");
        println!("  Available entities:");
        for (entity, name, _, _, _) in ground_query.iter().take(5) {
            println!("    {:?}: '{}'", entity, name.as_str());
        }
    }
}

// DIAGNOSTIC: Add system to detect ground penetration
fn diagnostic_detect_ground_penetration(
    vehicle_query: Query<(Entity, &Name, &Transform), (With<VehiclePart>, With<Collider>)>,
    vis_state: Res<VisualizationState>,
) {
    if !vis_state.is_simulating_current_vehicle {
        return;
    }
    
    // Check for ground penetration throughout the simulation
    for (entity, name, transform) in vehicle_query.iter() {
        let y_pos = transform.translation.y;
        
        // Ground surface is at -0.15, so anything below -0.2 is definitely through the ground
        if y_pos < -0.2 {
            println!("🚨 [GROUND PENETRATION] Time: {:.3}s - Entity {:?} '{}' penetrated ground: pos.y={:.2}", 
                vis_state.current_simulation_time, entity, name.as_str(), y_pos);
        }
        
        // Also check for the first time any entity goes below ground surface
        if y_pos < -0.15 && y_pos > -0.2 {
            println!("⚠️ [GROUND CONTACT] Time: {:.3}s - Entity {:?} '{}' at ground level: pos.y={:.2}", 
                vis_state.current_simulation_time, entity, name.as_str(), y_pos);
        }
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
        let fitness = headless_physics_world.evaluate_fitness(&test_chromosome, &sim_config, true);
        println!("[Focused Test] Fitness calculated: {}", fitness);
        println!("[Focused Test] Check headless_physics_log.txt for detailed physics state.");
        return; // Exit after the focused test
    }

    if args.len() > 1 {
        match args[1].as_str() {
            "--compare-modes" => {
                println!("[Comparison Test] Running both headless and visualization with same chromosome...");
                let test_chromosome = get_test_chromosome();
                println!("[Comparison Test] Test chromosome: {:#?}", test_chromosome);
                
                // Run headless first
                println!("[Comparison Test] Running headless simulation...");
                let config = SimulationConfig::default();
                let mut physics_world = HeadlessPhysicsWorld::new(&config);
                let headless_fitness = physics_world.evaluate_fitness(&test_chromosome, &config, true);
                println!("[Comparison Test] Headless fitness: {}", headless_fitness);
                
                // Now run visualization mode with the same chromosome
                println!("[Comparison Test] Starting visualization mode...");
                println!("[Comparison Test] Press ESC to exit after observing the vehicle behavior");
                // Fall through to normal visualization mode
            },
            _ => {
                println!("Unknown argument: {}", args[1]);
                println!("Available options:");
                println!("  --test-headless-eval  : Run headless evaluation test");
                println!("  --compare-modes       : Compare headless vs visualization");
                return;
            }
        }
    }

    println!("Vehicle Evolution Simulator Initializing...");

    let sim_config = SimulationConfig::default(); 
    let headless_physics_world = HeadlessPhysicsWorld::new(&sim_config);
    let evolution_engine = EvolutionEngine::new(sim_config.clone());

    // Calculate the initial chromosome once
    let _initial_chromosome = get_test_chromosome(); // MODIFIED: Prefixed with _
    
    // Check if we're in comparison mode and set up test chromosome
    let comparison_mode = args.len() > 1 && args[1] == "--compare-modes";
    let initial_chromosome = if comparison_mode {
        Some(get_test_chromosome())
    } else {
        None
    };

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
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
        .add_plugins(RapierDebugRenderPlugin::default())
        .add_plugins(EguiPlugin)
        
        // Initialize States
        .add_state::<SimulationMode>()

        // Resources
        .insert_resource(sim_config) // SimulationConfig is also a Bevy resource
        .insert_resource(EvoResource(headless_physics_world, evolution_engine))
        .insert_resource(CurrentVehicleChromosome(initial_chromosome)) // Set test chromosome if in comparison mode
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
        
        // DIAGNOSTIC: Add system to check Velocity components after physics sync
        .add_systems(PostUpdate, diagnostic_check_velocity_components.run_if(in_state(SimulationMode::Visualizing)))
        
        // DIAGNOSTIC: Add system to monitor ground collisions
        .add_systems(Update, diagnostic_monitor_ground_collision.run_if(in_state(SimulationMode::Visualizing)))
        
        // DIAGNOSTIC: Add system to verify ground entity after startup
        .add_systems(PostStartup, diagnostic_verify_ground_entity)
        
        // DIAGNOSTIC: Add system to detect ground penetration
        .add_systems(Update, diagnostic_detect_ground_penetration.run_if(in_state(SimulationMode::Visualizing)))
        
        // Add the new logging system - moved to Update to ensure entity commands are flushed
        .add_systems(
            Update,
            (
                handle_test_chassis_logging,
            ).run_if(in_state(SimulationMode::Visualizing)),
        )
        
        .run();
}

fn setup_ground_visualization(mut commands: Commands, config: Res<SimulationConfig>) {
    // Create ground using shared physics module directly from SimulationConfig
    let ground_def = GroundDefinition::new(&config);
    println!("[Ground Setup] Creating ground: center_y={:.3}, thickness={:.3}, friction={:.3}, restitution={:.3}", 
             ground_def.get_ground_center_y(), ground_def.half_thickness * 2.0, ground_def.friction, ground_def.restitution);
    
    // DIAGNOSTIC: Log detailed ground configuration before spawning
    println!("[DIAGNOSTIC GROUND] Ground config: half_width={}, half_thickness={}, y_position={}, center_y={}", 
        ground_def.half_width, ground_def.half_thickness, ground_def.y_position, ground_def.get_ground_center_y());
    println!("[DIAGNOSTIC GROUND] Collision groups: GROUP_GROUND={}, GROUND_FILTER={}", GROUP_GROUND, GROUND_FILTER);
    
    ground_def.spawn_for_bevy(&mut commands);
    
    println!("[Ground Setup] Ground spawned with collision groups: GROUP_GROUND={}, GROUND_FILTER={}", 
             GROUP_GROUND, GROUND_FILTER);
    
    // DIAGNOSTIC: Verify ground constants match headless mode
    println!("[DIAGNOSTIC GROUND] Ground constants: GROUND_Y_POSITION={}, GROUND_THICKNESS={}", 
        physics::GROUND_Y_POSITION, physics::GROUND_THICKNESS);
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
    _test_log_state: ResMut<TestChromosomeLogState>,
) {
    let shift_pressed = keyboard_input.pressed(KeyCode::ShiftLeft) || keyboard_input.pressed(KeyCode::ShiftRight);

    if keyboard_input.just_pressed(KeyCode::F) {
        if *current_sim_mode.get() != SimulationMode::RunningFast {
            println!("Switching to RunningFast mode (continuous).");
            batch_run_config.target_generations_this_batch = None; // Ensure continuous mode
            batch_run_config.generations_completed_this_batch = 0;
            batch_run_config.log_this_batch_intensively = false;
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
        evo_res.1.reset(); // This puts test chromosome as first individual
        
        // ADDED: Set the test chromosome as the current vehicle for visualization
        vis_chromosome.0 = Some(get_test_chromosome());
        println!("R pressed: Test chromosome loaded into CurrentVehicleChromosome for visualization.");
        
        next_sim_mode.set(SimulationMode::Paused);
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
                batch_run_config.log_this_batch_intensively = false;
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
        batch_run_config.log_this_batch_intensively = false;
        return;
    }

    if engine.population.generation_count >= config.num_generations {
        println!("Target total generations ({}) reached. Switching to Paused.", config.num_generations);
        next_sim_mode.set(SimulationMode::Paused);
        batch_run_config.target_generations_this_batch = None;
        batch_run_config.generations_completed_this_batch = 0;
        batch_run_config.log_this_batch_intensively = false;
        return;
    }

    engine.evolve_generation(physics_world, &batch_run_config);
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
            batch_run_config.target_generations_this_batch = None;
            batch_run_config.generations_completed_this_batch = 0;
            batch_run_config.log_this_batch_intensively = false;
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
    mut test_log_state: ResMut<TestChromosomeLogState>, // Changed from _test_log_state
    vehicle_parts_query: Query<Entity, With<VehiclePart>>, // Added for despawn
) {
    println!("Running setup_current_vehicle_for_visualization...");
    despawn_vehicle_parts_inline(&mut commands, &vehicle_parts_query);

    // Reset simulation time and state for the new vehicle
    vis_state.current_simulation_time = 0.0;
    vis_state.is_simulating_current_vehicle = true;
    vis_state.initial_chassis_position = None; // Will be set by spawner
    vis_state.chassis_entity_for_logging = None; // Reset chassis entity for logging

    // Optionally reset physics pause state, or leave as is based on user preference
    // vis_state.is_physics_paused = false; // Uncomment if you want physics to always unpause on new vehicle

    let chromosome_to_spawn = if vis_state.use_debug_vehicle {
        println!("DEBUG_VEHICLE flag is ON. Spawning test chromosome.");
        Some(get_test_chromosome()) // Use the test chromosome if flag is set
    } else {
        current_vehicle.0.clone() // Otherwise, use the one from the resource
    };

    if let Some(chromosome) = chromosome_to_spawn {
        println!("Spawning vehicle for visualization. Chassis width: {}", chromosome.chassis.width);

        let vehicle_def = VehicleDefinition::new(
            &chromosome, 
            &config
        );

        let (initial_x, initial_y) = vehicle_def.get_initial_chassis_position();
        vis_state.initial_chassis_position = Some(Vec2::new(initial_x, initial_y));
        
        if chromosome == get_test_chromosome() {
            test_log_state.is_logging = true;
            test_log_state.step_count = 0;
            
            // Add a small delay before starting logging to ensure entities are spawned
            // This prevents logging empty queries during the first few frames
            
            // Setup regular physics log file
            match File::create("visualization_physics_log.txt") {
                Ok(file) => {
                    test_log_state.log_file = Some(Mutex::new(file));
                    println!("Created/Truncated visualization_physics_log.txt for test chromosome.");
                    if let Some(log_file_mutex) = &test_log_state.log_file {
                        if let Ok(mut log_file_guard) = log_file_mutex.lock() {
                            writeln!(log_file_guard, "--- Visualization Physics Log: Test Chromosome ---").unwrap_or_default();
                            writeln!(log_file_guard, "Chassis Width (from chromosome): {}", chromosome.chassis.width).unwrap_or_default();
                            writeln!(log_file_guard, "Note: Entity spawning may take a few frames, expect empty queries initially.").unwrap_or_default();
                        }
                    }
                },
                Err(e) => {
                    eprintln!("FAILED to create/truncate visualization_physics_log.txt: {}", e);
                    test_log_state.log_file = None;
                }
            }
            // Setup debug query log file
            match File::create("visualization_query_debug_log.txt") {
                Ok(file) => {
                    test_log_state.debug_log_file = Some(Mutex::new(file));
                    println!("Created/Truncated visualization_query_debug_log.txt.");
                    if let Some(debug_log_mutex) = &test_log_state.debug_log_file {
                        if let Ok(mut debug_file_guard) = debug_log_mutex.lock() {
                            writeln!(debug_file_guard, "--- Visualization Query Debug Log ---").unwrap_or_default();
                            writeln!(debug_file_guard, "Expected chassis entity: Will be stored after spawning").unwrap_or_default();
                        }
                    }
                },
                Err(e) => {
                    eprintln!("FAILED to create/truncate visualization_query_debug_log.txt: {}", e);
                    test_log_state.debug_log_file = None;
                }
            }
        } else {
            test_log_state.is_logging = false;
            test_log_state.log_file = None;
            test_log_state.debug_log_file = None;
        }

        let chassis_width = vehicle_def.chromosome.chassis.width;
        let chassis_height = vehicle_def.chromosome.chassis.height;
        
        let chassis_entity = vehicle_def.spawn_with_customizers(
            &mut commands,
            &config, // Pass the SimulationConfig resource
            |chassis_cmds| {
                chassis_cmds
                    .insert(VehiclePart)
                    .insert(Name::new("Chassis"))
                    .insert(Sprite {
                        color: Color::rgb(0.7, 0.7, 0.8),
                        custom_size: Some(Vec2::new(chassis_width, chassis_height)),
                        ..default()
                    })
                    .insert(Velocity::default());
                println!("[Vis Setup] Applied chassis customizer. Should have VehiclePart, Name, Sprite.");
                
                // DIAGNOSTIC: Log what we're actually inserting
                println!("[DIAGNOSTIC] Chassis customizer inserting: VehiclePart, Name, Sprite (NOT SpriteBundle)");
                println!("[DIAGNOSTIC] Note: TransformBundle will be added by physics.rs spawn_with_customizers");
            },
            |wheel_cmds, wheel_idx, wheel_gene| {
                wheel_cmds
                    .insert(VehiclePart)
                    .insert(VisualizedWheel { motor_gene_torque: wheel_gene.motor_torque })
                    .insert(Name::new(format!("Wheel {}", wheel_idx)))
                    .insert(Sprite {
                        color: if wheel_gene.motor_torque > 0.0 { 
                            Color::rgb(0.8, 0.3, 0.3) 
                        } else if wheel_gene.motor_torque < 0.0 { 
                            Color::rgb(0.3, 0.3, 0.8) 
                        } else { 
                            Color::rgb(0.5, 0.5, 0.5) 
                        },
                        custom_size: Some(Vec2::new(wheel_gene.radius * 2.0, wheel_gene.radius * 2.0)),
                        ..default()
                    })
                    .insert(Velocity::default());
                
                // DIAGNOSTIC: Log what components we're adding
                println!("[DIAGNOSTIC] Wheel {} customizer inserting: VehiclePart, VisualizedWheel, Name, Sprite", wheel_idx);
                println!("[DIAGNOSTIC] Note: physics.rs will add Transform, RigidBody, Collider, Velocity, etc.");
                
                // Add wheel rotation indicator as a child
                wheel_cmds.with_children(|parent| {
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
                println!("[Vis Setup] Applied wheel {} customizer. Should have VehiclePart, VisualizedWheel, Name, Sprite.", wheel_idx);
            }
        );
        
        // ADDED: Store chassis entity ID in visualization state for logging
        vis_state.chassis_entity_for_logging = Some(chassis_entity);
        println!("[Vis Setup] Stored chassis entity {:?} in vis_state for logging", chassis_entity);

    } else {
        println!("No chromosome available in CurrentVehicleChromosome resource for visualization.");
    }

    // REMOVED: Manual dt setting that might interfere with Bevy's fixed timestep
    // rapier_context.integration_parameters.dt = 1.0 / 60.0; // Ensure physics is active
    // println!("Physics simulation should be active for visualization (dt={}).", rapier_context.integration_parameters.dt);
    println!("Physics simulation should be active for visualization.");
}

// Original despawn_vehicle_parts, called by OnExit - keeps the original signature for system registration
pub fn despawn_vehicle_parts(
    mut commands: Commands,
    vehicle_parts_query: Query<Entity, With<VehiclePart>>,
) {
    for entity in vehicle_parts_query.iter() {
        commands.entity(entity).despawn_recursive();
    }
}

// Inline version that takes references for direct calling
fn despawn_vehicle_parts_inline(
    commands: &mut Commands,
    vehicle_parts_query: &Query<Entity, With<VehiclePart>>,
) {
    for entity in vehicle_parts_query.iter() {
        commands.entity(entity).despawn_recursive();
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
    _test_log_state: ResMut<TestChromosomeLogState>, 
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
                batch_run_config.log_this_batch_intensively = false;
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
        let run_batch = |gens: usize, intensive_log: bool, batch_config: &mut BatchRunConfig, next_mode: &mut NextState<SimulationMode>| {
            batch_config.target_generations_this_batch = Some(gens);
            batch_config.generations_completed_this_batch = 0;
            batch_config.log_this_batch_intensively = intensive_log;
            next_mode.set(SimulationMode::RunningFast);
        };

        if ui.button("Run 1 Gen (Log All Steps)").clicked() {
            run_batch(1, true, &mut batch_run_config, &mut next_sim_mode);
        }
        if ui.button("Run 5 Gens").clicked() {
            run_batch(5, false, &mut batch_run_config, &mut next_sim_mode);
        }
        if ui.button("Run 10 Gens").clicked() {
            run_batch(10, false, &mut batch_run_config, &mut next_sim_mode);
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
    
    println!(
        "RapierConfiguration updated: gravity=({:.2}, {:.2})", 
        rapier_config.gravity.x, rapier_config.gravity.y
    );
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
    // Query for all wheels with their relevant components for logging and potential actions
    // wheel_query_for_log: Query<(&Transform, &Velocity, &Name, &VisualizedWheel), With<VehiclePart>>,
    // MODIFIED: Query for the chassis for logging, now includes Entity
    chassis_query_for_log: Query<(Entity, &Transform, &Velocity), (With<VehiclePart>, Without<VisualizedWheel>)>,
    _next_sim_mode: ResMut<NextState<SimulationMode>>,
    mut distance_data: ResMut<DistanceComparisonData>,
    mut test_log_state: ResMut<TestChromosomeLogState>,
    rapier_context: Res<RapierContext>, // ADDED: To check physics timestep
) {
    // Only log to file, remove console debug logging since file logging works now
    if vis_state.is_simulating_current_vehicle {
        // ADDED: Log physics stepping details for test chromosome
        if test_log_state.is_logging && test_log_state.step_count % 60 == 0 {
            println!("[Visualization] Step {}, Time<Fixed> elapsed: {:.4}s, vis_state.current_simulation_time: {:.4}s", 
                test_log_state.step_count, time.elapsed_seconds(), vis_state.current_simulation_time);
            
            // ADDED: Log actual physics timestep being used
            println!("[Visualization] Physics dt: {:.6}, Time<Fixed> delta: {:.6}", 
                rapier_context.integration_parameters.dt, time.delta_seconds());
        }
        
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
                // MODIFIED: Correctly destructure the 3-tuple from the query
                for (_entity, transform, _velocity) in chassis_query_for_log.iter() {
                    chassis_pos = Some(Vec2::new(transform.translation.x, transform.translation.y));
                    break; // Found the chassis, no need to iterate further
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
fn configure_rapier_params(
    mut rapier_context: ResMut<RapierContext>,
    config: Res<SimulationConfig>
) {
    // Access the underlying Rapier configuration
    let context = rapier_context.as_mut();
    
    // MODIFIED: Set parameters from SimulationConfig to match headless simulation
    context.integration_parameters.erp = config.erp; 
    context.integration_parameters.joint_erp = config.joint_erp;
    context.integration_parameters.max_velocity_iterations = config.solver_iterations;
    // Note: simulation.rs also doesn't set max_position_iterations due to previous errors, assuming default is fine or covered by velocity.
    
    println!(
        "Rapier integration_parameters configured: erp={}, joint_erp={}, max_vel_iters={}",
        context.integration_parameters.erp, 
        context.integration_parameters.joint_erp,
        context.integration_parameters.max_velocity_iterations
    );
}

fn handle_test_chassis_logging(
    mut test_log_state: ResMut<TestChromosomeLogState>,
    _vis_state: Res<VisualizationState>, 
    // MODIFIED: Add Velocity back for debugging physics issues
    chassis_query: Query<(&Transform, &Name, Option<&Velocity>)>,
    // NEW: Add Velocity back for wheel debugging  
    wheel_query: Query<(&Transform, &Name, Option<&VisualizedWheel>, Option<&Velocity>)>,
    // Keep the all entities query for debugging
    all_named_entities: Query<(Entity, &Name)>,
    time: Res<Time>, 
) {
    if !test_log_state.is_logging {
        return; 
    }

    let current_log_step = test_log_state.step_count;
    test_log_state.step_count += 1;

    // Skip logging for the first few steps to allow entities to be properly spawned
    if current_log_step < 5 {
        return;
    }

    // Only log every ~10th step since this now runs in Update (much faster than FixedUpdate)
    // DISABLED for high-resolution debugging: Log every step now
    // if current_log_step % 10 != 0 {
    //     return;
    // }

    // --- Query Debug Logging --- 
    if test_log_state.debug_log_file.is_some() {
        if let Some(debug_mutex) = &test_log_state.debug_log_file {
            if let Ok(mut debug_file) = debug_mutex.lock() {
                writeln!(debug_file, "--- Query Debug: Step {} (Time: {:.2}s) ---", current_log_step, time.elapsed_seconds()).unwrap_or_default();
                
                // Query ALL entities with Name to see what exists
                let all_entities: Vec<_> = all_named_entities.iter().collect(); 
                writeln!(debug_file, "  Found {} total entities with Name:", all_entities.len()).unwrap_or_default();
                for (entity, name) in all_entities.iter().take(10) { // Limit to first 10
                    writeln!(debug_file, "    Entity {:?}: Name=\"{}\"", entity, name.as_str()).unwrap_or_default();
                }
                
                // NEW: Check specific entities for components
                for (entity, name) in all_entities.iter() {
                    if name.as_str() == "Chassis" || name.as_str().starts_with("Wheel") {
                        writeln!(debug_file, "  Checking entity {:?} (Name: \"{}\")", entity, name.as_str()).unwrap_or_default();
                        
                        // Check if it has Transform, Velocity components using direct queries
                        let has_transform = chassis_query.iter().any(|(_, n, _)| n.as_str() == name.as_str());
                        let has_velocity = wheel_query.iter().any(|(_, n, _, v)| n.as_str() == name.as_str() && v.is_some());
                        
                        writeln!(debug_file, "    Has Transform+Name: {}", has_transform).unwrap_or_default();
                        writeln!(debug_file, "    Found in wheel_query: {}", has_velocity).unwrap_or_default();
                    }
                }
                
                // Try to find entities by name
                let chassis_entities: Vec<_> = chassis_query.iter().filter(|(_, name, _)| name.as_str() == "Chassis").collect();
                writeln!(debug_file, "  Found {} chassis entities by name:", chassis_entities.len()).unwrap_or_default();
                
                let wheel_entities: Vec<_> = wheel_query.iter().filter(|(_, name, _, _)| name.as_str().starts_with("Wheel")).collect();
                writeln!(debug_file, "  Found {} wheel entities by name:", wheel_entities.len()).unwrap_or_default();
            } 
        }
    } 
    
    // --- Regular Physics State Logging ---     
    if let Some(mutex_file) = &test_log_state.log_file {
        if let Ok(mut file) = mutex_file.lock() {
            writeln!(file, "--- Step {} ---", current_log_step).unwrap_or_default();
            
            let mut chassis_logged_this_step = false;
            // Find the chassis by name
            for (transform, name, velocity) in chassis_query.iter() { 
                if name.as_str() == "Chassis" {
                    let vel_info = match velocity {
                        Some(vel) => format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                            vel.linvel.x, vel.linvel.y, vel.angvel),
                        None => "NO VELOCITY COMPONENT".to_string()
                    };
                    
                    log_line_viz(
                        &mut *file, 
                        format!(
                            "  Chassis (Name): pos=({:.4}, {:.4}), rot={:.4}, {}", 
                            transform.translation.x, transform.translation.y, 
                            transform.rotation.to_euler(EulerRot::ZYX).0,
                            vel_info
                        )
                    );
                    chassis_logged_this_step = true;
                    break; 
                }
            }
            
            if !chassis_logged_this_step {
                log_line_viz(&mut *file, format!("  Chassis: NOT FOUND by name."));
            }

            // Log wheels by name
            for (transform, name, wheel_marker, velocity) in wheel_query.iter() {
                if name.as_str().starts_with("Wheel") {
                    let vel_info = match velocity {
                        Some(vel) => format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                            vel.linvel.x, vel.linvel.y, vel.angvel),
                        None => "NO VELOCITY COMPONENT".to_string()
                    };
                    
                    log_line_viz(
                        &mut *file, 
                        format!(
                            "  Wheel '{}' (GeneTorque: {:.2}): pos=({:.4}, {:.4}), rot={:.4}, {}", 
                            name.as_str(), 
                            wheel_marker.map_or(0.0, |w| w.motor_gene_torque), 
                            transform.translation.x, 
                            transform.translation.y, 
                            transform.rotation.to_euler(EulerRot::ZYX).0,
                            vel_info
                        )
                    );
                }
            }
        } else {
            eprintln!("[handle_test_chassis_logging] Failed to lock log file.");
        }
    }
}

// Helper function for logging lines in visualization, DRY
fn log_line_viz(file_guard: &mut impl std::io::Write, message: impl AsRef<str>) {
    if let Err(e) = writeln!(file_guard, "{}", message.as_ref()) {
        eprintln!("[VIS_LOG_WRITE_ERR] {}", e);
    }
} 