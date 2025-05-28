use crate::organism::{Chromosome, get_test_chromosome};
use crate::physics::{
    VehicleDefinition, GroundDefinition,
    GROUP_GROUND, GROUND_FILTER, 
};
use bevy::prelude::*;
// use bevy::app::CoreSchedule; // Removed unused import
use bevy_rapier2d::prelude::*;
use std::io::Write;
use std::fs::File;
use std::path::PathBuf; // Added for path manipulation
use std::sync::Mutex; // Added for Mutex
// use std::collections::HashMap; // Removed unused import
// use std::num::NonZeroUsize; // Removed unused import

// Resource to hold the log file handle for headless simulation
#[derive(Resource)]
struct HeadlessLogFile(Option<Mutex<File>>);

// Helper macro for logging to the HeadlessLogFile resource
macro_rules! sim_log {
    ($log_file_res:expr, $($arg:tt)*) => {{
        if let Some(log_file_mutexed) = $log_file_res.as_ref().and_then(|res| res.0.as_ref()) {
            if let Ok(mut file_guard) = log_file_mutexed.lock() {
                if let Err(e) = writeln!(file_guard, $($arg)*) {
                    eprintln!("[SIM_LOG_WRITE_ERR] {}", e);
                }
            } else {
                eprintln!("[SIM_LOG_LOCK_ERR] Could not lock log file. Original message: {}", format!($($arg)*));
            }
        } else {
            // Fallback to println if log file resource isn't available or file is None (e.g., not a test run)
            // In a real scenario, you might buffer these or handle differently.
            // For now, this ensures messages aren't lost during transition if something is missed.
            println!($($arg)*);
        }
    }};
}

// Placeholder for physics world details
pub struct PhysicsWorld {
    // Removed physics_parameters field, as SimulationConfig will be used directly or passed around.
}

impl PhysicsWorld {
    pub fn new(_config: &SimulationConfig) -> Self { // config might be used later if PhysicsWorld needs to store parts of it
        // PhysicsParameters struct is removed. Its values are now in SimulationConfig.
        // RapierConfiguration and other direct physics settings will be applied in evaluate_fitness
        // or by systems directly using SimulationConfig.
        Self {}
    }

    pub fn evaluate_fitness(&mut self, chromosome: &Chromosome, config: &SimulationConfig, force_intensive_logging: bool) -> f32 {
        let test_chromosome = get_test_chromosome();
        let effective_is_test_run = (*chromosome == test_chromosome) || force_intensive_logging;

        if effective_is_test_run {
            println!("HEADLESS: evaluate_fitness called. Chromosome is test_chromosome: {}, force_intensive_logging: {}. Logging all steps.", 
                     *chromosome == test_chromosome, force_intensive_logging);
        }

        let mut initial_log_file_handle: Option<File> = if effective_is_test_run {
            println!("HEADLESS: Attempting to create/truncate headless_main_log.txt");
            let main_log_path_str = "headless_main_log.txt";
            match File::create(main_log_path_str) { 
                Ok(mut file) => {
                    let abs_path = PathBuf::from(main_log_path_str).canonicalize().unwrap_or_else(|_| PathBuf::from(main_log_path_str));
                    println!("[SUCCESS] Created/Truncated {} at {:?}", main_log_path_str, abs_path);
                    // These initial writes still go direct to the handle before it's put in resource
                    writeln!(file, "--- Headless Physics Log (Main): Test Chromosome ---").unwrap_or_default();
                    writeln!(file, "Chassis Width (from chromosome): {}", chromosome.chassis.width).unwrap_or_default();
                    
                    // Also create/truncate the periodic log file
                    let periodic_log_path_str = "headless_periodic_state_log.txt";
                    match File::create(periodic_log_path_str) {
                        Ok(_) => {
                            let periodic_abs_path = PathBuf::from(periodic_log_path_str).canonicalize().unwrap_or_else(|_| PathBuf::from(periodic_log_path_str));
                            println!("[SUCCESS] Created/Truncated {} at {:?}", periodic_log_path_str, periodic_abs_path);
                            // No initial write needed here, log_simulation_state_periodically will handle its headers
                        },
                        Err(e) => {
                            eprintln!("[!!!! CRITICAL ERROR !!!!] FAILED to create/truncate {}: {}", periodic_log_path_str, e);
                            // Potentially decide if main log creation should also fail or proceed with caution
                        }
                    }
                    Some(file) // Return the main log file handle
                },
                Err(e) => {
                    eprintln!("[!!!! CRITICAL ERROR !!!!] FAILED to create/truncate {}: {}", main_log_path_str, e);
                    None
                }
            }
        } else {
            None
        };

        let mut app = App::new();

        // Insert the log file into a resource if it was created
        let log_file_resource = HeadlessLogFile(initial_log_file_handle.map(Mutex::new));
        app.insert_resource(log_file_resource); // Now the resource is available
        
        app.add_plugins(MinimalPlugins)
           .add_plugins(RapierPhysicsPlugin::<NoUserData>::default()) // Consider ::pixels_per_meter(1.0) if visual uses it for identical setup
           .insert_resource(RapierConfiguration { // MODIFIED: Only set gravity here
                gravity: Vec2::new(0.0, config.gravity),
                // solver_iterations, erp, joint_erp will be set via RapierContext.integration_parameters
                ..Default::default()
           })
           .insert_resource(config.clone())
        
        // Track simulation metrics using a resource
           .insert_resource(SimulationMetrics::default()) // Chained this call
        
        // Configure fixed timestep to match visualization
           .insert_resource(Time::<Fixed>::from_seconds(1.0 / 60.0)) // Chained this call
        
        // Store simulation parameters
           .insert_resource(HeadlessSimulationParams {
            chromosome: chromosome.clone(),
            initial_height: config.initial_height_above_ground,
            is_test_run: effective_is_test_run,
           })
        
        // Add the logging system here, it will manage its own conditions
           .insert_resource(LoggingConfig {
            is_test_run: effective_is_test_run,
            log_interval_steps: if effective_is_test_run { 10 } else { 10 },
            current_step: 0,
           });
        
        // Initialize QueryStates once before they are used by systems
        // These are now primarily for the final log, periodic will use live queries.
        let mut chassis_query_state_for_final_log = app.world.query_filtered::<(&Transform, &Velocity), With<HeadlessChassis>>();
        let mut wheel_query_state_for_final_log = app.world.query_filtered::<(&Transform, &HeadlessWheel, &Velocity), ()>();

        // Add system to configure Rapier integration parameters after setup
        // app.add_systems(Startup, configure_headless_rapier_parameters.after(setup_headless_simulation));
        app.add_systems(PostStartup, configure_headless_rapier_parameters); // Try PostStartup. Semicolon added if needed by style, but often not for single system line.

        // Add systems for simulation
        app.add_systems(Startup, setup_headless_simulation)
           .add_systems(FixedUpdate, (
                update_simulation_metrics, 
                log_simulation_state_periodically
            )); // Semicolon added
        
        // Run the main schedule once to execute Startup systems and apply their commands.
        // This also runs PostStartup systems like configure_headless_rapier_parameters.
        app.update();

        // app.update(); // REMOVED - Second update call, potentially causing premature FixedUpdate runs.
        
        // Now we can use sim_log! as the app and potentially the resource are initialized
        let maybe_log_res = app.world.get_resource::<HeadlessLogFile>();
        sim_log!(maybe_log_res, "[LOG Sim|EvalFit] Completed initial app.update() for startup systems.");

        let sim_steps = (config.sim_duration_secs * 60.0).round() as u32;
        let mut actual_steps_executed = 0;
        let fixed_time_delta_duration = std::time::Duration::from_secs_f64(1.0 / 60.0);
        
        // ADDED: Log simulation parameters - get fresh reference
        {
            let maybe_log_res = app.world.get_resource::<HeadlessLogFile>();
            sim_log!(maybe_log_res, "[LOG Sim|EvalFit] Starting simulation loop: sim_duration_secs={}, sim_steps={}, dt={}", 
                config.sim_duration_secs, sim_steps, 1.0 / 60.0);
        }
        
        for step in 0..sim_steps {
            // ADDED: Log step details periodically
            if effective_is_test_run && step % 60 == 0 {
                // FIXED: Get log resource reference inside the condition to avoid borrowing issues
                let maybe_log_res = app.world.get_resource::<HeadlessLogFile>();
                sim_log!(maybe_log_res, "[LOG Sim|EvalFit] Step {}/{}, Time<Fixed> before advance: elapsed={:.4}s", 
                    step, sim_steps, app.world.resource::<Time<Fixed>>().elapsed_seconds());
            }
            
            // Manually advance Time<Fixed> internal state and then run FixedUpdate schedule.
            app.world.resource_scope(|_world, mut time_fixed: Mut<Time<Fixed>>| {
                time_fixed.advance_by(fixed_time_delta_duration);
            });
            
            // CRITICAL FIX: Run the full app.update() to process Rapier physics
            // The FixedUpdate alone is insufficient for Rapier physics to work properly
            app.update();

            actual_steps_executed += 1;
            // Increment logging step counter if in test run
            if effective_is_test_run {
                if let Some(mut logging_config) = app.world.get_resource_mut::<LoggingConfig>() {
                    logging_config.current_step += 1;
                }
            }
        }
        
        // After the loop, if we had a log file, perform one final log using the app's state
        if let Some(mut file) = app.world.remove_resource::<HeadlessLogFile>().and_then(|res| res.0.map(|mutex| mutex.into_inner().unwrap())) {
            let final_metrics = app.world.resource::<SimulationMetrics>().clone(); 
            let final_params = app.world.resource::<HeadlessSimulationParams>().clone(); 
            
            // These printlns are for console feedback during finalization, or could also be logged to file if handle was kept somehow
            println!("[evaluate_fitness] Loop intended for {} steps, actual steps executed: {}", sim_steps, actual_steps_executed);
            let time_fixed_res = app.world.resource::<Time<Fixed>>();
            println!("[evaluate_fitness] Time<Fixed>: delta_seconds={:?}, timestep={:?}, overstep={:?}, elapsed_seconds={:?}", 
                time_fixed_res.delta_seconds(), time_fixed_res.timestep(), time_fixed_res.overstep(), time_fixed_res.elapsed_seconds());
            let time_main_res = app.world.resource::<Time>();
            println!("[evaluate_fitness] Time (main): delta_seconds={:?}, elapsed_seconds={:?}", 
                time_main_res.delta_seconds(), time_main_res.elapsed_seconds());

            let elapsed_time_for_log = time_fixed_res.elapsed_seconds(); 
            let manually_calculated_elapsed_time = actual_steps_executed as f32 * (1.0 / 60.0);
            // println!("[evaluate_fitness] Time<Fixed>.elapsed_seconds() for log: {:.4}", elapsed_time_for_log); // Already have this
            // println!("[evaluate_fitness] Manually calculated elapsed_time for comparison: {:.4}", manually_calculated_elapsed_time); // Already have this

            // Perform a final detailed log if it was a test run
            if final_params.is_test_run {
                 log_current_state_to_file(
                    &final_params,
                    &final_metrics,
                    &mut chassis_query_state_for_final_log,
                    &mut wheel_query_state_for_final_log,
                    elapsed_time_for_log, 
                    &mut file, // Pass the unwrapped file handle here
                    &app.world
                );
            }

            // Log summary results
            writeln!(file, "--- Final Results ---").unwrap_or_default();
            writeln!(file, "Distance travelled: {}", final_metrics.distance_travelled).unwrap_or_default();
            writeln!(file, "Fragmentation penalty: {}", final_metrics.fragmentation_penalty).unwrap_or_default();
            // Use the fitness calculated from metrics, not dependent on a potentially incorrect elapsed time for calculation here.
            let fitness_from_metrics = f32::max(final_metrics.distance_travelled, 0.0) * final_metrics.fragmentation_penalty;
            writeln!(file, "Final fitness: {}", fitness_from_metrics).unwrap_or_default();
        } else {
            // If the log file was never created (not a test run), these go to console
            let final_metrics = app.world.resource::<SimulationMetrics>(); 
            println!("[evaluate_fitness] Loop intended for {} steps, actual steps executed: {} (NO LOG FILE)", sim_steps, actual_steps_executed);
            // ... other console logs if needed ...
        }
        
        let metrics = app.world.resource::<SimulationMetrics>();
        let fitness = f32::max(metrics.distance_travelled, 0.0) * metrics.fragmentation_penalty;
        fitness
    }
}

// Resource to store simulation parameters
#[derive(Resource, Clone)]
struct HeadlessSimulationParams {
    chromosome: Chromosome,
    initial_height: f32,
    is_test_run: bool,
}

// Resource to track simulation metrics
#[derive(Resource, Default, Clone)]
struct SimulationMetrics {
    initial_chassis_position: Option<Vec2>,
    current_chassis_position: Option<Vec2>,
    distance_travelled: f32,
    fragmentation_penalty: f32,
}

// New resource to manage logging state
#[derive(Resource)]
struct LoggingConfig {
    is_test_run: bool,
    log_interval_steps: u32,
    current_step: u32,
}

// Component to tag entities that are part of the vehicle
#[derive(Component)]
pub struct HeadlessVehiclePart;

// Component to tag the chassis specifically
#[derive(Component)]
pub struct HeadlessChassis;

// Component to tag wheels
#[derive(Component)]
pub struct HeadlessWheel {
    pub index: usize,
    pub motor_torque: f32,
}

// System to set up the headless simulation
fn setup_headless_simulation(
    mut commands: Commands,
    params: Res<HeadlessSimulationParams>,
    mut metrics: ResMut<SimulationMetrics>,
    config: Res<SimulationConfig>, // ADDED
    log_file: Option<Res<HeadlessLogFile>> // Add log file resource
) {
    let chromosome = &params.chromosome;
    sim_log!(log_file, "[LOG Sim|Setup] Running setup_headless_simulation for chromosome width: {}", chromosome.chassis.width);
    
    // MODIFIED: Create GroundDefinition using config
    let ground_def = GroundDefinition::new(&config);
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, ground_def.get_ground_center_y(), 0.0)),
        Collider::cuboid(ground_def.half_width, ground_def.half_thickness),
        RigidBody::Fixed,
        Friction::coefficient(ground_def.friction),
        CollisionGroups::new(Group::from_bits_truncate(GROUP_GROUND), Group::from_bits_truncate(GROUND_FILTER)),
        Name::new("HeadlessGround"),
    ));
    
    // MODIFIED: Use the same VehicleDefinition spawning as rendered mode
    let vehicle_def = VehicleDefinition::new(chromosome, &config);
    let (initial_chassis_x, initial_chassis_y) = vehicle_def.get_initial_chassis_position();

    let chassis_entity_id = vehicle_def.spawn_with_customizers(
        &mut commands,
        &config, // ADDED: Pass config to spawn_with_customizers
        |chassis_cmds| {
            sim_log!(log_file, "[LOG Sim|Setup] Customizing chassis entity..."); 
            chassis_cmds
                .insert(HeadlessVehiclePart)
                .insert(HeadlessChassis)
                .insert(Name::new("Chassis"))
                .insert(Velocity::default())  // CRITICAL: Add Velocity component
                .insert(ExternalImpulse::default()); // Add ExternalImpulse for force application
            
            sim_log!(log_file, "[LOG Sim|Setup] Chassis components (HeadlessVehiclePart, HeadlessChassis, Name, Velocity) should be added.");
        },
        |wheel_cmds, wheel_idx, wheel_gene| {
            sim_log!(log_file, "[LOG Sim|Setup] Customizing wheel {}...", wheel_idx); // Stays as println!
            wheel_cmds
                .insert(HeadlessVehiclePart)
                .insert(HeadlessWheel { 
                    index: wheel_idx,
                    motor_torque: wheel_gene.motor_torque, 
                })
                .insert(Name::new(format!("Wheel {}", wheel_idx)))
                .insert(Velocity::default())  // CRITICAL: Add Velocity component
                .insert(ExternalImpulse::default()); // Add ExternalImpulse for torque application
            
            sim_log!(log_file, "[LOG Sim|Setup] Wheel {} components should be added.", wheel_idx); // Stays as println!
            // ADDED: Log wheel motor torque for debugging
            sim_log!(log_file, "[LOG Sim|Setup] Wheel {} motor_torque from gene: {:.4}", wheel_idx, wheel_gene.motor_torque);
        }
    );
    sim_log!(log_file, "[LOG Sim|Setup] Chassis entity ID from spawn_with_customizers: {:?}", chassis_entity_id);
    
    metrics.initial_chassis_position = Some(Vec2::new(initial_chassis_x, initial_chassis_y));
    sim_log!(log_file, "[LOG Sim|Setup] Initial chassis position set in metrics: ({}, {})", initial_chassis_x, initial_chassis_y);
}

// System to update simulation metrics
fn update_simulation_metrics(
    mut metrics: ResMut<SimulationMetrics>,
    chassis_query: Query<(Entity, &Transform, &Velocity), With<HeadlessChassis>>, // ADDED: Include Velocity
    _part_query: Query<&Transform, With<HeadlessVehiclePart>>,
    log_file: Option<Res<HeadlessLogFile>>, // Add log file resource
    logging_config: Option<Res<LoggingConfig>> // Add logging config to check current_step
) {
    // Only log detailed messages if we're in the main simulation loop (current_step > 0)
    let should_log_verbose = logging_config.as_ref().map_or(false, |config| config.is_test_run && config.current_step > 0);
    
    if should_log_verbose {
        sim_log!(log_file, "[LOG Sim|Metrics] update_simulation_metrics system running.");
    }
    
    if let Ok((entity, chassis_transform, chassis_velocity)) = chassis_query.get_single() {
        if should_log_verbose {
            let vel_info = format!("Velocity: linvel=({:.4}, {:.4}), angvel={:.4}", 
                            chassis_velocity.linvel.x, chassis_velocity.linvel.y, chassis_velocity.angvel);
            
            sim_log!(log_file, "[LOG Sim|Metrics] Chassis entity {:?} FOUND. Transform: {:?}, {}", 
                    entity, chassis_transform.translation, vel_info);
        }
        
        let current_pos = Vec2::new(
            chassis_transform.translation.x,
            chassis_transform.translation.y
        );
        
        // Update metrics
        if let Some(initial_pos) = metrics.initial_chassis_position {
            metrics.distance_travelled = current_pos.x - initial_pos.x;
        }
        
        metrics.current_chassis_position = Some(current_pos);
        
        // Check for fragmentation by counting disconnected pieces
        // For now, use a simple heuristic: if all parts are present, assume no fragmentation
        metrics.fragmentation_penalty = 1.0; // Default: no penalty
    } else {
        // Chassis not found - severe penalty
        if should_log_verbose {
            match chassis_query.get_single() { 
                Ok(_) => {} 
                Err(e) => sim_log!(log_file, "[LOG Sim|Metrics] Chassis NOT FOUND in update_simulation_metrics. Query error: {:?}", e),
            }
        }
        metrics.fragmentation_penalty = 0.01;
    }
}

// Logging system that runs periodically if conditions are met.
// It queries the world directly for the current state.
fn log_simulation_state_periodically(
    logging_config: Res<LoggingConfig>,
    _params_res: Res<HeadlessSimulationParams>, 
    _metrics_res: Res<SimulationMetrics>,
    time_res: Res<Time<Fixed>>, 
    chassis_q: Query<(&Transform, &Velocity), With<HeadlessChassis>>, 
    wheel_q: Query<(&Transform, &HeadlessWheel, &Velocity)>, 
    log_file: Option<Res<HeadlessLogFile>> // Add log file resource for debug messages
) {
    // Debug: Always log whether this system is running
    sim_log!(log_file, "[DEBUG] log_simulation_state_periodically is running, step={}, config.is_test_run={}, config.log_interval_steps={}", 
             logging_config.current_step, logging_config.is_test_run, logging_config.log_interval_steps);

    if logging_config.is_test_run && logging_config.current_step > 0 && logging_config.current_step % logging_config.log_interval_steps == 0 {
        // DEBUG: This check passed, now we'll log
        sim_log!(log_file, "[DEBUG] Periodic log condition met: step {}, interval {}", logging_config.current_step, logging_config.log_interval_steps);

        match File::options().append(true).open("headless_periodic_state_log.txt") { // MODIFIED: Log to new file
            Ok(mut file) => {
                // DEBUG: Confirm file opened
                sim_log!(log_file, "[DEBUG] headless_periodic_state_log.txt opened for append.");

                let elapsed_time = time_res.elapsed_seconds_f64();
                let current_step = logging_config.current_step;
                writeln!(file, "--- Step {} (Time={:.4}s) ---", current_step, elapsed_time).unwrap_or_default();
                
                if let Ok((chassis_transform, chassis_velocity)) = chassis_q.get_single() {
                    // DEBUG: Chassis found
                    sim_log!(log_file, "[DEBUG] Chassis found in periodic log: pos=({:.4}, {:.4})", 
                           chassis_transform.translation.x, chassis_transform.translation.y);
                    let vel_info = format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                            chassis_velocity.linvel.x, chassis_velocity.linvel.y, chassis_velocity.angvel);
                    
                    writeln!(file, "  Chassis: pos=({:.4}, {:.4}), rot={:.4}, {}", 
                            chassis_transform.translation.x, 
                            chassis_transform.translation.y,
                            chassis_transform.rotation.to_euler(EulerRot::ZYX).0,
                            vel_info
                    ).unwrap_or_default();
                } else {
                    // DEBUG: Chassis NOT found
                    sim_log!(log_file, "[DEBUG] Chassis NOT found in periodic log.");
                    writeln!(file, "  Chassis: NOT FOUND").unwrap_or_default();
                }
                
                // DEBUG: How many wheels found?
                let wheel_count = wheel_q.iter().count();
                sim_log!(log_file, "[DEBUG] Wheels found in periodic log: {}", wheel_count);

                for (transform, wheel, velocity) in wheel_q.iter() {
                    let vel_info = format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                          velocity.linvel.x, velocity.linvel.y, velocity.angvel);
                    
                    writeln!(file, "  Wheel {}: pos=({:.4}, {:.4}), rot={:.4}, {}",
                            wheel.index,
                            transform.translation.x,
                            transform.translation.y,
                            transform.rotation.to_euler(EulerRot::ZYX).0,
                            vel_info
                    ).unwrap_or_default();
                }
            },
            Err(e) => {
                // DEBUG: Error opening file
                sim_log!(log_file, "[ERROR] Failed to open headless_periodic_state_log.txt for periodic logging: {}", e);
            }
        }
    }
}

// Renamed to differentiate from the periodic logging that uses direct Query
// And updated to use live queries directly via World instead of pre-cached QueryState for the *very last* step.
fn log_current_state_to_file(
    _params: &HeadlessSimulationParams, 
    _metrics: &SimulationMetrics,
    chassis_query: &mut QueryState<(&Transform, &Velocity), With<HeadlessChassis>>, // CHANGED: Non-optional Velocity
    wheel_query: &mut QueryState<(&Transform, &HeadlessWheel, &Velocity)>,   // CHANGED: Non-optional Velocity
    elapsed_time: f32,
    log_file: &mut File,
    world: &World, 
) {
    writeln!(log_file, "--- Final State (Time={:.4}s) ---", elapsed_time).unwrap_or_default();
    
    // Use the QueryState for chassis for the final log
    if let Ok((chassis_transform, chassis_velocity)) = chassis_query.get_single(world) {
         let vel_info = format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                chassis_velocity.linvel.x, chassis_velocity.linvel.y, chassis_velocity.angvel);
         
         writeln!(log_file, "  Chassis: pos=({:.4}, {:.4}), rot={:.4}, {}", 
                chassis_transform.translation.x, 
                chassis_transform.translation.y,
                chassis_transform.rotation.to_euler(EulerRot::ZYX).0, // ZYX order, take Z (index 0)
                vel_info
        ).unwrap_or_default();
    } else {
        writeln!(log_file, "  Chassis: NOT FOUND (Final - QueryState)").unwrap_or_default();
    }
    
    // Use the QueryState for wheels for the final log
    for (transform, wheel, velocity) in wheel_query.iter(world) {
        let vel_info = format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                               velocity.linvel.x, velocity.linvel.y, velocity.angvel);
        
        writeln!(log_file, "  Wheel {} (QS): pos=({:.4}, {:.4}), rot={:.4}, {}",
                wheel.index,
                transform.translation.x,
                transform.translation.y,
                transform.rotation.to_euler(EulerRot::ZYX).0, // ZYX order, take Z (index 0)
                vel_info
        ).unwrap_or_default();
    }
}

// Startup system to configure Rapier integration parameters in headless mode
fn configure_headless_rapier_parameters(
    mut rapier_context: ResMut<RapierContext>,
    config: Res<SimulationConfig>, 
    log_file: Option<Res<HeadlessLogFile>>
) {
    sim_log!(log_file, "[LOG Sim|HeadlessConfig] Attempting to configure Rapier integration parameters for headless mode...");
    let context = rapier_context.as_mut();
    context.integration_parameters.erp = config.erp;
    context.integration_parameters.joint_erp = config.joint_erp;
    context.integration_parameters.max_velocity_iterations = config.solver_iterations;
    // Removed max_position_iterations as it's not a valid field

    sim_log!(log_file, "[LOG Sim|HeadlessConfig] Headless Rapier erp set to: {}, joint_erp set to: {}, max_vel_iters: {}", 
        context.integration_parameters.erp, 
        context.integration_parameters.joint_erp,
        context.integration_parameters.max_velocity_iterations
    );
}

#[derive(Debug, Clone, Resource)]
pub struct SimulationConfig {
    pub sim_duration_secs: f32,
    pub initial_height_above_ground: f32,
    pub gravity: f32,
    pub ground_friction: f32,
    // New physics parameters
    pub ground_restitution: f32,
    pub erp: f32,
    pub joint_erp: f32,
    pub solver_iterations: usize,
    pub ccd_enabled: bool, // Continuous Collision Detection
    pub vehicle_damping_linear: f32,
    pub vehicle_damping_angular: f32,
    pub vehicle_chassis_friction: f32,
    pub vehicle_wheel_restitution: f32,
    // Evolutionary algorithm parameters
    pub population_size: usize,
    pub num_generations: usize,
    pub tournament_size: usize, // For tournament selection
    pub elitism_count: usize, // Number of elite individuals to carry over
    pub mutation_rate_per_gene: f64, // e.g., 0.01 to 0.05
    pub mutation_rate_per_individual: f64, // Chance an individual undergoes at least one mutation
    pub crossover_type: CrossoverType,
}

#[derive(Debug, Clone, Copy)]
pub enum CrossoverType {
    SinglePoint,
    Uniform,
}

// Added from main.rs
#[derive(Debug, Clone, Resource, Default)]
pub struct BatchRunConfig {
    pub target_generations_this_batch: Option<usize>,
    pub generations_completed_this_batch: usize,
    pub log_this_batch_intensively: bool,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            sim_duration_secs: 15.0,
            initial_height_above_ground: 1.0,
            gravity: -9.81,
            ground_friction: 1.0,
            // Defaults for new physics parameters
            ground_restitution: 0.0,        // From PhysicsParameters default
            erp: 0.01,                      // From PhysicsParameters default
            joint_erp: 0.01,                // From PhysicsParameters default
            solver_iterations: 4,           // From PhysicsParameters default (Rapier default)
            ccd_enabled: true,             // CHANGED: Enable CCD to prevent tunneling
            vehicle_damping_linear: 0.5,    // From VehicleDefinition default
            vehicle_damping_angular: 0.5,   // From VehicleDefinition default
            vehicle_chassis_friction: 0.7,  // From VehicleDefinition default
            vehicle_wheel_restitution: 0.1, // From VehicleDefinition default
            // Evolutionary algorithm parameters
            population_size: 50,
            num_generations: 100,
            tournament_size: 5,
            elitism_count: 2,
            mutation_rate_per_gene: 0.02,
            mutation_rate_per_individual: 0.8,
            crossover_type: CrossoverType::Uniform,
        }
    }
} 