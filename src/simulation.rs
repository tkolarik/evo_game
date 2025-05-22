use crate::organism::{Chromosome, get_test_chromosome};
use crate::physics::{
    PhysicsParameters, 
    GROUP_GROUND, GROUND_FILTER, 
    VehicleDefinition, GroundDefinition
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
    physics_parameters: PhysicsParameters,
}

impl PhysicsWorld {
    pub fn new(config: &SimulationConfig) -> Self {
        // Create physics parameters
        let physics_params = PhysicsParameters {
            gravity: config.gravity,
            ground_friction: config.ground_friction,
            ground_restitution: 0.0,
            erp: 0.01, // Unified value from testing
            joint_erp: 0.01, // Unified value from testing
            solver_iterations: 4,
        };
        
        Self {
            physics_parameters: physics_params,
        }
    }

    pub fn evaluate_fitness(&mut self, chromosome: &Chromosome, config: &SimulationConfig) -> f32 {
        let test_chromosome = get_test_chromosome();
        let is_test_run = *chromosome == test_chromosome;

        if is_test_run {
            println!("HEADLESS: evaluate_fitness called FOR TEST CHROMOSOME.");
        }

        let mut initial_log_file_handle: Option<File> = if is_test_run {
            println!("HEADLESS: Attempting to create/truncate headless_physics_log.txt");
            let log_path_str = "headless_physics_log.txt";
            match File::create(log_path_str) { 
                Ok(mut file) => {
                    let abs_path = PathBuf::from(log_path_str).canonicalize().unwrap_or_else(|_| PathBuf::from(log_path_str));
                    println!("[SUCCESS] Created/Truncated headless_physics_log.txt at {:?}", abs_path);
                    // These initial writes still go direct to the handle before it's put in resource
                    writeln!(file, "--- Headless Physics Log: Test Chromosome ---").unwrap_or_default();
                    writeln!(file, "Chassis Width (from chromosome): {}", chromosome.chassis.width).unwrap_or_default();
                    Some(file)
                },
                Err(e) => {
                    eprintln!("[!!!! CRITICAL ERROR !!!!] FAILED to create/truncate headless_physics_log.txt: {}", e);
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
           .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
           .insert_resource(RapierConfiguration {
                gravity: Vec2::new(0.0, self.physics_parameters.gravity),
                ..Default::default()
           })
           .insert_resource(self.physics_parameters.clone());
        
        // Insert SimulationConfig as a resource
        app.insert_resource(config.clone());
        
        // Track simulation metrics using a resource
        app.insert_resource(SimulationMetrics::default());
        
        // Configure fixed timestep to match visualization
        app.insert_resource(Time::<Fixed>::from_seconds(1.0 / 60.0));
        
        // Store simulation parameters
        app.insert_resource(HeadlessSimulationParams {
            chromosome: chromosome.clone(),
            initial_height: config.initial_height_above_ground,
            is_test_run,
        });
        
        // Add the logging system here, it will manage its own conditions
        app.insert_resource(LoggingConfig {
            is_test_run,
            log_interval_steps: 10, // MODIFIED: Log every 10 steps (about 6 times per second at 60fps)
            current_step: 0,
        });
        
        // Initialize QueryStates once before they are used by systems
        // These are now primarily for the final log, periodic will use live queries.
        let mut chassis_query_state_for_final_log = app.world.query_filtered::<(&Transform, Option<&Velocity>), With<HeadlessChassis>>();
        let mut wheel_query_state_for_final_log = app.world.query_filtered::<(&Transform, &HeadlessWheel, Option<&Velocity>), ()>();

        // Add system to configure Rapier integration parameters after setup
        // app.add_systems(Startup, configure_headless_rapier_parameters.after(setup_headless_simulation));
        app.add_systems(PostStartup, configure_headless_rapier_parameters); // Try PostStartup

        // Add systems for simulation
        app.add_systems(Startup, setup_headless_simulation)
           .add_systems(FixedUpdate, (
                update_simulation_metrics, 
                log_simulation_state_periodically
            ));
        
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
        
        for _ in 0..sim_steps {
            // Manually advance Time<Fixed> internal state and then run FixedUpdate schedule.
            app.world.resource_scope(|_world, mut time_fixed: Mut<Time<Fixed>>| {
                time_fixed.advance_by(fixed_time_delta_duration);
            });
            app.world.run_schedule(FixedUpdate);

            // We might still need to run the main schedule for other things if systems rely on it (e.g. PostUpdate)
            // For a minimal Rapier setup, FixedUpdate might be enough. Let's test this first.
            // If PostUpdate systems are needed (e.g. for query state sync before logging), we might need app.update() or parts of it.
            // For now, assume FixedUpdate is sufficient to drive physics and update transforms for logging.
            // app.update(); // Comment out for now to see if FixedUpdate alone is enough

            actual_steps_executed += 1;
            // Increment logging step counter if in test run
            if is_test_run {
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
    physics_params_res: Res<PhysicsParameters>,
    config: Res<SimulationConfig>,
    log_file: Option<Res<HeadlessLogFile>> // Add log file resource
) {
    let chromosome = &params.chromosome;
    sim_log!(log_file, "[LOG Sim|Setup] Running setup_headless_simulation for chromosome width: {}", chromosome.chassis.width);
    
    let ground_def = GroundDefinition::new(&physics_params_res);
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, ground_def.get_ground_center_y(), 0.0)),
        Collider::cuboid(ground_def.half_width, ground_def.half_thickness),
        RigidBody::Fixed,
        Friction::coefficient(ground_def.friction),
        CollisionGroups::new(Group::from_bits_truncate(GROUP_GROUND), Group::from_bits_truncate(GROUND_FILTER)),
        Name::new("HeadlessGround"),
    ));
    
    // Create vehicle using VehicleDefinition and the new spawner
    let vehicle_def = VehicleDefinition::new(chromosome, config.initial_height_above_ground);
    let (initial_chassis_x, initial_chassis_y) = vehicle_def.get_initial_chassis_position();

    let chassis_entity_id = vehicle_def.spawn_with_customizers(
        &mut commands,
        |chassis_cmds| {
            println!("[LOG Sim|Setup] Customizing chassis entity..."); 
            chassis_cmds
                .insert(HeadlessVehiclePart)
                .insert(HeadlessChassis)
                .insert(Name::new("Chassis"))
                .insert(Velocity::default())
                .insert(ExternalImpulse::default()); // Add ExternalImpulse for force application
            
            println!("[LOG Sim|Setup] Chassis components (HeadlessVehiclePart, HeadlessChassis, Name, Velocity) should be added.");
        },
        |wheel_cmds, wheel_idx, wheel_gene| {
            println!("[LOG Sim|Setup] Customizing wheel {}...", wheel_idx); // Stays as println!
            wheel_cmds
                .insert(HeadlessVehiclePart)
                .insert(HeadlessWheel { 
                    index: wheel_idx,
                    motor_torque: wheel_gene.motor_torque, 
                })
                .insert(Name::new(format!("Wheel {}", wheel_idx)))
                .insert(Velocity::default())
                .insert(ExternalImpulse::default()); // Add ExternalImpulse for torque application
            
            println!("[LOG Sim|Setup] Wheel {} components should be added.", wheel_idx); // Stays as println!
        }
    );
    sim_log!(log_file, "[LOG Sim|Setup] Chassis entity ID from spawn_with_customizers: {:?}", chassis_entity_id);
    
    metrics.initial_chassis_position = Some(Vec2::new(initial_chassis_x, initial_chassis_y));
    sim_log!(log_file, "[LOG Sim|Setup] Initial chassis position set in metrics: ({}, {})", initial_chassis_x, initial_chassis_y);
}

// System to update simulation metrics
fn update_simulation_metrics(
    mut metrics: ResMut<SimulationMetrics>,
    chassis_query: Query<(Entity, &Transform, Option<&Velocity>), With<HeadlessChassis>>,
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
            let vel_info = match chassis_velocity {
                Some(vel) => format!("Velocity: linvel=({:.4}, {:.4}), angvel={:.4}", 
                                vel.linvel.x, vel.linvel.y, vel.angvel),
                None => "NO VELOCITY COMPONENT".to_string()
            };
            
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
    chassis_q: Query<(&Transform, Option<&Velocity>), With<HeadlessChassis>>, 
    wheel_q: Query<(&Transform, &HeadlessWheel, Option<&Velocity>)>, 
    log_file: Option<Res<HeadlessLogFile>> // Add log file resource for debug messages
) {
    // Debug: Always log whether this system is running
    sim_log!(log_file, "[DEBUG] log_simulation_state_periodically is running, step={}, config.is_test_run={}, config.log_interval_steps={}", 
             logging_config.current_step, logging_config.is_test_run, logging_config.log_interval_steps);

    if logging_config.is_test_run && logging_config.current_step > 0 && logging_config.current_step % logging_config.log_interval_steps == 0 {
        // DEBUG: This check passed, now we'll log
        sim_log!(log_file, "[DEBUG] Periodic log condition met: step {}, interval {}", logging_config.current_step, logging_config.log_interval_steps);

        match File::options().append(true).open("headless_physics_log.txt") {
            Ok(mut file) => {
                // DEBUG: Confirm file opened
                sim_log!(log_file, "[DEBUG] headless_physics_log.txt opened for append.");

                let elapsed_time = time_res.elapsed_seconds_f64();
                let current_step = logging_config.current_step;
                writeln!(file, "--- Step {} (Time={:.4}s) ---", current_step, elapsed_time).unwrap_or_default();
                
                if let Ok((chassis_transform, chassis_velocity)) = chassis_q.get_single() {
                    // DEBUG: Chassis found
                    sim_log!(log_file, "[DEBUG] Chassis found in periodic log: pos=({:.4}, {:.4})", 
                           chassis_transform.translation.x, chassis_transform.translation.y);
                    let vel_info = match chassis_velocity {
                        Some(vel) => format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                            vel.linvel.x, vel.linvel.y, vel.angvel),
                        None => "NO VELOCITY COMPONENT".to_string()
                    };
                    
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
                    let vel_info = match velocity {
                        Some(vel) => format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                          vel.linvel.x, vel.linvel.y, vel.angvel),
                        None => "NO VELOCITY COMPONENT".to_string()
                    };
                    
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
                sim_log!(log_file, "[ERROR] Failed to open headless_physics_log.txt for periodic logging: {}", e);
            }
        }
    }
}

// Renamed to differentiate from the periodic logging that uses direct Query
// And updated to use live queries directly via World instead of pre-cached QueryState for the *very last* step.
fn log_current_state_to_file(
    _params: &HeadlessSimulationParams, 
    _metrics: &SimulationMetrics,
    chassis_query: &mut QueryState<(&Transform, Option<&Velocity>), With<HeadlessChassis>>, // Make Velocity optional
    wheel_query: &mut QueryState<(&Transform, &HeadlessWheel, Option<&Velocity>)>,   // Make Velocity optional
    elapsed_time: f32,
    log_file: &mut File,
    world: &World, 
) {
    writeln!(log_file, "--- Final State (Time={:.4}s) ---", elapsed_time).unwrap_or_default();
    // writeln!(log_file, "HEADLESS LOG (Final): Time={:.2}s, Distance={:.2}", 
    //         elapsed_time, 
    //         metrics.distance_travelled).unwrap_or_default();
    
    // Use the QueryState for chassis for the final log
    if let Ok((chassis_transform, chassis_velocity)) = chassis_query.get_single(world) {
         let vel_info = match chassis_velocity {
             Some(vel) => format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                                vel.linvel.x, vel.linvel.y, vel.angvel),
             None => "NO VELOCITY COMPONENT".to_string()
         };
         
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
        let vel_info = match velocity {
            Some(vel) => format!("linvel=({:.4}, {:.4}), angvel={:.4}", 
                               vel.linvel.x, vel.linvel.y, vel.angvel),
            None => "NO VELOCITY COMPONENT".to_string()
        };
        
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
    physics_params_res: Res<PhysicsParameters>, // Your custom PhysicsParameters resource
    log_file: Option<Res<HeadlessLogFile>> // Add log file resource
) {
    sim_log!(log_file, "[LOG Sim|HeadlessConfig] Attempting to configure Rapier integration parameters for headless mode...");
    let context = rapier_context.as_mut();
    context.integration_parameters.erp = physics_params_res.erp;
    context.integration_parameters.joint_erp = physics_params_res.joint_erp;
    // dt is typically handled by Bevy's Time<Fixed> and RapierPhysicsPlugin
    // context.integration_parameters.dt = 1.0 / 60.0; 
    sim_log!(log_file, "[LOG Sim|HeadlessConfig] Headless Rapier erp set to: {}, joint_erp set to: {}", 
        context.integration_parameters.erp, context.integration_parameters.joint_erp);
}

#[derive(Debug, Clone, Resource)]
pub struct SimulationConfig {
    pub sim_duration_secs: f32,
    pub initial_height_above_ground: f32,
    pub gravity: f32,
    pub ground_friction: f32,
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

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            sim_duration_secs: 15.0,
            initial_height_above_ground: 1.0, // DEBUG: Increased substantially to 1.0
            gravity: -9.81,
            ground_friction: 1.0, // Increased friction for better traction
            population_size: 50,
            num_generations: 100,
            tournament_size: 5,
            elitism_count: 2, // Carry over top 2 individuals
            mutation_rate_per_gene: 0.02,
            mutation_rate_per_individual: 0.8, // 80% chance an individual is mutated
            crossover_type: CrossoverType::Uniform,
        }
    }
} 