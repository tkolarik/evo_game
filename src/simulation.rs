use crate::organism::{Chromosome, get_test_chromosome};
use crate::physics::{PhysicsParameters, GROUP_GROUND, GROUP_VEHICLE, GROUND_FILTER, VEHICLE_FILTER};
use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use std::io::Write;
use std::fs::File;
use std::collections::HashMap;
use std::num::NonZeroUsize;

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

        // Create the log file outside the Bevy app if we are in a test run
        let mut log_file_handle: Option<File> = if is_test_run {
            println!("HEADLESS: Attempting to create headless_physics_log.txt");
            match File::create("headless_physics_log.txt") {
                Ok(file) => Some(file),
                Err(e) => {
                    eprintln!("Failed to create headless_physics_log.txt: {}", e);
                    None
                }
            }
        } else {
            None
        };

        // Create a headless Bevy app for physics simulation
        let mut app = App::new();
        
        // Add only the minimal plugins needed for physics simulation
        app.add_plugins(MinimalPlugins)
           .add_plugins(RapierPhysicsPlugin::<NoUserData>::default())
           .insert_resource(RapierConfiguration {
                gravity: Vec2::new(0.0, self.physics_parameters.gravity),
                ..Default::default()
           });
        
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
            log_interval_steps: 60, // Log every 60 steps (1 second at 60 FPS)
            current_step: 0,
        });
        
        // Initialize QueryStates once before they are used by systems
        let mut chassis_query_state_for_final_log = app.world.query_filtered::<&Transform, With<HeadlessChassis>>();
        let mut wheel_query_state_for_final_log = app.world.query_filtered::<(&Transform, &HeadlessWheel), ()>();

        // Add systems for simulation
        app.add_systems(Startup, setup_headless_simulation)
           .add_systems(Update, (
                update_simulation_metrics, 
                apply_wheel_motors,
                log_simulation_state_periodically
            ));
        
        // Run the simulation for the configured duration
        let sim_steps = (config.sim_duration_secs * 60.0).round() as u32; // 60 FPS
        
        for _ in 0..sim_steps {
            app.update();
            // Increment logging step counter if in test run
            if is_test_run {
                if let Some(mut logging_config) = app.world.get_resource_mut::<LoggingConfig>() {
                    logging_config.current_step += 1;
                }
            }
        }
        
        // After the loop, if we had a log file, perform one final log using the app's state
        if let Some(mut file) = log_file_handle.take() { // Explicitly take to close
            let final_metrics = app.world.resource::<SimulationMetrics>().clone(); // Clone to release borrow
            let final_params = app.world.resource::<HeadlessSimulationParams>().clone(); // Clone to release borrow
            let elapsed_time = app.world.resource::<Time>().elapsed_seconds();

            // Perform a final detailed log if it was a test run
            if final_params.is_test_run {
                 log_current_state_to_file_with_query_state(
                    &final_params,
                    &final_metrics,
                    &mut chassis_query_state_for_final_log,
                    &mut wheel_query_state_for_final_log,
                    elapsed_time,
                    &mut file,
                    &app.world
                );
            }

            // Log summary results
            writeln!(file, "--- Final Results ---").unwrap_or_default();
            writeln!(file, "Distance travelled: {}", final_metrics.distance_travelled).unwrap_or_default();
            writeln!(file, "Fragmentation penalty: {}", final_metrics.fragmentation_penalty).unwrap_or_default();
            writeln!(file, "Final fitness: {}", f32::max(final_metrics.distance_travelled, 0.0) * final_metrics.fragmentation_penalty).unwrap_or_default();
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
struct HeadlessVehiclePart;

// Component to tag the chassis specifically
#[derive(Component)]
struct HeadlessChassis;

// Component to tag wheels
#[derive(Component)]
struct HeadlessWheel {
    index: usize,
    motor_torque: f32,
}

// System to set up the headless simulation
fn setup_headless_simulation(
    mut commands: Commands,
    params: Res<HeadlessSimulationParams>,
    mut metrics: ResMut<SimulationMetrics>,
) {
    let chromosome = &params.chromosome;
    
    // Create ground using the same logic as in visualization
    commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, -0.15, 0.0)),
        Collider::cuboid(50.0, 0.1), // Large flat ground
        RigidBody::Fixed,
        Friction::coefficient(1.0),
        CollisionGroups::new(Group::from_bits_truncate(GROUP_GROUND), Group::from_bits_truncate(VEHICLE_FILTER)),
        Name::new("HeadlessGround"), // Added name for clarity
    ));
    
    // Create chassis
    let initial_y = params.initial_height + chromosome.chassis.height / 2.0;
    let chassis = commands.spawn((
        TransformBundle::from(Transform::from_xyz(0.0, initial_y, 0.0)),
        RigidBody::Dynamic,
        Collider::cuboid(chromosome.chassis.width / 2.0, chromosome.chassis.height / 2.0),
        ColliderMassProperties::Density(chromosome.chassis.density),
        Friction::coefficient(0.5),
        CollisionGroups::new(Group::from_bits_truncate(GROUP_VEHICLE), Group::from_bits_truncate(GROUND_FILTER)),
        HeadlessVehiclePart,
        HeadlessChassis,
        Name::new("Chassis"),
    )).id();
    
    // Store initial chassis position
    metrics.initial_chassis_position = Some(Vec2::new(0.0, initial_y));
    
    // Create wheels
    for (i, wheel_gene) in chromosome.wheels.iter().enumerate() {
        if wheel_gene.active {
            let wheel_x = wheel_gene.get_x_position(chromosome.chassis.width);
            let wheel_y = wheel_gene.get_y_position(chromosome.chassis.height);
            
            // Position relative to chassis
            let wheel_pos = Vec2::new(wheel_x, wheel_y);
            
            // World position of wheel
            let world_pos = Vec3::new(
                wheel_x, 
                initial_y + wheel_y,
                0.0
            );
            
            // Create wheel entity
            let wheel = commands.spawn((
                TransformBundle::from(Transform::from_translation(world_pos)),
                RigidBody::Dynamic,
                Collider::ball(wheel_gene.radius),
                ColliderMassProperties::Density(wheel_gene.density),
                Friction::coefficient(wheel_gene.friction_coefficient),
                CollisionGroups::new(Group::from_bits_truncate(GROUP_VEHICLE), Group::from_bits_truncate(GROUND_FILTER)),
                ExternalImpulse::default(),
                HeadlessVehiclePart,
                HeadlessWheel { 
                    index: i,
                    motor_torque: wheel_gene.motor_torque,
                },
                Name::new(format!("Wheel {}", i)),
            )).id();
            
            // Create joint between wheel and chassis
            let joint_builder = RevoluteJointBuilder::new()
                .local_anchor1(wheel_pos) // Anchor on chassis (relative to chassis center)
                .local_anchor2(Vec2::ZERO); // Anchor on wheel (relative to wheel center)
            
            // The ImpulseJoint component is added to the chassis entity.
            // The first argument to ImpulseJoint::new is the other entity in the joint (the wheel).
            commands.entity(chassis).insert(ImpulseJoint::new(wheel, joint_builder.build()));
        }
    }
}

// System to apply motor torques to wheels
fn apply_wheel_motors(
    mut wheel_query: Query<(&HeadlessWheel, &mut ExternalImpulse)>,
    time: Res<Time>,
) {
    for (wheel, mut impulse) in wheel_query.iter_mut() {
        // Apply the motor torque as an external impulse
        // First clear any existing impulse to prevent accumulation
        impulse.torque_impulse = 0.0;
        
        // Scale torque by time delta
        let scaled_torque = wheel.motor_torque * time.delta_seconds();
        
        // Apply torque
        impulse.torque_impulse = scaled_torque;
    }
}

// System to update simulation metrics
fn update_simulation_metrics(
    mut metrics: ResMut<SimulationMetrics>,
    chassis_query: Query<&Transform, With<HeadlessChassis>>,
    _part_query: Query<&Transform, With<HeadlessVehiclePart>>,
) {
    // Update chassis position and calculate distance
    if let Ok(chassis_transform) = chassis_query.get_single() {
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
        metrics.fragmentation_penalty = 0.01;
    }
}

// Logging system that runs periodically if conditions are met.
// It queries the world directly for the current state.
fn log_simulation_state_periodically(
    logging_config: Res<LoggingConfig>,
    _params_res: Res<HeadlessSimulationParams>,
    metrics_res: Res<SimulationMetrics>,
    time_res: Res<Time>,
    chassis_q: Query<&Transform, With<HeadlessChassis>>,
    wheel_q: Query<(&Transform, &HeadlessWheel)>,
) {
    if logging_config.is_test_run && logging_config.current_step > 0 && logging_config.current_step % logging_config.log_interval_steps == 0 {
        match File::options().append(true).open("headless_physics_log.txt") {
            Ok(mut file) => {
                let elapsed_time = time_res.elapsed_seconds();
                writeln!(file, "HEADLESS LOG (Periodic): Time={:.2}s, Distance={:.2}", 
                        elapsed_time, 
                        metrics_res.distance_travelled).unwrap_or_default();
                
                if let Ok(chassis_transform) = chassis_q.get_single() {
                    writeln!(file, "  Chassis: pos=({:.4}, {:.4}), rot={:.4}", 
                            chassis_transform.translation.x, 
                            chassis_transform.translation.y,
                            chassis_transform.rotation.to_euler(EulerRot::XYZ).2).unwrap_or_default();
                }
                
                for (transform, wheel) in wheel_q.iter() {
                    writeln!(file, "  Wheel {}: pos=({:.4}, {:.4}), rot={:.4}",
                            wheel.index,
                            transform.translation.x,
                            transform.translation.y,
                            transform.rotation.to_euler(EulerRot::XYZ).2).unwrap_or_default();
                }
            },
            Err(e) => eprintln!("Failed to open headless_physics_log.txt for periodic logging: {}", e),
        }
    }
}

// Renamed to differentiate from the periodic logging that uses direct Query
fn log_current_state_to_file_with_query_state(
    _params: &HeadlessSimulationParams, 
    metrics: &SimulationMetrics,
    chassis_query_state: &mut QueryState<&Transform, With<HeadlessChassis>>,
    wheel_query_state: &mut QueryState<(&Transform, &HeadlessWheel)>, 
    elapsed_time: f32,
    log_file: &mut File,
    world: &World, // World is still needed for QueryState::get_single / QueryState::iter
) {
    writeln!(log_file, "HEADLESS LOG (Final): Time={:.2}s, Distance={:.2}", 
            elapsed_time, 
            metrics.distance_travelled).unwrap_or_default();
    
    if let Ok(chassis_transform) = chassis_query_state.get_single(world) {
        writeln!(log_file, "  Chassis: pos=({:.4}, {:.4}), rot={:.4}", 
                chassis_transform.translation.x, 
                chassis_transform.translation.y,
                chassis_transform.rotation.to_euler(EulerRot::XYZ).2).unwrap_or_default();
    }
    
    for (transform, wheel) in wheel_query_state.iter(world) {
        writeln!(log_file, "  Wheel {}: pos=({:.4}, {:.4}), rot={:.4}",
                wheel.index,
                transform.translation.x,
                transform.translation.y,
                transform.rotation.to_euler(EulerRot::XYZ).2).unwrap_or_default();
    }
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