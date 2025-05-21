use crate::organism::{Chromosome, get_test_chromosome};
use rapier2d::prelude::*; 
use bevy::prelude::Resource;
use std::io::Write; // For logging
use std::fs::File; // For logging
use std::collections::HashMap; // To map handles to wheel index for logging

// --- Collision Groups (mirrored from main.rs) ---
const GROUP_VEHICLE: u32 = 1 << 0; // 0b00000001
const GROUP_GROUND: u32 = 1 << 1;  // 0b00000010
// const VEHICLE_FILTER: u32 = GROUP_GROUND; // Vehicles collide with ground - Not directly used by ColliderBuilder like this
// const GROUND_FILTER: u32 = GROUP_VEHICLE;   // Ground collides with vehicles - Not directly used by ColliderBuilder like this

// Placeholder for physics world details
pub struct PhysicsWorld {
    gravity: Vector<f32>,
    integration_parameters: IntegrationParameters,
    ground_friction: f32,
    ground_restitution: f32, // Added for ground properties
}

// Position ground slightly below origin for initial placement
pub const GROUND_Y_POSITION: f32 = -0.1; 
const GROUND_THICKNESS: f32 = 0.1;

impl PhysicsWorld {
    pub fn new(config: &SimulationConfig) -> Self {
        // Create integration parameters that match Bevy's integration more closely
        let mut integration_params = IntegrationParameters::default();
        integration_params.dt = 1.0 / 60.0; // Match the fixed timestep used in visualization
        integration_params.erp = 0.8; // Default is 0.2, Bevy Rapier default is 0.8 for 2D
        integration_params.joint_erp = 0.8; // Default is 0.2, Bevy Rapier default is 0.8 for 2D
        // warmstart_coefficient is 1.0 by default in IntegrationParameters, which matches Bevy Rapier.
        
        Self {
            gravity: vector![0.0, config.gravity],
            integration_parameters: integration_params,
            ground_friction: config.ground_friction,
            ground_restitution: 0.0, // Ground doesn't bounce much
        }
    }

    pub fn evaluate_fitness(&mut self, chromosome: &Chromosome, config: &SimulationConfig) -> f32 {
        let test_chromosome = get_test_chromosome();
        let is_test_run = *chromosome == test_chromosome;

        if is_test_run {
            println!("HEADLESS: evaluate_fitness called FOR TEST CHROMOSOME."); // Confirm entry
        }

        let mut log_file: Option<File> = if is_test_run {
            println!("HEADLESS: Attempting to create headless_physics_log.txt"); // Confirm attempt
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

        let mut wheel_handles_for_logging: HashMap<RigidBodyHandle, usize> = HashMap::new();

        if is_test_run {
            if let Some(file) = &mut log_file {
                writeln!(file, "HEADLESS SIMULATION LOG").unwrap_or_default();
                writeln!(file, "Chassis target: width={}, height={}, density={}", 
                    chromosome.chassis.width, chromosome.chassis.height, chromosome.chassis.density).unwrap_or_default();
                for (i, wheel) in chromosome.wheels.iter().enumerate() {
                    if wheel.active {
                        writeln!(file, "Wheel {} target: radius={}, density={}, torque={}, friction={}", 
                            i, wheel.radius, wheel.density, wheel.motor_torque, wheel.friction_coefficient).unwrap_or_default();
                    }
                }
            }
        }

        // Create new physics state for each evaluation
        let mut rigid_body_set = RigidBodySet::new();
        let mut collider_set = ColliderSet::new();
        let mut impulse_joint_set = ImpulseJointSet::new();
        let mut multibody_joint_set = MultibodyJointSet::new();
        
        // Create fresh local instances of island manager, etc.
        let mut island_manager = IslandManager::new();
        let mut broad_phase = BroadPhase::new();
        let mut narrow_phase = NarrowPhase::new();
        let mut ccd_solver = CCDSolver::new();
        let mut query_pipeline = QueryPipeline::new(); // Create a query pipeline

        // 1. Create the ground (with fixed body)
        let ground_body = RigidBodyBuilder::fixed()
            .translation(vector![0.0, GROUND_Y_POSITION - GROUND_THICKNESS])
            .build();
        let ground_handle = rigid_body_set.insert(ground_body);
        
        let ground_collider = ColliderBuilder::cuboid(1000.0, GROUND_THICKNESS) // Effectively infinite ground
            .friction(self.ground_friction)
            .restitution(self.ground_restitution)
            .collision_groups(InteractionGroups::new(GROUP_GROUND.into(), GROUP_VEHICLE.into())) // Ground interacts with vehicles
            .build();
        collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

        // Store all vehicle part handles
        let mut vehicle_part_handles: Vec<RigidBodyHandle> = Vec::new();

        // 2. Genotype to Phenotype Mapping: Create the vehicle
        let initial_chassis_x = 0.0;
        let initial_chassis_y = config.initial_height_above_ground + chromosome.chassis.height / 2.0;

        // Create chassis with more damping to match visualization
        let chassis_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![initial_chassis_x, initial_chassis_y])
            .linear_damping(5.0)
            .angular_damping(5.0)
            .ccd_enabled(true)
            .build();
        let chassis_handle = rigid_body_set.insert(chassis_rigid_body);
        vehicle_part_handles.push(chassis_handle);
        
        let chassis_collider = ColliderBuilder::cuboid(chromosome.chassis.width / 2.0, chromosome.chassis.height / 2.0)
            .density(chromosome.chassis.density)
            .friction(0.7) // Default chassis friction
            .collision_groups(InteractionGroups::new(GROUP_VEHICLE.into(), GROUP_GROUND.into())) // Vehicle interacts with ground
            .build();
        collider_set.insert_with_parent(chassis_collider, chassis_handle, &mut rigid_body_set);

        // Create wheels and attach them
        for (idx, wheel_gene) in chromosome.wheels.iter().enumerate() {
            if wheel_gene.active {
                let wheel_x_abs = wheel_gene.get_x_position(chromosome.chassis.width);
                let wheel_y_abs = wheel_gene.get_y_position(chromosome.chassis.height);

                let wheel_rb = RigidBodyBuilder::dynamic()
                    .translation(vector![
                        initial_chassis_x + wheel_x_abs,
                        initial_chassis_y + wheel_y_abs
                    ])
                    .linear_damping(5.0)
                    .angular_damping(5.0)
                    .ccd_enabled(true)
                    .build();
                let wheel_handle = rigid_body_set.insert(wheel_rb);
                vehicle_part_handles.push(wheel_handle);
                if is_test_run { // Store handle for logging if it's the test run
                    wheel_handles_for_logging.insert(wheel_handle, idx);
                }
                
                let wheel_collider = ColliderBuilder::ball(wheel_gene.radius)
                    .density(wheel_gene.density)
                    .friction(wheel_gene.friction_coefficient)
                    .restitution(0.1) // Wheels might have a little bounce
                    .collision_groups(InteractionGroups::new(GROUP_VEHICLE.into(), GROUP_GROUND.into())) // Vehicle parts interact with ground
                    .build();
                collider_set.insert_with_parent(wheel_collider, wheel_handle, &mut rigid_body_set);

                // Attach wheel to chassis with a revolute joint
                let mut joint_builder = RevoluteJointBuilder::new()
                    .local_anchor1(point![wheel_x_abs, wheel_y_abs]) // Anchor on chassis
                    .local_anchor2(point![0.0, 0.0]); // Anchor on wheel center
                
                if wheel_gene.motor_torque != 0.0 {
                    joint_builder = joint_builder
                        .motor_model(MotorModel::ForceBased)
                        // Set target velocity very high/low to indicate direction, force is the limit
                        .motor_velocity(if wheel_gene.motor_torque > 0.0 { f32::MAX } else { f32::MIN }, 1.0) // REVERT factor to 1.0
                        .motor_max_force(wheel_gene.motor_torque.abs());
                }
                let joint = joint_builder.build();
                impulse_joint_set.insert(chassis_handle, wheel_handle, joint, true);
            }
        }

        // 3. Run the simulation loop
        let sim_steps = (config.sim_duration_secs / self.integration_parameters.dt).round() as usize;
        let initial_rb_x_pos = rigid_body_set.get(chassis_handle).map_or(0.0, |rb| rb.translation().x);

        // Create a fresh physics pipeline just for this evaluation
        let mut physics_pipeline = PhysicsPipeline::new();

        for step_num in 0..sim_steps {
            // Log state if it's the test chromosome
            if is_test_run {
                if let Some(file) = &mut log_file {
                    writeln!(file, "--- Step {} ---", step_num).unwrap_or_default();
                    if let Some(chassis_rb) = rigid_body_set.get(chassis_handle) {
                        writeln!(file, "Chassis: pos=({:.4}, {:.4}), rot={:.4}, linvel=({:.4}, {:.4}), angvel={:.4}", 
                            chassis_rb.translation().x, chassis_rb.translation().y, chassis_rb.rotation().angle(),
                            chassis_rb.linvel().x, chassis_rb.linvel().y, chassis_rb.angvel()).unwrap_or_default();
                    } else {
                        writeln!(file, "Chassis: NOT FOUND").unwrap_or_default();
                    }

                    for (handle, wheel_idx) in &wheel_handles_for_logging {
                        if let Some(wheel_rb) = rigid_body_set.get(*handle) {
                            writeln!(file, "Wheel {}: pos=({:.4}, {:.4}), rot={:.4}, linvel=({:.4}, {:.4}), angvel={:.4}",
                                wheel_idx, wheel_rb.translation().x, wheel_rb.translation().y, wheel_rb.rotation().angle(),
                                wheel_rb.linvel().x, wheel_rb.linvel().y, wheel_rb.angvel()).unwrap_or_default();
                        } else {
                            writeln!(file, "Wheel {}: NOT FOUND", wheel_idx).unwrap_or_default();
                        }
                    }
                }
            }

            physics_pipeline.step(
                &self.gravity,
                &self.integration_parameters,
                &mut island_manager,
                &mut broad_phase,
                &mut narrow_phase,
                &mut rigid_body_set,
                &mut collider_set,
                &mut impulse_joint_set,
                &mut multibody_joint_set,
                &mut ccd_solver,
                Some(&mut query_pipeline), // Pass the query pipeline
                &(), // Hook, not used here
                &(), // Event handler, not used here
            );
        }

        // 4. Calculate fitness
        let final_rb_x_pos = rigid_body_set.get(chassis_handle).map_or(initial_rb_x_pos, |rb| rb.translation().x);
        let distance_travelled = final_rb_x_pos - initial_rb_x_pos;
        let mut fitness = f32::max(distance_travelled, 0.0);

        // Fragmentation penalty logic
        let mut num_missing_parts = 0;
        for handle in &vehicle_part_handles {
            if rigid_body_set.get(*handle).is_none() {
                num_missing_parts += 1;
            }
        }

        if num_missing_parts > 0 {
            // Severe penalty for missing parts
            println!("Creature lost {} parts! Applying severe penalty.", num_missing_parts);
            fitness *= 0.01; // Or set to a very small value, or 0
        } else {
            // Check for disconnected pieces based on distance between parts
            // Threshold distance beyond which we consider parts disconnected
            let distance_threshold = 16.0; // Units as per your requirement
            
            // Get positions of all vehicle parts
            let part_positions: Vec<Vector<f32>> = vehicle_part_handles
                .iter()
                .filter_map(|&handle| rigid_body_set.get(handle).map(|rb| rb.translation().clone()))
                .collect();
            
            // Count disconnected pieces using a simple distance-based approach
            if !part_positions.is_empty() {
                // Count pieces by checking which parts are too far from any others
                let mut num_pieces = 0;
                let mut visited = vec![false; part_positions.len()];
                
                for i in 0..part_positions.len() {
                    if !visited[i] {
                        num_pieces += 1;
                        visited[i] = true;
                        
                        // Mark all parts that are connected to this one
                        let mut queue = vec![i];
                        while let Some(current) = queue.pop() {
                            for j in 0..part_positions.len() {
                                if !visited[j] {
                                    let distance = (part_positions[current] - part_positions[j]).magnitude();
                                    if distance <= distance_threshold {
                                        visited[j] = true;
                                        queue.push(j);
                                    }
                                }
                            }
                        }
                    }
                }
                
                if num_pieces > 1 {
                    println!("Creature broke into {} pieces! Applying penalty.", num_pieces);
                    fitness /= num_pieces as f32;
                }
            }
        }
        
        fitness
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
            initial_height_above_ground: 0.25, // Increased from 0.2 for a slight nudge
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