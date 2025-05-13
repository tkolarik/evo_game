use crate::organism::{Chromosome, WheelGenes};
use rapier2d::prelude::*; 
use bevy::prelude::Resource;

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
        Self {
            gravity: vector![0.0, config.gravity],
            integration_parameters: IntegrationParameters::default(),
            ground_friction: config.ground_friction,
            ground_restitution: 0.0, // Ground doesn't bounce much
        }
    }

    pub fn evaluate_fitness(&mut self, chromosome: &Chromosome, config: &SimulationConfig) -> f32 {
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

        // 1. Create the ground (with fixed body)
        let ground_body = RigidBodyBuilder::fixed()
            .translation(vector![0.0, GROUND_Y_POSITION - GROUND_THICKNESS])
            .build();
        let ground_handle = rigid_body_set.insert(ground_body);
        
        let ground_collider = ColliderBuilder::cuboid(1000.0, GROUND_THICKNESS) // Effectively infinite ground
            .friction(self.ground_friction)
            .restitution(self.ground_restitution)
            .build();
        collider_set.insert_with_parent(ground_collider, ground_handle, &mut rigid_body_set);

        // 2. Genotype to Phenotype Mapping: Create the vehicle
        let initial_chassis_x = 0.0;
        let initial_chassis_y = config.initial_height_above_ground + chromosome.chassis.height / 2.0;

        // Create chassis
        let chassis_rigid_body = RigidBodyBuilder::dynamic()
            .translation(vector![initial_chassis_x, initial_chassis_y])
            .build();
        let chassis_handle = rigid_body_set.insert(chassis_rigid_body);
        
        let chassis_collider = ColliderBuilder::cuboid(chromosome.chassis.width / 2.0, chromosome.chassis.height / 2.0)
            .density(chromosome.chassis.density)
            .friction(0.7) // Default chassis friction
            .build();
        collider_set.insert_with_parent(chassis_collider, chassis_handle, &mut rigid_body_set);

        let mut active_wheel_motors: Vec<(RigidBodyHandle, f32)> = Vec::new();

        // Create wheels and attach them
        for wheel_gene in &chromosome.wheels {
            if wheel_gene.active {
                let wheel_x_abs = wheel_gene.get_x_position(chromosome.chassis.width);
                let wheel_y_abs = wheel_gene.get_y_position(chromosome.chassis.height);

                let wheel_rb = RigidBodyBuilder::dynamic()
                    .translation(vector![
                        initial_chassis_x + wheel_x_abs,
                        initial_chassis_y + wheel_y_abs
                    ])
                    .build();
                let wheel_handle = rigid_body_set.insert(wheel_rb);
                
                let wheel_collider = ColliderBuilder::ball(wheel_gene.radius)
                    .density(wheel_gene.density)
                    .friction(wheel_gene.friction_coefficient)
                    .restitution(0.1) // Wheels might have a little bounce
                    .build();
                collider_set.insert_with_parent(wheel_collider, wheel_handle, &mut rigid_body_set);

                // Attach wheel to chassis with a revolute joint
                let joint = RevoluteJointBuilder::new()
                    .local_anchor1(point![wheel_x_abs, wheel_y_abs]) // Anchor on chassis
                    .local_anchor2(point![0.0, 0.0]); // Anchor on wheel center
                
                impulse_joint_set.insert(chassis_handle, wheel_handle, joint, true);

                if wheel_gene.motor_torque.abs() > 1e-3 { // Check for non-zero torque
                    active_wheel_motors.push((wheel_handle, wheel_gene.motor_torque));
                }
            }
        }

        // 3. Run the simulation loop
        let sim_steps = (config.sim_duration_secs / self.integration_parameters.dt).round() as usize;
        let initial_rb_x_pos = rigid_body_set[chassis_handle].translation().x;

        // Create a fresh physics pipeline just for this evaluation
        let mut physics_pipeline = PhysicsPipeline::new();

        for _step in 0..sim_steps {
            // Apply motor torques
            for (wheel_body_handle, torque) in &active_wheel_motors {
                if let Some(wheel_rb_mut) = rigid_body_set.get_mut(*wheel_body_handle) {
                    wheel_rb_mut.apply_torque_impulse(*torque * self.integration_parameters.dt, true);
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
                None, // Query pipeline, not used here
                &(), // Hook, not used here
                &(), // Event handler, not used here
            );
        }

        // 4. Calculate fitness
        let final_rb_x_pos = rigid_body_set[chassis_handle].translation().x;
        let distance_travelled = final_rb_x_pos - initial_rb_x_pos;

        // Fitness: max horizontal distance in positive direction
        f32::max(distance_travelled, 0.0)
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
            initial_height_above_ground: 1.0, // To avoid initial collision
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