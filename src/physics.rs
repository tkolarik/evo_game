use crate::organism::Chromosome;
use crate::simulation::SimulationConfig; // MODIFIED: Import SimulationConfig
// Import specific Bevy types
use bevy::prelude::*;
// Import specific Bevy Rapier types
use bevy_rapier2d::prelude::{
    RigidBody, Collider, RevoluteJointBuilder, ImpulseJoint,
    Friction, Restitution, Damping, ColliderMassProperties,
    CollisionGroups, Group, ActiveEvents, MotorModel, ExternalImpulse, Ccd, // Added Ccd
};
use bevy::ecs::system::EntityCommands; // Added for customizer closures
use crate::organism::WheelGenes; // Added for wheel_customizer signature

// --- Collision Groups (shared across all physics systems) ---
pub const GROUP_VEHICLE: u32 = 1 << 0; // 0b00000001
pub const GROUP_GROUND: u32 = 1 << 1;  // 0b00000010
pub const VEHICLE_FILTER: u32 = GROUP_GROUND; // Vehicles collide with ground
pub const GROUND_FILTER: u32 = GROUP_VEHICLE;   // Ground collides with vehicles

// --- Ground Constants ---
pub const GROUND_Y_POSITION: f32 = -0.15; // Top surface of ground
pub const GROUND_THICKNESS: f32 = 0.1;    // Full thickness of ground

// --- Motor Configuration Constants ---
pub const MOTOR_TARGET_VEL_FORWARD: f32 = f32::MAX;
pub const MOTOR_TARGET_VEL_REVERSE: f32 = f32::MIN;
pub const MOTOR_STIFFNESS_FACTOR: f32 = 1.0;

// === Ground Builder ===
pub struct GroundDefinition {
    pub half_width: f32,
    pub half_thickness: f32,
    pub y_position: f32,
    pub friction: f32,
    pub restitution: f32,
}

impl GroundDefinition {
    pub fn new(config: &SimulationConfig) -> Self {
        Self {
            half_width: 1000.0, // This could also be moved to SimulationConfig if it needs to vary
            half_thickness: GROUND_THICKNESS / 2.0,
            y_position: GROUND_Y_POSITION,
            friction: config.ground_friction,
            restitution: config.ground_restitution,
        }
    }

    // Calculate ground center position
    pub fn get_ground_center_y(&self) -> f32 {
        self.y_position - self.half_thickness
    }

    // Creates the ground for Bevy visualization
    pub fn spawn_for_bevy(&self, commands: &mut Commands) {
        let ground_center_y = self.get_ground_center_y();
        
        commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.3, 0.6, 0.3),
                    custom_size: Some(Vec2::new(self.half_width * 2.0, self.half_thickness * 2.0)),
                    ..default()
                },
                transform: Transform::from_xyz(0.0, ground_center_y, -0.1),
                ..default()
            },
            RigidBody::Fixed,
            Collider::cuboid(self.half_width, self.half_thickness),
            Friction::coefficient(self.friction),
            Restitution::coefficient(self.restitution),
            CollisionGroups::new(
                Group::from_bits_truncate(GROUP_GROUND), 
                Group::from_bits_truncate(GROUND_FILTER)
            ),
            Name::new("Ground"),
        ));
    }
}

// === Vehicle Builder ===
pub struct VehicleDefinition {
    pub chromosome: Chromosome,
    pub initial_height: f32, // This comes from SimulationConfig effectively
    // Fields now derived directly from SimulationConfig inside methods or passed during spawning
    // pub damping: f32, // Will use config.vehicle_damping_linear
    // pub angular_damping: f32, // Will use config.vehicle_damping_angular
    // pub chassis_friction: f32, // Will use config.vehicle_chassis_friction
    // pub wheel_restitution: f32, // Will use config.vehicle_wheel_restitution
    // pub ccd_enabled: bool, // Will use config.ccd_enabled
}

impl VehicleDefinition {
    pub fn new(chromosome: &Chromosome, config: &SimulationConfig) -> Self {
        Self {
            chromosome: chromosome.clone(),
            initial_height: config.initial_height_above_ground,
            // Damping, friction, restitution will be applied during spawn_with_customizers from config
            // ccd_enabled will be applied during spawn_with_customizers from config
        }
    }

    // Calculate the initial chassis position
    pub fn get_initial_chassis_position(&self) -> (f32, f32) {
        let initial_chassis_x = 0.0;
        let initial_chassis_y = self.initial_height + self.chromosome.chassis.height / 2.0;
        (initial_chassis_x, initial_chassis_y)
    }

    /// Spawns the vehicle (chassis and wheels) as Bevy entities with core physics components.
    /// Allows customization through closures for adding context-specific components.
    pub fn spawn_with_customizers<'w, 's, 'a, FChassis, FWheel>(
        &self,
        commands: &'a mut Commands<'w, 's>,
        config: &SimulationConfig, // ADDED: Pass SimulationConfig here
        mut chassis_customizer: FChassis,
        mut wheel_customizer: FWheel,
    ) -> Entity
    where
        FChassis: FnMut(&mut EntityCommands),
        FWheel: FnMut(&mut EntityCommands, usize /*wheel_idx*/, &WheelGenes),
    {
        let (initial_chassis_x, initial_chassis_y) = self.get_initial_chassis_position();
        
        // ADDED: Diagnostic print
        println!(
            "[Physics::spawn_with_customizers] Spawning chassis. Config initial_height: {}, Calculated initial_pos: ({}, {}), Chassis height: {}", 
            self.initial_height, 
            initial_chassis_x, 
            initial_chassis_y,
            self.chromosome.chassis.height
        );

        let chassis_genes = &self.chromosome.chassis;

        let mut chassis_entity_builder = commands.spawn_empty();
        chassis_entity_builder
            .insert(RigidBody::Dynamic)
            .insert(Collider::cuboid(chassis_genes.width / 2.0, chassis_genes.height / 2.0))
            .insert(ColliderMassProperties::Density(chassis_genes.density))
            // MODIFIED: Use values from SimulationConfig
            .insert(Friction::coefficient(config.vehicle_chassis_friction))
            .insert(Damping { linear_damping: config.vehicle_damping_linear, angular_damping: config.vehicle_damping_angular })
            .insert(CollisionGroups::new(
                Group::from_bits_truncate(GROUP_VEHICLE), 
                Group::from_bits_truncate(VEHICLE_FILTER)
            ))
            .insert(ActiveEvents::COLLISION_EVENTS); // For collision detection if needed
            // .insert(TransformBundle::from(Transform::from_xyz(initial_chassis_x, initial_chassis_y, 0.0)));
        
        // Apply CCD if enabled in config
        if config.ccd_enabled {
            chassis_entity_builder.insert(Ccd::enabled());
        }
        // Insert TransformBundle after potential Ccd insertion to avoid issues with bundle overwriting Ccd.
        chassis_entity_builder.insert(TransformBundle::from(Transform::from_xyz(initial_chassis_x, initial_chassis_y, 0.0)));

        let chassis_entity = chassis_entity_builder.id();
        
        // Apply chassis customizer
        let mut chassis_entity_commands = commands.entity(chassis_entity);
        chassis_customizer(&mut chassis_entity_commands);

        for (wheel_idx, wheel_gene) in self.chromosome.wheels.iter().enumerate() {
            if !wheel_gene.active {
                continue;
            }

            let wheel_x_rel = wheel_gene.get_x_position(self.chromosome.chassis.width);
            let wheel_y_rel = wheel_gene.get_y_position(self.chromosome.chassis.height);
            
            let wheel_world_x = initial_chassis_x + wheel_x_rel;
            let wheel_world_y = initial_chassis_y + wheel_y_rel;

            // ADDED: Log wheel positioning details
            println!(
                "[Physics] Wheel {} positioning: x_factor={:.2}, y_factor={:.2}, rel_pos=({:.2}, {:.2}), world_pos=({:.2}, {:.2})",
                wheel_idx,
                wheel_gene.x_position_factor,
                wheel_gene.y_position_factor,
                wheel_x_rel,
                wheel_y_rel,
                wheel_world_x,
                wheel_world_y
            );

            let mut wheel_entity_builder = commands.spawn_empty();
            wheel_entity_builder
                .insert(RigidBody::Dynamic)
                .insert(Collider::ball(wheel_gene.radius))
                .insert(ColliderMassProperties::Density(wheel_gene.density))
                .insert(Friction::coefficient(wheel_gene.friction_coefficient))
                // MODIFIED: Use values from SimulationConfig
                .insert(Restitution::coefficient(config.vehicle_wheel_restitution))
                .insert(Damping { linear_damping: config.vehicle_damping_linear, angular_damping: config.vehicle_damping_angular })
                .insert(CollisionGroups::new(
                    Group::from_bits_truncate(GROUP_VEHICLE), 
                    Group::from_bits_truncate(VEHICLE_FILTER)
                ))
                .insert(ExternalImpulse::default());
                // .insert(TransformBundle::from(Transform::from_xyz(wheel_world_x, wheel_world_y, 0.1)));

            // Apply CCD if enabled in config
            if config.ccd_enabled {
                wheel_entity_builder.insert(Ccd::enabled());
            }
            // Insert TransformBundle after potential Ccd insertion.
            wheel_entity_builder.insert(TransformBundle::from(Transform::from_xyz(wheel_world_x, wheel_world_y, 0.1)));

            let wheel_entity = wheel_entity_builder.id();

            // Apply wheel customizer
            let mut wheel_entity_commands = commands.entity(wheel_entity);
            wheel_customizer(&mut wheel_entity_commands, wheel_idx, wheel_gene);
            
            // ADDED: Log entity IDs before joint creation
            println!(
                "[Physics] Wheel {} entities: chassis_entity={:?}, wheel_entity={:?}",
                wheel_idx,
                chassis_entity,
                wheel_entity
            );
            
            // Create joint between chassis and wheel
            // Anchor on chassis is relative to chassis center (which is its transform's translation)
            // Anchor on wheel is relative to wheel center (Vec2::ZERO)
            let mut joint_builder = RevoluteJointBuilder::new()
                .local_anchor1(Vec2::new(wheel_x_rel, wheel_y_rel)) 
                .local_anchor2(Vec2::ZERO);
            
            // ADDED: Log joint anchor configuration
            println!(
                "[Physics] Wheel {} joint anchors: chassis_anchor=({:.2}, {:.2}), wheel_anchor=(0.0, 0.0)",
                wheel_idx,
                wheel_x_rel,
                wheel_y_rel
            );
            
            if wheel_gene.motor_torque.abs() > 1e-9 { // Check if torque is non-negligible
                joint_builder = joint_builder
                    .motor_model(MotorModel::ForceBased)
                    .motor_velocity(if wheel_gene.motor_torque > 0.0 { MOTOR_TARGET_VEL_FORWARD } else { MOTOR_TARGET_VEL_REVERSE }, MOTOR_STIFFNESS_FACTOR)
                    .motor_max_force(wheel_gene.motor_torque.abs());
                
                // ADDED: Log motor configuration for debugging
                println!(
                    "[Physics] Wheel {} motor: torque={:.4}, target_vel={}, stiffness={}, max_force={:.4}",
                    wheel_idx,
                    wheel_gene.motor_torque,
                    if wheel_gene.motor_torque > 0.0 { "f32::MAX (forward)" } else { "f32::MIN (reverse)" },
                    MOTOR_STIFFNESS_FACTOR,
                    wheel_gene.motor_torque.abs()
                );
            }
            
            // ADDED: Log joint creation
            println!(
                "[Physics] Creating joint: chassis_entity={:?} -> wheel_entity={:?}",
                chassis_entity,
                wheel_entity
            );
            
            // FIXED: Attach joint to wheel entity, not chassis entity
            // This allows multiple joints per chassis since each wheel gets its own joint
            commands.entity(wheel_entity).insert(ImpulseJoint::new(chassis_entity, joint_builder.build()));
        }
        
        chassis_entity
    }
} 