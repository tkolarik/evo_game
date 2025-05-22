use crate::organism::Chromosome;
use crate::simulation::HeadlessChassis; // Added import for HeadlessChassis
// Import specific Bevy types
use bevy::prelude::*;
// Import specific Bevy Rapier types
use bevy_rapier2d::prelude::{
    RigidBody, Collider, RevoluteJointBuilder, ImpulseJoint,
    Friction, Restitution, Damping, ColliderMassProperties,
    CollisionGroups, Group, ActiveEvents, MotorModel, ExternalImpulse,
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

// --- Physics Parameters ---
#[derive(Resource, Clone, Debug)]
pub struct PhysicsParameters {
    pub gravity: f32,
    pub ground_friction: f32,
    pub ground_restitution: f32,
    pub erp: f32,
    pub joint_erp: f32,
    pub solver_iterations: usize,
}

impl Default for PhysicsParameters {
    fn default() -> Self {
        Self {
            gravity: -9.81,
            ground_friction: 1.0,
            ground_restitution: 0.0,
            erp: 0.01,             // Unified from previous testing
            joint_erp: 0.01,       // Unified from previous testing
            solver_iterations: 4,  // Rapier default
        }
    }
}

// === Ground Builder ===
pub struct GroundDefinition {
    pub half_width: f32,
    pub half_thickness: f32,
    pub y_position: f32,
    pub friction: f32,
    pub restitution: f32,
}

impl Default for GroundDefinition {
    fn default() -> Self {
        Self {
            half_width: 1000.0,
            half_thickness: GROUND_THICKNESS / 2.0,
            y_position: GROUND_Y_POSITION,
            friction: 1.0,
            restitution: 0.0,
        }
    }
}

impl GroundDefinition {
    pub fn new(params: &PhysicsParameters) -> Self {
        Self {
            half_width: 1000.0,
            half_thickness: GROUND_THICKNESS / 2.0,
            y_position: GROUND_Y_POSITION,
            friction: params.ground_friction,
            restitution: params.ground_restitution,
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
    pub initial_height: f32,
    pub damping: f32,
    pub angular_damping: f32,
    pub chassis_friction: f32,
    pub wheel_restitution: f32,
}

impl VehicleDefinition {
    pub fn new(chromosome: &Chromosome, initial_height: f32) -> Self {
        Self {
            chromosome: chromosome.clone(),
            initial_height,
            damping: 0.5,             // Default damping value
            angular_damping: 0.5,     // Default angular damping
            chassis_friction: 0.7,    // Default chassis friction
            wheel_restitution: 0.1,   // Default wheel restitution
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
        mut chassis_customizer: FChassis,
        mut wheel_customizer: FWheel,
    ) -> Entity
    where
        FChassis: FnMut(&mut EntityCommands),
        FWheel: FnMut(&mut EntityCommands, usize /*wheel_idx*/, &WheelGenes),
    {
        println!("[DEBUG Physics] spawn_with_customizers called. Chassis width: {}", self.chromosome.chassis.width);

        let (initial_chassis_x, initial_chassis_y) = self.get_initial_chassis_position();
        let chassis_genes = &self.chromosome.chassis;

        let chassis_entity = commands.spawn_empty()
            .insert(RigidBody::Dynamic)
            .insert(Collider::cuboid(chassis_genes.width / 2.0, chassis_genes.height / 2.0))
            .insert(ColliderMassProperties::Density(chassis_genes.density))
            .insert(Friction::coefficient(self.chassis_friction))
            .insert(Damping { linear_damping: self.damping, angular_damping: self.angular_damping })
            .insert(CollisionGroups::new(
                Group::from_bits_truncate(GROUP_VEHICLE), 
                Group::from_bits_truncate(VEHICLE_FILTER)
            ))
            .insert(ActiveEvents::COLLISION_EVENTS) // For collision detection if needed
            .insert(TransformBundle::from(Transform::from_xyz(initial_chassis_x, initial_chassis_y, 0.0)))
            .id();
        
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

            let wheel_entity = commands.spawn_empty()
                .insert(RigidBody::Dynamic)
                .insert(Collider::ball(wheel_gene.radius))
                .insert(ColliderMassProperties::Density(wheel_gene.density))
                .insert(Friction::coefficient(wheel_gene.friction_coefficient))
                .insert(Restitution::coefficient(self.wheel_restitution))
                .insert(Damping { linear_damping: self.damping, angular_damping: self.angular_damping })
                .insert(CollisionGroups::new(
                    Group::from_bits_truncate(GROUP_VEHICLE), 
                    Group::from_bits_truncate(VEHICLE_FILTER)
                ))
                .insert(ExternalImpulse::default()) // For applying motor torque if needed (though joints are primary)
                .insert(TransformBundle::from(Transform::from_xyz(wheel_world_x, wheel_world_y, 0.1))) // Wheels slightly in front
                .id();

            // Apply wheel customizer
            let mut wheel_entity_commands = commands.entity(wheel_entity);
            wheel_customizer(&mut wheel_entity_commands, wheel_idx, wheel_gene);
            
            // Create joint between chassis and wheel
            // Anchor on chassis is relative to chassis center (which is its transform's translation)
            // Anchor on wheel is relative to wheel center (Vec2::ZERO)
            let mut joint_builder = RevoluteJointBuilder::new()
                .local_anchor1(Vec2::new(wheel_x_rel, wheel_y_rel)) 
                .local_anchor2(Vec2::ZERO);
            
            if wheel_gene.motor_torque.abs() > 1e-9 { // Check if torque is non-negligible
                joint_builder = joint_builder
                    .motor_model(MotorModel::ForceBased)
                    .motor_velocity(if wheel_gene.motor_torque > 0.0 { MOTOR_TARGET_VEL_FORWARD } else { MOTOR_TARGET_VEL_REVERSE }, MOTOR_STIFFNESS_FACTOR)
                    .motor_max_force(wheel_gene.motor_torque.abs());
            }
            
            commands.entity(chassis_entity).insert(ImpulseJoint::new(wheel_entity, joint_builder.build()));
        }
        
        chassis_entity
    }
} 