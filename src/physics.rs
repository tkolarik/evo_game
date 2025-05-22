use crate::organism::Chromosome;
// Import specific Rapier2D types, including vector and point macros
use rapier2d::prelude::{
    RigidBodyBuilder as HeadlessRigidBodyBuilder,
    ColliderBuilder as HeadlessColliderBuilder,
    RevoluteJointBuilder as HeadlessRevoluteJointBuilder,
    InteractionGroups, ActiveEvents as HeadlessActiveEvents,
    MotorModel as HeadlessMotorModel,
    vector, point,
};
// Import specific Bevy types
use bevy::prelude::*;
// Import specific Bevy Rapier types
use bevy_rapier2d::prelude::{
    RigidBody, Collider, RevoluteJointBuilder, ImpulseJoint,
    Friction, Restitution, Damping, ColliderMassProperties,
    CollisionGroups, Group, Velocity, ReadMassProperties,
    ExternalImpulse, ActiveEvents, MotorModel,
};

// --- Collision Groups (shared across all physics systems) ---
pub const GROUP_VEHICLE: u32 = 1 << 0; // 0b00000001
pub const GROUP_GROUND: u32 = 1 << 1;  // 0b00000010
pub const VEHICLE_FILTER: u32 = GROUP_GROUND; // Vehicles collide with ground
pub const GROUND_FILTER: u32 = GROUP_VEHICLE;   // Ground collides with vehicles

// --- Ground Constants ---
pub const GROUND_Y_POSITION: f32 = -0.15; // Top surface of ground
pub const GROUND_THICKNESS: f32 = 0.1;    // Full thickness of ground

// --- Physics Parameters ---
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

    // Creates the ground for headless simulation
    pub fn create_for_headless(&self) -> (HeadlessRigidBodyBuilder, HeadlessColliderBuilder) {
        // Calculate ground center position
        let ground_center_y = self.get_ground_center_y();
        
        // Create ground rigid body (fixed)
        let body = HeadlessRigidBodyBuilder::fixed()
            .translation(vector![0.0, ground_center_y]);
            
        // Create ground collider
        let collider = HeadlessColliderBuilder::cuboid(self.half_width, self.half_thickness)
            .friction(self.friction)
            .restitution(self.restitution)
            .collision_groups(InteractionGroups::new(GROUP_GROUND.into(), GROUP_VEHICLE.into()));
            
        (body, collider)
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

    // Creates chassis for headless simulation
    pub fn create_chassis_for_headless(&self) -> (HeadlessRigidBodyBuilder, HeadlessColliderBuilder) {
        let (initial_x, initial_y) = self.get_initial_chassis_position();
        
        // Create chassis rigid body
        let body = HeadlessRigidBodyBuilder::dynamic()
            .translation(vector![initial_x, initial_y])
            .linear_damping(self.damping)
            .angular_damping(self.angular_damping)
            .ccd_enabled(false);
            
        // Create chassis collider
        let collider = HeadlessColliderBuilder::cuboid(
                self.chromosome.chassis.width / 2.0,
                self.chromosome.chassis.height / 2.0,
            )
            .density(self.chromosome.chassis.density)
            .friction(self.chassis_friction)
            .collision_groups(InteractionGroups::new(
                GROUP_VEHICLE.into(),
                GROUP_GROUND.into(),
            ))
            .active_events(HeadlessActiveEvents::COLLISION_EVENTS);
            
        (body, collider)
    }
    
    // Spawn chassis for Bevy visualization, returns the entity
    pub fn spawn_chassis_for_bevy(&self, commands: &mut Commands) -> Entity {
        let (initial_x, initial_y) = self.get_initial_chassis_position();
        let chassis_genes = &self.chromosome.chassis;
        
        commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.7, 0.7, 0.8),
                    custom_size: Some(Vec2::new(chassis_genes.width, chassis_genes.height)),
                    ..default()
                },
                transform: Transform::from_xyz(initial_x, initial_y, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(chassis_genes.width / 2.0, chassis_genes.height / 2.0),
            ColliderMassProperties::Density(chassis_genes.density),
            Friction::coefficient(self.chassis_friction),
            Damping { linear_damping: self.damping, angular_damping: self.angular_damping },
            CollisionGroups::new(
                Group::from_bits_truncate(GROUP_VEHICLE), 
                Group::from_bits_truncate(VEHICLE_FILTER)
            ),
            ActiveEvents::COLLISION_EVENTS,
            Name::new("Chassis"),
            crate::visualization::VehiclePart,
            Velocity::default(), 
            ReadMassProperties::default(),
        )).id()
    }

    // Creates wheels for headless simulation
    pub fn create_wheel_for_headless(&self, wheel_idx: usize, initial_chassis_x: f32, initial_chassis_y: f32) 
        -> Option<(HeadlessRigidBodyBuilder, HeadlessColliderBuilder, HeadlessRevoluteJointBuilder)> 
    {
        let wheel_gene = &self.chromosome.wheels[wheel_idx];
        
        if !wheel_gene.active {
            return None;
        }
        
        let wheel_x_abs = wheel_gene.get_x_position(self.chromosome.chassis.width);
        let wheel_y_abs = wheel_gene.get_y_position(self.chromosome.chassis.height);

        // Create wheel rigid body
        let body = HeadlessRigidBodyBuilder::dynamic()
            .translation(vector![
                initial_chassis_x + wheel_x_abs,
                initial_chassis_y + wheel_y_abs
            ])
            .linear_damping(self.damping)
            .angular_damping(self.angular_damping)
            .ccd_enabled(false);
            
        // Create wheel collider
        let collider = HeadlessColliderBuilder::ball(wheel_gene.radius)
            .density(wheel_gene.density)
            .friction(wheel_gene.friction_coefficient)
            .restitution(self.wheel_restitution)
            .collision_groups(InteractionGroups::new(
                GROUP_VEHICLE.into(),
                GROUP_GROUND.into(),
            ))
            .active_events(HeadlessActiveEvents::COLLISION_EVENTS);
            
        // Create joint
        let chassis_width = self.chromosome.chassis.width;
        let chassis_height = self.chromosome.chassis.height;

        let anchor_on_chassis = point![
            wheel_gene.x_position_factor * chassis_width / 2.0,
            wheel_gene.y_position_factor * chassis_height / 2.0
        ];

        let mut joint_builder = HeadlessRevoluteJointBuilder::new()
            .local_anchor1(anchor_on_chassis) // Anchor on chassis relative to its center
            .local_anchor2(point![0.0, 0.0]); // Anchor on wheel (center of wheel)
        
        if wheel_gene.motor_torque != 0.0 {
            joint_builder = joint_builder
                .motor_model(HeadlessMotorModel::ForceBased)
                .motor_velocity(if wheel_gene.motor_torque > 0.0 { f32::MAX } else { f32::MIN }, 1.0)
                .motor_max_force(wheel_gene.motor_torque.abs());
        }
        
        Some((body, collider, joint_builder))
    }
    
    // Spawn wheel for Bevy visualization, returns the wheel entity
    pub fn spawn_wheel_for_bevy(&self, commands: &mut Commands, chassis_entity: Entity, wheel_idx: usize) -> Option<Entity> {
        let wheel_gene = &self.chromosome.wheels[wheel_idx];
        
        if !wheel_gene.active {
            return None;
        }
        
        let (initial_chassis_x, initial_chassis_y) = self.get_initial_chassis_position();
        let wheel_x_abs = wheel_gene.get_x_position(self.chromosome.chassis.width);
        let wheel_y_abs = wheel_gene.get_y_position(self.chromosome.chassis.height);
        
        // Determine wheel color based on motor torque
        let wheel_color = if wheel_gene.motor_torque > 0.0 { 
            Color::rgb(0.8, 0.3, 0.3) 
        } else if wheel_gene.motor_torque < 0.0 { 
            Color::rgb(0.3, 0.3, 0.8) 
        } else { 
            Color::rgb(0.5, 0.5, 0.5) 
        };
        
        // Spawn wheel entity
        let wheel_entity = commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: wheel_color,
                    custom_size: Some(Vec2::new(wheel_gene.radius * 2.0, wheel_gene.radius * 2.0)),
                    ..default()
                },
                transform: Transform::from_xyz(
                    initial_chassis_x + wheel_x_abs, 
                    initial_chassis_y + wheel_y_abs, 
                    0.1
                ),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::ball(wheel_gene.radius),
            ColliderMassProperties::Density(wheel_gene.density),
            Friction::coefficient(wheel_gene.friction_coefficient),
            Restitution::coefficient(self.wheel_restitution),
            Damping { linear_damping: self.damping, angular_damping: self.angular_damping },
            CollisionGroups::new(
                Group::from_bits_truncate(GROUP_VEHICLE), 
                Group::from_bits_truncate(VEHICLE_FILTER)
            ),
            ExternalImpulse::default(),
            Name::new("Wheel"),
            crate::visualization::VehiclePart,
            crate::visualization::VisualizedWheel { motor_gene_torque: wheel_gene.motor_torque },
            Velocity::default(),
        )).id();
        
        // Create joint between chassis and wheel
        let mut joint_builder = RevoluteJointBuilder::new()
            .local_anchor1(Vec2::new(wheel_x_abs, wheel_y_abs))
            .local_anchor2(Vec2::new(0.0, 0.0));
        
        if wheel_gene.motor_torque != 0.0 {
            joint_builder = joint_builder
                .motor_model(MotorModel::ForceBased)
                .motor_velocity(if wheel_gene.motor_torque > 0.0 { f32::MAX } else { f32::MIN }, 1.0)
                .motor_max_force(wheel_gene.motor_torque.abs());
        }
        
        commands.entity(chassis_entity).insert(ImpulseJoint::new(wheel_entity, joint_builder.build()));
        
        // Add wheel rotation indicator
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
        
        Some(wheel_entity)
    }
} 