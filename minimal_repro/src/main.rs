use bevy::prelude::*;
use bevy_rapier2d::prelude::*;

// Simulation parameters (mirroring the main project for consistency)
const GRAVITY: f32 = -9.81;
const TIME_STEP: f32 = 1.0 / 60.0;
const ERP: f32 = 0.01;
const JOINT_ERP: f32 = 0.01;

// Chassis properties
const CHASSIS_WIDTH: f32 = 2.0;
const CHASSIS_HEIGHT: f32 = 1.0;
const CHASSIS_DENSITY: f32 = 1.0;
const CHASSIS_INITIAL_Y_OFFSET: f32 = 1.0; // Initial height above ground (to center)

// Wheel properties
const WHEEL_RADIUS: f32 = 0.5;
const WHEEL_DENSITY: f32 = 1.0;
// Relative anchor position on chassis for the wheel
const WHEEL_ANCHOR_X_FACTOR: f32 = 0.0; // Centered X
const WHEEL_ANCHOR_Y_FACTOR: f32 = -1.0; // Bottom surface of chassis

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(RapierPhysicsPlugin::<NoUserData>::pixels_per_meter(100.0)) // Using pixels_per_meter for consistency if needed, though 1.0 is also fine if units are meters.
        // .add_plugins(RapierDebugRenderPlugin::default()) // Optional: for visual debugging
        .insert_resource(Time::<Fixed>::from_seconds(TIME_STEP as f64))
        .init_resource::<LogStep>()
        .add_systems(Startup, (setup_physics, setup_scene).chain())
        .add_systems(FixedUpdate, log_physics_state)
        .run();
}

fn setup_physics(mut rapier_config: ResMut<RapierConfiguration>, mut rapier_context: ResMut<RapierContext>) {
    rapier_config.gravity = Vec2::new(0.0, GRAVITY);
    
    // Configure integration parameters directly on RapierContext
    let context = rapier_context.as_mut();
    context.integration_parameters.dt = TIME_STEP;
    context.integration_parameters.erp = ERP;
    context.integration_parameters.joint_erp = JOINT_ERP;
    // num_solver_iterations defaults to 4 in RapierConfiguration, which is consistent.
    // num_additional_friction_iterations also defaults to 4 in Rapier's IntegrationParameters.

    println!(
        "Minimal Repro: Rapier integration parameters configured: dt={}, erp={}, joint_erp={}",
        context.integration_parameters.dt,
        context.integration_parameters.erp,
        context.integration_parameters.joint_erp
    );
}

fn setup_scene(mut commands: Commands) {
    // Calculate initial positions carefully
    let initial_chassis_y_world = CHASSIS_INITIAL_Y_OFFSET + CHASSIS_HEIGHT / 2.0;
    
    let wheel_x_abs_on_chassis = WHEEL_ANCHOR_X_FACTOR * (CHASSIS_WIDTH / 2.0);
    let wheel_y_abs_on_chassis = WHEEL_ANCHOR_Y_FACTOR * (CHASSIS_HEIGHT / 2.0);

    // Initial world position for the wheel's center
    let initial_wheel_x_world = 0.0 + wheel_x_abs_on_chassis;
    let initial_wheel_y_world = initial_chassis_y_world + wheel_y_abs_on_chassis;

    // Spawn Chassis
    let chassis_entity = commands.spawn((
        Name::new("Chassis"),
        TransformBundle::from(Transform::from_xyz(0.0, initial_chassis_y_world, 0.0)),
        RigidBody::Dynamic,
        Collider::cuboid(CHASSIS_WIDTH / 2.0, CHASSIS_HEIGHT / 2.0),
        ColliderMassProperties::Density(CHASSIS_DENSITY),
        Damping { linear_damping: 0.5, angular_damping: 0.5 },
        Friction::coefficient(0.7),
        // ActiveEvents::COLLISION_EVENTS, // Keep this off for minimal case first
        Velocity::default(), // For logging
    )).id();

    // Spawn Wheel
    let wheel_entity = commands.spawn((
        Name::new("Wheel"),
        TransformBundle::from(Transform::from_xyz(initial_wheel_x_world, initial_wheel_y_world, 0.0)),
        RigidBody::Dynamic,
        Collider::ball(WHEEL_RADIUS),
        ColliderMassProperties::Density(WHEEL_DENSITY),
        Damping { linear_damping: 0.5, angular_damping: 0.5 },
        Friction::coefficient(1.0),
        Restitution::coefficient(0.1),
        // ActiveEvents::COLLISION_EVENTS, // Keep this off for minimal case first
        Velocity::default(), // For logging
    )).id();

    // Add Joint
    // Anchors are relative to the center of each body
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(Vec2::new(wheel_x_abs_on_chassis, wheel_y_abs_on_chassis)) // On chassis
        .local_anchor2(Vec2::new(0.0, 0.0)); // On wheel (its center)
    
    commands.entity(chassis_entity).insert(ImpulseJoint::new(wheel_entity, joint));

    println!("Minimal Repro: Scene setup complete.");
    println!("  Chassis spawned at: (0.0, {:.4})", initial_chassis_y_world);
    println!("  Wheel spawned at: ({:.4}, {:.4})", initial_wheel_x_world, initial_wheel_y_world);
    println!("  Joint anchor on chassis (local): ({:.4}, {:.4})", wheel_x_abs_on_chassis, wheel_y_abs_on_chassis);

}

#[derive(Resource, Default)]
struct LogStep(usize);

fn log_physics_state(
    mut step: ResMut<LogStep>,
    query: Query<(Entity, &Name, &Transform, &Velocity), With<RigidBody>>,
) {
    if step.0 > 10 { // Log for a few steps only
        return;
    }

    println!("--- Minimal Repro Log: Step {} ---", step.0);
    for (entity, name, transform, velocity) in query.iter() {
        println!(
            "  Entity: {:?}, Name: {}, Pos: ({:.4}, {:.4}), Linvel: ({:.4}, {:.4}), Angvel: {:.4}",
            entity,
            name,
            transform.translation.x,
            transform.translation.y,
            velocity.linvel.x,
            velocity.linvel.y,
            velocity.angvel
        );
    }
    step.0 += 1;
} 