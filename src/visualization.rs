use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use crate::organism::*;
use crate::DistanceComparisonData;
use crate::simulation::SimulationConfig;
use crate::physics::{VehicleDefinition};

// Marker component for the main camera
#[derive(Component)]
pub struct MainCamera;

// Component to tag entities that represent a simulated vehicle part
#[derive(Component)]
pub struct VehiclePart;

// New component to mark visualized wheels and store their gene torque
#[derive(Component)]
pub struct VisualizedWheel {
    pub motor_gene_torque: f32,
}

// Resource to hold the chromosome of the vehicle currently being visualized
#[derive(Resource, Default)]
pub struct CurrentVehicleChromosome(pub Option<Chromosome>);

// Resource to control simulation state for visualization
#[derive(Resource)]
pub struct VisualizationState {
    pub is_simulating_current_vehicle: bool,
    pub current_simulation_time: f32,
    pub show_best_overall: bool, // Flag to show all-time best vs current gen best
    pub is_physics_paused: bool, // New flag: true to pause Rapier physics during visualization
    pub single_step_requested: bool, // New flag: true to advance physics by one tick when paused
    pub use_debug_vehicle: bool, // New flag: true to use a hardcoded debug vehicle
    // For distance tracking and comparison
    pub initial_chassis_position: Option<Vec2>,
    pub expected_fitness: Option<f32>,
    // Add other state controls like simulation speed factor
}

impl Default for VisualizationState {
    fn default() -> Self {
        Self {
            is_simulating_current_vehicle: false,
            current_simulation_time: 0.0,
            show_best_overall: false,
            is_physics_paused: false, // Default to not paused
            single_step_requested: false, // Default to no single step requested
            use_debug_vehicle: false, // Default to not using debug vehicle
            initial_chassis_position: None,
            expected_fitness: None,
        }
    }
}

// --- Systems ---

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera2dBundle {
            transform: Transform::from_xyz(0.0, 0.0, 999.0), // Ensure Z is far enough
            projection: OrthographicProjection {
                scale: 0.03, // Smaller scale = more zoom.
                ..default()
            },
            ..default()
        },
        MainCamera,
    ));
    println!("Bevy camera spawned with more zoom.");
}

// System to despawn old vehicle parts before simulating a new one
pub fn despawn_vehicle_parts(
    mut commands: Commands,
    vehicle_parts_query: Query<Entity, With<VehiclePart>>,
) {
    for entity in vehicle_parts_query.iter() {
        commands.entity(entity).despawn_recursive();
    }
}

// System to spawn the visual representation of the current vehicle
pub fn spawn_vehicle_visualization(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    chromosome_res: Res<CurrentVehicleChromosome>,
    mut vis_state: ResMut<VisualizationState>,
) {
    if let Some(chromosome) = &chromosome_res.0 {
        println!("Spawning vehicle visualization for chromosome: {:?}", chromosome.chassis.width);

        let vehicle_def = VehicleDefinition::new(
            chromosome, 
            config.initial_height_above_ground
        );

        let (initial_x, initial_y) = vehicle_def.get_initial_chassis_position();
        vis_state.initial_chassis_position = Some(Vec2::new(initial_x, initial_y));

        let _chassis_entity = vehicle_def.spawn_with_customizers(
            &mut commands,
            |chassis_cmds| {
                chassis_cmds
                    .insert(SpriteBundle {
                        sprite: Sprite {
                            color: Color::rgb(0.7, 0.7, 0.8),
                            custom_size: Some(Vec2::new(vehicle_def.chromosome.chassis.width, vehicle_def.chromosome.chassis.height)),
                            ..default()
                        },
                        ..default()
                    })
                    .insert(VehiclePart)
                    .insert(Name::new("Chassis"))
                    .insert(Velocity::default())
                    .insert(ReadMassProperties::default());
            },
            |wheel_cmds, wheel_idx, wheel_gene| {
                let wheel_color = if wheel_gene.motor_torque > 0.0 { 
                    Color::rgb(0.8, 0.3, 0.3) 
                } else if wheel_gene.motor_torque < 0.0 { 
                    Color::rgb(0.3, 0.3, 0.8) 
                } else { 
                    Color::rgb(0.5, 0.5, 0.5) 
                };

                wheel_cmds
                    .insert(SpriteBundle {
                        sprite: Sprite {
                            color: wheel_color,
                            custom_size: Some(Vec2::new(wheel_gene.radius * 2.0, wheel_gene.radius * 2.0)),
                            ..default()
                        },
                        ..default()
                    })
                    .insert(VehiclePart)
                    .insert(VisualizedWheel { motor_gene_torque: wheel_gene.motor_torque })
                    .insert(Name::new(format!("Wheel {}", wheel_idx)))
                    .insert(Velocity::default());
                
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
            }
        );
        
        println!("Spawned vehicle for visualization using centralized spawner.");

    } else {
        println!("No chromosome available in CurrentVehicleChromosome resource for visualization.");
    }
}

// Placeholder for UI system
pub fn ui_system(
    // mut contexts: EguiContexts, // If using bevy_egui
    // evolution_engine: Res<EvolutionEngine>, // To display stats
) {
    // Example: egui::Window::new("Evolutionary Progress").show(contexts.ctx_mut(), |ui| {
    //     ui.label(format!("Generation: {}", evolution_engine.population.generation_count));
    //     // Add more stats
    // });
}

// System to run one step of the visualized simulation
pub fn step_vehicle_visualization_simulation(
    mut vis_state: ResMut<VisualizationState>,
    _config: Res<SimulationConfig>,
    time: Res<Time>,
    chassis_query: Query<(Entity, &Transform), (With<VehiclePart>, Without<VisualizedWheel>)>,
    all_visualized_wheels_query: Query<(Entity, &Name, &Transform), With<VisualizedWheel>>,
) {
    if vis_state.is_simulating_current_vehicle {
        // Log physics state (optional, can be made conditional)
        // Consider adding a flag to vis_state to control this detailed logging
        if vis_state.current_simulation_time % 1.0 < time.delta_seconds() { // Log roughly once per second
            println!("=== VISUALIZATION PHYSICS STATE (Time: {:.2}s) ===", vis_state.current_simulation_time);
            for (entity, transform) in chassis_query.iter() {
                println!("Chassis entity {entity:?}, Transform: {}", transform.translation);
            }
            let collected_wheels: Vec<_> = all_visualized_wheels_query.iter().collect();
            println!("Logging all collected wheels ({}): ", collected_wheels.len());
            for (entity, name, transform) in collected_wheels.iter() {
                println!("Wheel entity {entity:?} with name: {}, Transform: {}", name, transform.translation);
            }
            println!("=== END VISUALIZATION PHYSICS STATE ===");
        }
        
        // Only update simulation time if physics is not paused or we're doing a single step
        if !vis_state.is_physics_paused || vis_state.single_step_requested {
            // Motor torque application is now handled by Rapier's joint motors,
            // configured during vehicle spawning.
            // No need to manually apply ExternalImpulse here.
            
            // Increment simulation time if physics is running
            vis_state.current_simulation_time += time.delta_seconds();
            
            // If a single step was requested, clear the request
            if vis_state.single_step_requested {
                vis_state.single_step_requested = false;
            }
        }
    }
}

// System to make the camera follow the visualized vehicle
pub fn camera_follow_vehicle(
    mut camera_query: Query<(&mut Transform, &mut OrthographicProjection), (With<MainCamera>, Without<VehiclePart>)>,
    vehicle_parts_query: Query<(&Transform, Option<&Name>), With<VehiclePart>>,
) {
    if let Ok((mut camera_transform, mut projection)) = camera_query.get_single_mut() {
        // Find the farthest x position and the center of mass (chassis)
        let mut farthest_x = f32::NEG_INFINITY;
        let mut chassis_pos = Vec2::ZERO;
        let mut chassis_found = false;
        
        // First pass to find chassis by name
        for (part_transform, name) in vehicle_parts_query.iter() {
            // Try to identify chassis by name (or use first entity as fallback)
            if !chassis_found {
                if let Some(name) = name {
                    if name.as_str().contains("Chassis") {
                        chassis_pos = Vec2::new(part_transform.translation.x, part_transform.translation.y);
                        chassis_found = true;
                    }
                } else {
                    // Fallback: use first entity if no name found
                    chassis_pos = Vec2::new(part_transform.translation.x, part_transform.translation.y);
                    chassis_found = true;
                }
            }
            
            // Track farthest piece
            if part_transform.translation.x > farthest_x {
                farthest_x = part_transform.translation.x;
            }
        }
        
        if chassis_found {
            // Calculate the midpoint between chassis and farthest piece
            let midpoint_x = (chassis_pos.x + farthest_x) / 2.0;
            
            // Position camera at the midpoint
            camera_transform.translation.x = midpoint_x;
            camera_transform.translation.y = chassis_pos.y;
            
            // Calculate the distance between chassis and farthest piece to determine zoom
            let distance = (farthest_x - chassis_pos.x).abs();
            
            // Adjust zoom level based on distance (with min/max limits)
            if distance > 5.0 {
                // The larger the distance, the more we need to zoom out (larger scale)
                let desired_scale = (distance / 15.0).max(0.03).min(0.2);
                // Smooth the scale transition
                projection.scale = projection.scale * 0.9 + desired_scale * 0.1;
            } else {
                // Default zoom for normal scenarios
                projection.scale = projection.scale * 0.9 + 0.03 * 0.1;
            }
        }
    }
}

// New system to track and compare distances
pub fn track_vehicle_distance(
    time: Res<Time>,
    vis_state: Res<VisualizationState>,
    chassis_query: Query<(&Transform, &Name), With<VehiclePart>>,
    mut distance_data: ResMut<DistanceComparisonData>,
) {
    // Only check every ~2 seconds to avoid spamming the console
    if !vis_state.is_simulating_current_vehicle || 
       vis_state.expected_fitness.is_none() {
        return;
    }
    
    // Find the chassis by name
    for (transform, name) in chassis_query.iter() {
        if name.as_str().contains("Chassis") {
            if let (Some(initial_pos), Some(expected)) = (vis_state.initial_chassis_position, vis_state.expected_fitness) {
                let current_pos = Vec2::new(transform.translation.x, transform.translation.y);
                
                // The simulation and visualization coordinate systems differ by their initial position
                // So we need to measure from the initial position in the visualization
                let visualized_distance = current_pos.x - initial_pos.x;
                
                // Compare with the expected distance from simulation
                let delta = visualized_distance - expected;
                let delta_percent = if expected != 0.0 { (delta / expected) * 100.0 } else { 0.0 };
                
                // Update the distance comparison data resource for UI display
                distance_data.simulated = Some(expected);
                distance_data.visualized = Some(visualized_distance);
                
                // Only log every ~2 seconds to avoid spamming the console
                if time.elapsed_seconds() % 2.0 < 0.1 {
                    println!(
                        "Distance check at t={:.1}s: Simulated={:.2}, Visualized={:.2}, Delta={:.2} ({:.1}%)",
                        vis_state.current_simulation_time, expected, visualized_distance, delta, delta_percent
                    );
                }
            }
            break; // Only need to check the first chassis found
        }
    }
} 