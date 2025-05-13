use bevy::prelude::*;
use bevy_rapier2d::prelude::*;
use crate::organism::Chromosome;
use crate::simulation::SimulationConfig;

// Marker component for the main camera
#[derive(Component)]
pub struct MainCamera;

// Component to tag entities that represent a simulated vehicle part
#[derive(Component)]
pub struct VehiclePart;

// Resource to hold the chromosome of the vehicle currently being visualized
#[derive(Resource, Default)]
pub struct CurrentVehicleChromosome(pub Option<Chromosome>);

// Resource to control simulation state for visualization
#[derive(Resource)]
pub struct VisualizationState {
    pub is_simulating_current_vehicle: bool,
    pub current_simulation_time: f32,
    pub show_best_overall: bool, // Flag to show all-time best vs current gen best
    // Add other state controls like simulation speed factor
}

impl Default for VisualizationState {
    fn default() -> Self {
        Self {
            is_simulating_current_vehicle: false,
            current_simulation_time: 0.0,
            show_best_overall: false,
        }
    }
}

// --- Systems ---

pub fn setup_graphics(mut commands: Commands) {
    commands.spawn((
        Camera2dBundle::default(),
        MainCamera,
    ));
    println!("Bevy camera spawned.");
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
// This will be expanded significantly
pub fn spawn_vehicle_visualization(
    mut commands: Commands,
    config: Res<SimulationConfig>,
    chromosome_res: Res<CurrentVehicleChromosome>,
    mut rapier_config: ResMut<RapierConfiguration>,
) {
    if let Some(chromosome) = &chromosome_res.0 {
        println!("Spawning vehicle visualization for chromosome: {:?}", chromosome.chassis.width);
        // Set gravity for this specific visualization run if needed (usually global)
        // rapier_config.gravity = Vector::new(0.0, config.gravity);

        // Placeholder: Just draw a rectangle for the chassis for now
        // Actual vehicle spawning from chromosome will be more complex, mirroring PhysicsWorld logic
        // but using Bevy entities and Rapier components.
        
        let initial_chassis_y = config.initial_height_above_ground + chromosome.chassis.height / 2.0;

        commands.spawn((
            SpriteBundle {
                sprite: Sprite {
                    color: Color::rgb(0.7, 0.7, 0.8),
                    custom_size: Some(Vec2::new(chromosome.chassis.width, chromosome.chassis.height)),
                    ..default()
                },
                transform: Transform::from_xyz(0.0, initial_chassis_y, 0.0),
                ..default()
            },
            RigidBody::Dynamic,
            Collider::cuboid(chromosome.chassis.width / 2.0, chromosome.chassis.height / 2.0),
            VehiclePart,
            // Note: We'd also add joints and wheels here for a full visualization
        ));
        println!("Spawned placeholder chassis for visualization.");
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
    config: Res<SimulationConfig>,
    time: Res<Time>,
    // query for vehicle parts if we need to apply motor torques directly here
) {
    if vis_state.is_simulating_current_vehicle {
        vis_state.current_simulation_time += time.delta_seconds();
        // Apply motor torques to visualized vehicle parts here
        // ... physics system handles the stepping ...

        if vis_state.current_simulation_time >= config.sim_duration_secs {
            vis_state.is_simulating_current_vehicle = false;
            vis_state.current_simulation_time = 0.0;
            println!("Visualized simulation step finished.");
            // Potentially trigger an event to signal completion
        }
    }
} 