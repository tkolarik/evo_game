Epic: Achieve Physics Consistency: Headless vs. Visualization
Goal: Ensure that a given vehicle chromosome produces quantitatively identical (or within a very tight, defined tolerance) physical outcomes (e.g., distance traveled, final orientation, contact points) in both the headless simulation.rs and the Bevy-Rapier main.rs visualization over a fixed duration.

Phase 1: Configuration & Parameter Audit and Synchronization 🛠️
Ticket 1.1: Centralize and Verify Core Physics Parameters

Description: Ensure all fundamental physics parameters are identically defined and accessed by both simulation modes.
Sub-Tasks:
[ ] 1.1.1: Review SimulationConfig. Identify every parameter that directly or indirectly influences Rapier2D's behavior (e.g., gravity, ground_friction, sim_duration_secs).
[ ] 1.1.2: Verify that main.rs (Bevy/Rapier visualization) and simulation.rs (headless Rapier) are both using these exact same values from a single source of truth (likely SimulationConfig resource).
[ ] 1.1.3: CRITICAL: Double-check gravity. In main.rs, it's rapier_config.gravity = Vec2::new(0.0, config.gravity);. In simulation.rs, it's gravity: vector![0.0, config.gravity], and then physics_pipeline.step(&self.gravity, ...). These look consistent but confirm no other gravity settings are interfering (e.g., GravityScale component on ground/vehicle if not intended).
[ ] 1.1.4: CRITICAL: Confirm ground_friction. main.rs uses Friction::coefficient(config.ground_friction) for the visual ground. simulation.rs uses ColliderBuilder::cuboid(...).friction(self.ground_friction) for the headless ground. Ensure self.ground_friction in PhysicsWorld is indeed initialized from config.ground_friction.
Acceptance Criteria: All shared physics parameters are loaded from SimulationConfig and demonstrably identical at the point of use in both modes.
Ticket 1.2: Audit Rapier2D IntegrationParameters & Timestepping

Description: Ensure timestepping and Rapier2D's IntegrationParameters are identical.
Sub-Tasks:
[ ] 1.2.1: In simulation.rs, IntegrationParameters.dt is set to 1.0 / 60.0.
[ ] 1.2.2: In main.rs, Bevy is configured for a fixed timestep: insert_resource(Time::<Fixed>::from_seconds(1.0 / 60.0)). Confirm the Bevy Rapier plugin correctly uses this Time<Fixed> for its steps. (This is usually the default behavior for RapierPhysicsPlugin::pixels_per_meter).
[ ] 1.2.3: Compare all other fields of IntegrationParameters (e.g., erp, damping_ratio, solver iteration counts if set here) used in simulation.rs versus the defaults or settings applied by RapierPhysicsPlugin in main.rs. If RapierConfiguration in Bevy has equivalent settings (like solver counts), they must match.
Acceptance Criteria: dt is 1/60s in both. All other integration parameters are explicitly set to be identical or confirmed to default to identical values.
Ticket 1.3: Standardize Physics Object Initialization Logic

Description: Refactor or closely align the code that creates RigidBody and Collider for vehicles (chassis and wheels) so it's as identical as possible between simulation.rs and main.rs/visualization.rs.
Sub-Tasks:
[ ] 1.3.1: Create a shared helper function/module or meticulously compare line-by-line the creation of RigidBodyDesc / RigidBodyBuilder and ColliderDesc / ColliderBuilder for the chassis in both modes.
[ ] 1.3.2: Do the same for the wheels.
[ ] 1.3.3: Pay extreme attention to:
Initial position and orientation (see initial_height_above_ground).
Collider::cuboid and Collider::ball dimensions.
ColliderMassProperties::Density / ColliderBuilder::density().
Friction::coefficient / ColliderBuilder::friction().
Restitution::coefficient / ColliderBuilder::restitution().
Damping (linear and angular). You've noted "Stronger damping for stability" in main.rs – this is a prime suspect for divergence. These must be identical for a fair comparison.
CollisionGroups. Headless mode doesn't explicitly show group setup but Rapier defaults might apply. Ensure visual mode's GROUP_VEHICLE, GROUND_FILTER has an equivalent effect in headless.
GravityScale (if used on vehicle parts, must be identical).
Ccd::enabled(). If enabled in visualization for stability, it must be enabled and configured identically in the headless simulation for comparison.
Acceptance Criteria: Code for creating physics bodies/colliders is either shared or verifiably produces identical Rapier2D descriptors. All physical properties are identical.
Ticket 1.4: Standardize Joint Creation Logic

Description: Ensure RevoluteJoint (or any other joints) are defined and created identically.
Sub-Tasks:
[ ] 1.4.1: Compare RevoluteJointBuilder parameters for wheels in simulation.rs and main.rs.
[ ] 1.4.2: Specifically check local_anchor1, local_anchor2.
[ ] 1.4.3: CRITICAL (Motor): motor_model(MotorModel::ForceBased), motor_velocity(if wheel_gene.motor_torque > 0.0 { f32::MAX } else { f32::MIN }, 0.0), and motor_max_force(wheel_gene.motor_torque.abs()). These look structurally similar. Confirm the actual wheel_gene.motor_torque values being fed are scaled and interpreted identically.
Acceptance Criteria: Joint definitions are identical. Motor parameters are functionally equivalent.
Phase 2: Runtime Behavior Analysis & Debugging 🐛
Ticket 2.1: Implement Detailed Per-Step Physics State Logging

Description: Add functionality to log key physics state variables for a single, specific, identical chromosome in both modes at each physics step.
Sub-Tasks:
[ ] 2.1.1: Choose a simple, deterministic chromosome for testing (e.g., one wheel, constant positive torque).
[ ] 2.1.2 (Headless): In simulation.rs's evaluate_fitness loop, log:
Step number.
Chassis: position (x, y), rotation, linear velocity, angular velocity.
Each Wheel: position, rotation, linear velocity, angular velocity.
Contact forces/points (if feasible and necessary, Rapier's event handler or state access).
[ ] 2.1.3 (Visualized): In main.rs, within a Bevy system that runs with FixedUpdate (or however Rapier steps), log the same data for the visualized vehicle. step_vehicle_visualization_simulation is a good candidate system.
[ ] 2.1.4: Output logs to separate files (e.g., headless_physics_log.txt, visual_physics_log.txt) for easy comparison.
Acceptance Criteria: Logs are generated. Tools (even simple diff or spreadsheet) can be used to compare step-by-step data.
Ticket 2.2: Step-Through Comparison of Logs

Description: Methodically compare the generated logs from Ticket 2.1, identifying the very first step where a significant divergence occurs.
Sub-Tasks:
[ ] 2.2.1: Analyze chassis position. Does it diverge first?
[ ] 2.2.2: Analyze wheel positions/rotations.
[ ] 2.2.3: Analyze velocities.
Acceptance Criteria: The point of first significant divergence is identified.
Ticket 2.3: Verify Force/Torque Application

Description: Confirm that motor torques and any other external forces/impulses are being applied equivalently.
Sub-Tasks:
[ ] 2.3.1: Given the motor setup, the primary check is that wheel_gene.motor_torque is the same and the joint motor is configured identically (Ticket 1.4.3).
[ ] 2.3.2: If any ExternalImpulse or direct force application is used elsewhere (currently, visualization has ExternalImpulse on wheels but step_vehicle_visualization_simulation comments out direct impulse application), ensure it's mirrored or intentionally different and understood. The debug print "Torque application to wheels is now handled by joint motors, so this loop is removed." in main.rs is good, implies a shift to joint motors.
Acceptance Criteria: Motor torques are confirmed to be the only significant external force intended, and their effective application via joints is identical.
Ticket 2.4: Examine Rapier Solver Settings (Advanced)

Description: If discrepancies persist, investigate Rapier's internal solver settings.
Sub-Tasks:
[ ] 2.4.1: Check RapierConfiguration in Bevy (main.rs) for solver iteration counts (position and velocity).
[ ] 2.4.2: Ensure the headless simulation in simulation.rs uses the same number of solver iterations. This might involve passing them into PhysicsPipeline::step or configuring them on the IntegrationParameters if applicable (Rapier's API details might vary slightly between direct use and Bevy plugin abstraction).
Acceptance Criteria: Solver iteration counts are confirmed to be identical.
Phase 3: Controlled Experimentation & Validation ✅
Ticket 3.1: "Simplest Possible Vehicle" Test

Description: Test with an extremely simple vehicle (e.g., a single box chassis, no wheels, falling onto the ground) to verify basic body and collision response.
Sub-Tasks:
[ ] 3.1.1: Define this simple chromosome/setup.
[ ] 3.1.2: Run and log (per Ticket 2.1) in both modes.
[ ] 3.1.3: Compare final resting position and time to settle.
Acceptance Criteria: The simple box behaves identically.
Ticket 3.2: "Single Wheel, No Motor" Test

Description: Test a chassis with one passive (no motor torque) wheel attached by a revolute joint.
Sub-Tasks:
[ ] 3.2.1: Define chromosome.
[ ] 3.2.2: Run and log.
[ ] 3.2.3: Compare rolling behavior, how it settles.
Acceptance Criteria: Passive wheel vehicle behaves identically.
Ticket 3.3: "Single Wheel, Constant Motor" Test

Description: Test a chassis with one wheel with a constant, non-zero motor torque.
Sub-Tasks:
[ ] 3.3.1: Define chromosome.
[ ] 3.3.2: Run and log.
[ ] 3.3.3: CRITICAL: Compare distance traveled, final velocities, and wheel rotation over sim_duration_secs. This is the core test for your DistanceComparisonData.
Acceptance Criteria: Actuated wheel vehicle behaves identically, and DistanceComparisonData shows minimal/zero discrepancy.
Ticket 3.4: Introduce Complexity Incrementally

Description: If the single motor wheel test passes, gradually add complexity (more wheels, different torques, varied chassis shapes) and re-verify at each step.
Sub-Tasks:
[ ] 3.4.1: Test two wheels with same torque.
[ ] 3.4.2: Test two wheels with different torques.
[ ] 3.4.3: Test a more complex chassis shape.
Acceptance Criteria: Consistency is maintained as complexity is added.
Phase 4: Documentation & Future-Proofing 📝
Ticket 4.1: Document All Physics-Sensitive Parameters

Description: Create a markdown document or extensive code comments detailing every parameter that affects physics and why it's set to a particular value, specifically noting its role in maintaining consistency.
Acceptance Criteria: Clear documentation exists.
Ticket 4.2: Implement an Automated Consistency Check (Stretch Goal)

Description: Create a test that programmatically runs a reference chromosome in both modes and asserts that key outcomes (e.g., final X position) are within a tight tolerance.
Acceptance Criteria: An automated test helps prevent future regressions.