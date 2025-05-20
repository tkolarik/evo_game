use rand::Rng;

// --- Gene Range Constants ---

// Chassis
pub const MIN_CHASSIS_WIDTH: f32 = 0.5;
pub const MAX_CHASSIS_WIDTH: f32 = 4.0;
pub const MIN_CHASSIS_HEIGHT: f32 = 0.2;
pub const MAX_CHASSIS_HEIGHT: f32 = 3.0;
pub const MIN_CHASSIS_DENSITY: f32 = 0.5;
pub const MAX_CHASSIS_DENSITY: f32 = 3.0;

// Wheel
pub const MIN_WHEEL_RADIUS: f32 = 0.3;
pub const MAX_WHEEL_RADIUS: f32 = 1.5;
pub const MIN_WHEEL_DENSITY: f32 = 0.3;
pub const MAX_WHEEL_DENSITY: f32 = 2.0;
pub const MIN_WHEEL_MOTOR_TORQUE: f32 = -0.000001;
pub const MAX_WHEEL_MOTOR_TORQUE: f32 = 0.000001;
pub const MIN_WHEEL_FRICTION_COEFFICIENT: f32 = 0.2;
pub const MAX_WHEEL_FRICTION_COEFFICIENT: f32 = 2.0;

pub const NUM_POTENTIAL_WHEELS: usize = 4;

// --- Gene Structs ---

#[derive(Debug, Clone, PartialEq)]
pub struct ChassisGenes {
    pub width: f32,
    pub height: f32,
    pub density: f32,
}

impl ChassisGenes {
    pub fn random<R: Rng + ?Sized>(rng: &mut R) -> Self {
        Self {
            width: rng.gen_range(MIN_CHASSIS_WIDTH..=MAX_CHASSIS_WIDTH),
            height: rng.gen_range(MIN_CHASSIS_HEIGHT..=MAX_CHASSIS_HEIGHT),
            density: rng.gen_range(MIN_CHASSIS_DENSITY..=MAX_CHASSIS_DENSITY),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct WheelGenes {
    pub active: bool, // true for 1, false for 0
    pub x_position_factor: f32, // Relative to chassis width/2, range: -1.0 to 1.0 (approx)
    pub y_position_factor: f32, // Relative to chassis height/2, range: -1.0 to 1.0 (approx)
    pub radius: f32,
    pub density: f32,
    pub motor_torque: f32,
    pub friction_coefficient: f32,
}

impl WheelGenes {
    pub fn random<R: Rng + ?Sized>(rng: &mut R) -> Self {
        Self {
            active: rng.gen_bool(0.5), // 50% chance of being active initially
            x_position_factor: rng.gen_range(-1.0/1.8..=1.0/1.8), // As per spec: -chassis_width/1.8 to chassis_width/1.8
            y_position_factor: rng.gen_range(-1.0/1.8..=1.0/1.8), // As per spec: -chassis_height/1.8 to chassis_height/1.8
            radius: rng.gen_range(MIN_WHEEL_RADIUS..=MAX_WHEEL_RADIUS),
            density: rng.gen_range(MIN_WHEEL_DENSITY..=MAX_WHEEL_DENSITY),
            motor_torque: rng.gen_range(MIN_WHEEL_MOTOR_TORQUE..=MAX_WHEEL_MOTOR_TORQUE),
            friction_coefficient: rng.gen_range(MIN_WHEEL_FRICTION_COEFFICIENT..=MAX_WHEEL_FRICTION_COEFFICIENT),
        }
    }

    // Actual x_position relative to chassis center
    pub fn get_x_position(&self, chassis_width: f32) -> f32 {
        self.x_position_factor * (chassis_width / 2.0)
    }

    // Actual y_position relative to chassis center
    pub fn get_y_position(&self, chassis_height: f32) -> f32 {
        self.y_position_factor * (chassis_height / 2.0)
    }
}


// --- Chromosome Struct ---

#[derive(Debug, Clone, PartialEq)]
pub struct Chromosome {
    pub chassis: ChassisGenes,
    pub wheels: Vec<WheelGenes>,
    // Note: The problem statement implies a fixed-length chromosome if represented as a flat list.
    // Here, we use structs for clarity, but the total number of gene values is fixed.
}

impl Chromosome {
    pub fn new_random<R: Rng + ?Sized>(rng: &mut R) -> Self {
        let chassis = ChassisGenes::random(rng);
        let wheels = (0..NUM_POTENTIAL_WHEELS)
            .map(|_| WheelGenes::random(rng))
            .collect();
        Self { chassis, wheels }
    }

    // Convert chromosome to a flat list of f32 for crossover operations.
    // Boolean 'active' gene is represented as 0.0 or 1.0.
    pub fn to_flat_list(&self) -> Vec<f32> {
        let mut genes = Vec::new();
        genes.push(self.chassis.width);
        genes.push(self.chassis.height);
        genes.push(self.chassis.density);

        for wheel in &self.wheels {
            genes.push(if wheel.active { 1.0 } else { 0.0 });
            genes.push(wheel.x_position_factor);
            genes.push(wheel.y_position_factor);
            genes.push(wheel.radius);
            genes.push(wheel.density);
            genes.push(wheel.motor_torque);
            genes.push(wheel.friction_coefficient);
        }
        genes
    }

    // Create a chromosome from a flat list of f32.
    // Assumes the list is in the correct order and length.
    pub fn from_flat_list(genes: &[f32]) -> Self {
        let mut current_idx = 0;
        let chassis = ChassisGenes {
            width: genes[current_idx], 
            height: genes[current_idx + 1],
            density: genes[current_idx + 2],
        };
        current_idx += 3;

        let mut wheels = Vec::with_capacity(NUM_POTENTIAL_WHEELS);
        for _ in 0..NUM_POTENTIAL_WHEELS {
            if current_idx + 7 > genes.len() { // Check bounds
                // This case should ideally not happen if gene list is correct
                // Fill with default/random if undersized, or handle error
                wheels.push(WheelGenes::random(&mut rand::thread_rng())); // Fallback, consider error handling
                break; 
            }
            wheels.push(WheelGenes {
                active: genes[current_idx] >= 0.5, // Convert float back to bool
                x_position_factor: genes[current_idx + 1],
                y_position_factor: genes[current_idx + 2],
                radius: genes[current_idx + 3],
                density: genes[current_idx + 4],
                motor_torque: genes[current_idx + 5],
                friction_coefficient: genes[current_idx + 6],
            });
            current_idx += 7;
        }
        // Ensure wheels vec has NUM_POTENTIAL_WHEELS elements, even if gene list was short
        while wheels.len() < NUM_POTENTIAL_WHEELS {
            wheels.push(WheelGenes::random(&mut rand::thread_rng())); // Or some default
        }

        Self { chassis, wheels }
    }

    // TODO: Implement genotype to phenotype mapping function later
    // This function will take a Chromosome and create a physical representation in the physics engine.
}

// Example usage (will be moved to main or other modules later)
#[cfg(test)]
mod tests {
    use super::*;
    use rand::thread_rng;

    #[test]
    fn create_random_chromosome() {
        let mut rng = thread_rng();
        let chromosome = Chromosome::new_random(&mut rng);

        assert_eq!(chromosome.wheels.len(), NUM_POTENTIAL_WHEELS);
        println!("Random chromosome: {:#?}", chromosome);

        // Test wheel position calculation
        let chassis_width = chromosome.chassis.width;
        let chassis_height = chromosome.chassis.height;

        for wheel in &chromosome.wheels {
            let x_pos = wheel.get_x_position(chassis_width);
            let y_pos = wheel.get_y_position(chassis_height);

            let max_x_offset = chassis_width / 2.0 / 1.8;
            let max_y_offset = chassis_height / 2.0 / 1.8;

            assert!(x_pos.abs() <= max_x_offset + 1e-6, "x_pos: {}, max_x_offset: {}", x_pos, max_x_offset );
            assert!(y_pos.abs() <= max_y_offset + 1e-6, "y_pos: {}, max_y_offset: {}", y_pos, max_y_offset );
        }
    }

    #[test]
    fn test_chromosome_flat_list_conversion() {
        let mut rng = thread_rng();
        let original_chromosome = Chromosome::new_random(&mut rng);
        
        let flat_genes = original_chromosome.to_flat_list();
        // Expected number of genes: 3 for chassis + 7 per wheel * NUM_POTENTIAL_WHEELS
        assert_eq!(flat_genes.len(), 3 + 7 * NUM_POTENTIAL_WHEELS);

        let reconstructed_chromosome = Chromosome::from_flat_list(&flat_genes);

        // Compare float values with a tolerance
        assert!((original_chromosome.chassis.width - reconstructed_chromosome.chassis.width).abs() < 1e-6);
        assert!((original_chromosome.chassis.height - reconstructed_chromosome.chassis.height).abs() < 1e-6);
        assert!((original_chromosome.chassis.density - reconstructed_chromosome.chassis.density).abs() < 1e-6);

        for i in 0..NUM_POTENTIAL_WHEELS {
            assert_eq!(original_chromosome.wheels[i].active, reconstructed_chromosome.wheels[i].active);
            assert!((original_chromosome.wheels[i].x_position_factor - reconstructed_chromosome.wheels[i].x_position_factor).abs() < 1e-6);
            assert!((original_chromosome.wheels[i].y_position_factor - reconstructed_chromosome.wheels[i].y_position_factor).abs() < 1e-6);
            assert!((original_chromosome.wheels[i].radius - reconstructed_chromosome.wheels[i].radius).abs() < 1e-6);
            assert!((original_chromosome.wheels[i].density - reconstructed_chromosome.wheels[i].density).abs() < 1e-6);
            assert!((original_chromosome.wheels[i].motor_torque - reconstructed_chromosome.wheels[i].motor_torque).abs() < 1e-6);
            assert!((original_chromosome.wheels[i].friction_coefficient - reconstructed_chromosome.wheels[i].friction_coefficient).abs() < 1e-6);
        }
        // For a more robust comparison, especially if there are many f32s, implementing PartialEq with tolerance is better.
        // However, direct field comparison is fine here given the context.
        // Using `assert_eq!(original_chromosome, reconstructed_chromosome);` would require careful f32 comparison in PartialEq.
    }
} 