use crate::organism::{Chromosome, MIN_CHASSIS_WIDTH, MAX_CHASSIS_WIDTH, MIN_CHASSIS_HEIGHT, MAX_CHASSIS_HEIGHT, MIN_CHASSIS_DENSITY, MAX_CHASSIS_DENSITY, MIN_WHEEL_RADIUS, MAX_WHEEL_RADIUS, MIN_WHEEL_DENSITY, MAX_WHEEL_DENSITY, MIN_WHEEL_MOTOR_TORQUE, MAX_WHEEL_MOTOR_TORQUE, MIN_WHEEL_FRICTION_COEFFICIENT, MAX_WHEEL_FRICTION_COEFFICIENT, get_test_chromosome};
use crate::simulation::{PhysicsWorld, SimulationConfig, CrossoverType, BatchRunConfig};
use rand::prelude::*;
use rand_distr::{Normal, Distribution};
use std::sync::atomic::{AtomicUsize, Ordering};

// Global counter for unique individual IDs
static NEXT_INDIVIDUAL_ID: AtomicUsize = AtomicUsize::new(0);

fn generate_unique_id() -> usize {
    NEXT_INDIVIDUAL_ID.fetch_add(1, Ordering::SeqCst)
}

#[derive(Debug, Clone)]
pub struct Individual {
    pub id: usize,
    pub chromosome: Chromosome,
    pub fitness: f32,
    // pub parents: Option<(usize, usize)>,
    pub generation: usize, // Track which generation this individual belongs to
}

impl Individual {
    pub fn new_random<R: Rng + ?Sized>(rng: &mut R, generation: usize) -> Self {
        Self {
            id: generate_unique_id(),
            chromosome: Chromosome::new_random(rng),
            fitness: 0.0,
            // parents: None, 
            generation,
        }
    }

    // Applies mutation to a chromosome based on the simulation config
    fn mutate_chromosome<R: Rng + ?Sized>(chromosome: &mut Chromosome, rng: &mut R, config: &SimulationConfig) {
        if rng.gen_bool(config.mutation_rate_per_individual) {
            // Chassis genes
            if rng.gen_bool(config.mutation_rate_per_gene) { 
                Self::mutate_float_gene(&mut chromosome.chassis.width, MIN_CHASSIS_WIDTH, MAX_CHASSIS_WIDTH, rng);
            }
            if rng.gen_bool(config.mutation_rate_per_gene) { 
                Self::mutate_float_gene(&mut chromosome.chassis.height, MIN_CHASSIS_HEIGHT, MAX_CHASSIS_HEIGHT, rng);
            }
            if rng.gen_bool(config.mutation_rate_per_gene) { 
                Self::mutate_float_gene(&mut chromosome.chassis.density, MIN_CHASSIS_DENSITY, MAX_CHASSIS_DENSITY, rng);
            }

            // Wheel genes
            for wheel in &mut chromosome.wheels {
                // wheel_active (discrete)
                if rng.gen_bool(config.mutation_rate_per_gene) {
                    wheel.active = !wheel.active;
                }
                // wheel_x_position_factor
                if rng.gen_bool(config.mutation_rate_per_gene) {
                    Self::mutate_float_gene(&mut wheel.x_position_factor, -1.0/1.8, 1.0/1.8, rng);
                }
                // wheel_y_position_factor
                if rng.gen_bool(config.mutation_rate_per_gene) {
                    Self::mutate_float_gene(&mut wheel.y_position_factor, -1.0/1.8, 1.0/1.8, rng);
                }
                // wheel_radius
                if rng.gen_bool(config.mutation_rate_per_gene) {
                    Self::mutate_float_gene(&mut wheel.radius, MIN_WHEEL_RADIUS, MAX_WHEEL_RADIUS, rng);
                }
                // wheel_density
                if rng.gen_bool(config.mutation_rate_per_gene) {
                    Self::mutate_float_gene(&mut wheel.density, MIN_WHEEL_DENSITY, MAX_WHEEL_DENSITY, rng);
                }
                // wheel_motor_torque
                if rng.gen_bool(config.mutation_rate_per_gene) {
                    Self::mutate_float_gene(&mut wheel.motor_torque, MIN_WHEEL_MOTOR_TORQUE, MAX_WHEEL_MOTOR_TORQUE, rng);
                }
                // wheel_friction_coefficient
                if rng.gen_bool(config.mutation_rate_per_gene) {
                    Self::mutate_float_gene(&mut wheel.friction_coefficient, MIN_WHEEL_FRICTION_COEFFICIENT, MAX_WHEEL_FRICTION_COEFFICIENT, rng);
                }
            }
        }
    }

    fn mutate_float_gene<R: Rng + ?Sized>(value: &mut f32, min: f32, max: f32, rng: &mut R) {
        let range = max - min;
        // Standard deviation is a fraction of the range (e.g., 10%)
        let std_dev = range * 0.1;
        let normal = Normal::new(0.0, std_dev).unwrap();
        let mutation_value = normal.sample(rng);
        *value += mutation_value;
        *value = value.clamp(min, max); // Ensure value stays within bounds
    }

    pub fn from_parents<R: Rng + ?Sized>(
        parent1: &Individual,
        parent2: &Individual,
        rng: &mut R,
        config: &SimulationConfig,
        generation: usize,
    ) -> Self {
        let mut offspring_chromosome = match config.crossover_type {
            CrossoverType::SinglePoint => {
                // Represent chromosome as a flat list of f32 for crossover
                let mut parent1_genes = parent1.chromosome.to_flat_list();
                let parent2_genes = parent2.chromosome.to_flat_list();
                let gene_count = parent1_genes.len();
                if gene_count == 0 { parent1.chromosome.clone() } else {
                    let crossover_point = rng.gen_range(0..gene_count);
                    for i in crossover_point..gene_count {
                        parent1_genes[i] = parent2_genes[i];
                    }
                    Chromosome::from_flat_list(&parent1_genes)
                }
            }
            CrossoverType::Uniform => {
                let mut parent1_genes = parent1.chromosome.to_flat_list();
                let parent2_genes = parent2.chromosome.to_flat_list();
                let gene_count = parent1_genes.len();
                 if gene_count == 0 { parent1.chromosome.clone() } else {
                    for i in 0..gene_count {
                        if rng.gen_bool(0.5) { // 50% chance to take from parent2
                            parent1_genes[i] = parent2_genes[i];
                        }
                    }
                    Chromosome::from_flat_list(&parent1_genes)
                }
            }
        };

        Self::mutate_chromosome(&mut offspring_chromosome, rng, config);

        Self {
            id: generate_unique_id(),
            chromosome: offspring_chromosome,
            fitness: 0.0,
            // parents: Some((parent1.id, parent2.id)),
            generation,
        }
    }
}

#[derive(Debug)]
pub struct Population {
    pub individuals: Vec<Individual>,
    pub generation_count: usize,
}

impl Population {
    pub fn new_random<R: Rng + ?Sized>(size: usize, rng: &mut R, generation: usize) -> Self {
        let individuals = (0..size).map(|_| Individual::new_random(rng, generation)).collect();
        Self {
            individuals,
            generation_count: generation,
        }
    }

    // Tournament Selection
    fn select_one_parent<'a, R: Rng + ?Sized>(
        &'a self,
        rng: &mut R,
        tournament_size: usize,
    ) -> &'a Individual {
        let mut best_in_tournament: Option<&Individual> = None;
        for _ in 0..tournament_size {
            let contender_idx = rng.gen_range(0..self.individuals.len());
            let contender = &self.individuals[contender_idx];
            if best_in_tournament.is_none() || contender.fitness > best_in_tournament.unwrap().fitness {
                best_in_tournament = Some(contender);
            }
        }
        best_in_tournament.expect("Tournament selection failed to pick an individual")
    }

    pub fn select_parents<'a, R: Rng + ?Sized>(
        &'a self,
        rng: &mut R,
        tournament_size: usize,
    ) -> (&'a Individual, &'a Individual) {
        let parent1 = self.select_one_parent(rng, tournament_size);
        let mut parent2 = self.select_one_parent(rng, tournament_size);
        // Ensure parents are different if population size allows
        if self.individuals.len() > 1 {
            while parent2.id == parent1.id {
                parent2 = self.select_one_parent(rng, tournament_size);
            }
        }
        (parent1, parent2)
    }

    pub fn evaluate_fitness(&mut self, physics_world: &mut PhysicsWorld, sim_config: &SimulationConfig, batch_run_config: &BatchRunConfig) {
        let force_intensive_logging = batch_run_config.log_this_batch_intensively;
        for individual in &mut self.individuals {
            // Ensure fitness is re-evaluated if it's an elite from a previous gen or a new offspring
            individual.fitness = physics_world.evaluate_fitness(&individual.chromosome, sim_config, force_intensive_logging);
        }
        // Sort by fitness (descending) for elitism and stats
        self.individuals.sort_by(|a, b| b.fitness.partial_cmp(&a.fitness).unwrap_or(std::cmp::Ordering::Equal));
    }
}

pub struct EvolutionEngine {
    pub population: Population,
    config: SimulationConfig,
    rng: StdRng,
    // For phylogeny tracking (child_id, parent1_id, parent2_id, generation_number)
    pub phylogeny_data: Vec<(usize, Option<usize>, Option<usize>, usize)>,
    pub all_time_best_individual: Option<Individual>, // Added for Hall of Fame
}

impl EvolutionEngine {
    pub fn new(config: SimulationConfig) -> Self {
        let mut rng = StdRng::from_entropy();
        let initial_generation = 0;
        let population = Population::new_random(config.population_size, &mut rng, initial_generation);
        let mut phylogeny_data = Vec::new();
        for individual in &population.individuals {
            phylogeny_data.push((individual.id, None, None, initial_generation));
        }

        Self {
            population,
            config,
            rng,
            phylogeny_data,
            all_time_best_individual: None, // Initialize Hall of Fame
        }
    }

    // Method to reset the engine to its initial state
    pub fn reset(&mut self) {
        self.rng = StdRng::from_entropy();
        let initial_generation = 0;
        self.population = Population::new_random(self.config.population_size, &mut self.rng, initial_generation);
        
        // Overwrite the first individual with the test chromosome for logging purposes
        if !self.population.individuals.is_empty() && self.config.population_size > 0 {
            let original_id = self.population.individuals[0].id;
            self.population.individuals[0] = Individual {
                id: original_id, // Keep original ID to avoid issues if ID matters elsewhere
                chromosome: get_test_chromosome(),
                fitness: 0.0, // Will be evaluated
                generation: initial_generation,
            };
            println!("ENGINE RESET: First individual replaced with TEST chromosome.");
        } else {
            println!("ENGINE RESET: Population is empty or size 0, cannot insert test chromosome.");
        }

        self.phylogeny_data.clear();
        for individual in &self.population.individuals {
            self.phylogeny_data.push((individual.id, None, None, initial_generation));
        }
        self.all_time_best_individual = None;
        println!("EvolutionEngine has been reset.");
    }

    fn update_all_time_best(&mut self) {
        if let Some(current_gen_best) = self.population.individuals.first() {
            if self.all_time_best_individual.is_none() || current_gen_best.fitness > self.all_time_best_individual.as_ref().unwrap().fitness {
                self.all_time_best_individual = Some(current_gen_best.clone());
                println!(
                    "New all-time best individual! ID: {}, Fitness: {:.2}, Gen: {}",
                    current_gen_best.id,
                    current_gen_best.fitness,
                    current_gen_best.generation
                );
            }
        }
    }

    pub fn evolve_generation(&mut self, physics_world: &mut PhysicsWorld, batch_run_config: &BatchRunConfig) {
        let next_generation_number = self.population.generation_count + 1;
        let mut new_population = Vec::with_capacity(self.config.population_size);

        // Elitism: Carry over the best individuals from the current population
        for i in 0..self.config.elitism_count {
            if i < self.population.individuals.len() {
                let elite = self.population.individuals[i].clone();
                // Elite individuals fitness is already known, no need to re-evaluate unless forced
                new_population.push(elite);
            }
        }

        // Fill the rest of the new population with offspring
        while new_population.len() < self.config.population_size {
            let (parent1, parent2) = self.population.select_parents(&mut self.rng, self.config.tournament_size);
            let mut offspring = Individual::from_parents(parent1, parent2, &mut self.rng, &self.config, next_generation_number);
            
            // Record phylogeny for the offspring
            self.phylogeny_data.push((offspring.id, Some(parent1.id), Some(parent2.id), next_generation_number));
            new_population.push(offspring);
        }

        self.population.individuals = new_population;
        self.population.generation_count = next_generation_number;

        // Evaluate fitness for the new generation
        self.population.evaluate_fitness(physics_world, &self.config, batch_run_config);
        
        // Update all-time best after evaluation
        self.update_all_time_best();

        // Optional: Print some stats
        if self.population.generation_count % 10 == 0 || self.config.population_size <= 10 { // Print more often for small pops
            println!(
                "Gen: {} - Best Fitness: {:.2}, Avg Fitness: {:.2}",
                self.population.generation_count,
                self.population.individuals.first().map_or(0.0, |ind| ind.fitness),
                if !self.population.individuals.is_empty() {
                    self.population.individuals.iter().map(|ind| ind.fitness).sum::<f32>() / self.population.individuals.len() as f32
                } else { 0.0 }
            );
        }
    }

    // pub fn run_simulation(&mut self, physics_world: &mut PhysicsWorld) {
    //     self.population.evaluate_fitness(physics_world, &self.config);
    //     self.update_all_time_best(); // Check after initial population evaluation

    //     for gen_idx in 0..self.config.num_generations {
    //          if gen_idx > 0 { // For gen 0, fitness is already evaluated
    //             self.evolve_generation(physics_world);
    //          }
    //         if self.population.generation_count >= self.config.num_generations {
    //             // This condition might be met if num_generations is 0 or 1.
    //             // For num_generations = 1, evolve_generation is not called in loop, so we print final stats after loop.
    //             break; 
    //         }
    //     }
    //     // Ensure final stats are printed if simulation ended by reaching num_generations.
    //     // Also ensure the last generation (if not gen 0) gets its fitness evaluated if loop structure changes.
    //     // Current structure: gen 0 evaluated before loop, subsequent gens in evolve_generation.
    //     // If num_generations = 0, loop is skipped, initial stats printed.
    //     // If num_generations = 1, loop is skipped, initial stats printed. This needs a slight adjustment for clarity.
        
    //     // If the simulation ran for at least one generation (beyond initial setup)
    //     // and the final generation's fitness hasn't been printed by the loop's own evolve_generation call.
    //     // The loop structure ensures that if evolve_generation was called, its print occurs.
    //     // This is mainly for the case where num_generations = 0 or 1.

    //     println!("Simulation finished after {} target generations (Actual: {}).", self.config.num_generations, self.population.generation_count);
    //     println!(
    //         "Final Stats - Generation: {}, Best Fitness: {:.2}, Avg Fitness: {:.2}",
    //         self.population.generation_count,
    //         self.population.individuals.first().map_or(0.0, |ind| ind.fitness),
    //         if self.population.individuals.is_empty() { 0.0 } 
    //         else { self.population.individuals.iter().map(|ind| ind.fitness).sum::<f32>() / self.population.individuals.len() as f32 }
    //     );
    //     if let Some(best_overall) = &self.all_time_best_individual {
    //          println!("All-time Best Individual - ID: {}, Fitness: {:.2}, Gen: {}", best_overall.id, best_overall.fitness, best_overall.generation);
    //     }
    // }
}

#[cfg(test)]
mod tests {
    use super::*;    

    // Helper to get default config for tests
    fn test_config() -> SimulationConfig {
        SimulationConfig {
            population_size: 10,
            num_generations: 3, 
            sim_duration_secs: 0.5, // Shorter sim for tests
            tournament_size: 3,
            elitism_count: 1,
            mutation_rate_per_gene: 0.1,
            mutation_rate_per_individual: 1.0, // Ensure mutation happens for testing
            ..Default::default()
        }
    }

    #[test]
    fn test_population_initialization_and_evaluation() {
        let config = test_config();
        let mut rng = StdRng::from_entropy();
        let mut population = Population::new_random(config.population_size, &mut rng, 0);
        assert_eq!(population.individuals.len(), config.population_size);
        
        let mut physics_world = PhysicsWorld::new(&config);
        population.evaluate_fitness(&mut physics_world, &config, &BatchRunConfig { 
            log_this_batch_intensively: false,
            target_generations_this_batch: None,
            generations_completed_this_batch: 0,
        });
        assert!(population.individuals.first().unwrap().fitness >= 0.0);
    }

    #[test]
    fn test_tournament_selection() {
        let config = test_config();
        let mut rng = StdRng::from_entropy();
        let mut population = Population::new_random(config.population_size, &mut rng, 0);
        // Assign some fitness values for testing selection
        for (i, ind) in population.individuals.iter_mut().enumerate() {
            ind.fitness = i as f32;
        }
        population.individuals.sort_by(|a, b| b.fitness.partial_cmp(&a.fitness).unwrap_or(std::cmp::Ordering::Equal)); // Sort descending

        let (p1, p2) = population.select_parents(&mut rng, config.tournament_size);
        assert!(p1.fitness >= 0.0);
        assert!(p2.fitness >= 0.0);
        if config.population_size > 1 {
            assert_ne!(p1.id, p2.id, "Parents should ideally be different");
        }
    }

    #[test]
    fn test_crossover_and_mutation() {
        let config = test_config();
        let mut rng = StdRng::from_entropy();
        let parent1 = Individual::new_random(&mut rng, 0);
        let parent2 = Individual::new_random(&mut rng, 0);

        let offspring_single_point = Individual::from_parents(&parent1, &parent2, &mut rng, &SimulationConfig {crossover_type: CrossoverType::SinglePoint, ..config.clone()}, 1);
        let offspring_uniform = Individual::from_parents(&parent1, &parent2, &mut rng, &SimulationConfig {crossover_type: CrossoverType::Uniform, ..config.clone()}, 1);

        assert_ne!(parent1.chromosome.chassis.width, offspring_single_point.chromosome.chassis.width, "Offspring should differ from parent after crossover/mutation (highly probable)");
        assert_ne!(parent1.chromosome.chassis.width, offspring_uniform.chromosome.chassis.width, "Offspring should differ from parent after crossover/mutation (highly probable)");
    }

    #[test]
    fn test_all_time_best_tracking() {
        let config = test_config();
        let mut engine = EvolutionEngine::new(config.clone());
        let mut physics_world = PhysicsWorld::new(&config);

        // Manually assign fitness to ensure tracking works
        engine.population.individuals[0].fitness = 10.0;
        for i in 1..engine.population.individuals.len() {
            engine.population.individuals[i].fitness = i as f32;
        }
        engine.population.individuals.sort_by(|a, b| b.fitness.partial_cmp(&a.fitness).unwrap_or(std::cmp::Ordering::Equal));
        engine.update_all_time_best();
        assert_eq!(engine.all_time_best_individual.as_ref().unwrap().fitness, 10.0);

        // Simulate a generation where a new best might appear or not
        engine.evolve_generation(&mut physics_world, &BatchRunConfig { 
            log_this_batch_intensively: false,
            target_generations_this_batch: None, 
            generations_completed_this_batch: 0,
        }); // This will re-evaluate with random physics
        // The test for all_time_best should be more robust if we could mock fitness after evolve_generation
        println!("All-time best after 1st evolve: {:?}", engine.all_time_best_individual.as_ref().map(|i| i.fitness));
        assert!(engine.all_time_best_individual.is_some());
    }

    #[test]
    fn test_engine_reset() {
        let config = test_config();
        let mut engine = EvolutionEngine::new(config.clone());
        let mut physics_world = PhysicsWorld::new(&config);
        engine.evolve_generation(&mut physics_world, &BatchRunConfig { 
            log_this_batch_intensively: false,
            target_generations_this_batch: None,
            generations_completed_this_batch: 0,
        });
        assert_ne!(engine.population.generation_count, 0);
        assert!(!engine.phylogeny_data.is_empty());
        engine.population.individuals[0].fitness = 100.0; // Set an all time best
        engine.update_all_time_best();
        assert!(engine.all_time_best_individual.is_some());

        engine.reset();
        assert_eq!(engine.population.generation_count, 0);
        // Phylogeny for gen 0 will have initial population entries
        assert_eq!(engine.phylogeny_data.len(), config.population_size);
        assert!(engine.all_time_best_individual.is_none());
        assert_eq!(engine.population.individuals.len(), config.population_size);
    }

    #[test]
    fn test_evolution_engine_run_with_real_physics() {
        let config = test_config();
        let mut engine = EvolutionEngine::new(config.clone());
        let mut physics_world = PhysicsWorld::new(&config);

        // engine.run_simulation(&mut physics_world);
        assert_eq!(engine.population.generation_count, config.num_generations);
        assert_eq!(engine.population.individuals.len(), config.population_size);
        if let Some(best_ind) = engine.population.individuals.first() {
            println!("Test Run - Best Fitness: {:.2}, Gen: {}", best_ind.fitness, best_ind.generation);
            assert!(best_ind.fitness >= 0.0);
        }
        assert!(!engine.phylogeny_data.is_empty());
        if let Some(best_overall) = &engine.all_time_best_individual {
            println!("Test Run - All Time Best Fitness: {:.2}, Gen: {}", best_overall.fitness, best_overall.generation);
            assert!(best_overall.fitness >= 0.0);
        }
    }
} 