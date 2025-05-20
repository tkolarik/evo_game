use crate::evolution::EvolutionEngine; // To access phylogeny_data
use crate::simulation::SimulationConfig; // Potentially for labels
use std::fs::File;
use std::io::Write;

// pub type PhylogenyNode = usize; // Individual ID
// pub type PhylogenyEdge = (usize, usize); // (ParentID, ChildID)

// Generates a DOT language string for the phylogeny tree
pub fn generate_dot_string(engine: &EvolutionEngine, _config: &SimulationConfig) -> String {
    let mut nodes = std::collections::HashSet::new();
    let mut edges = Vec::new();

    for (child_id, parent1_id_opt, parent2_id_opt, _generation) in &engine.phylogeny_data {
        nodes.insert(*child_id);
        if let Some(p1_id) = parent1_id_opt {
            // Don't draw edges from elite to itself if it's just carrying over
            if *p1_id != *child_id || parent2_id_opt.is_some() { 
                nodes.insert(*p1_id);
                edges.push((*p1_id, *child_id));
            }
        }
        if let Some(p2_id) = parent2_id_opt {
            nodes.insert(*p2_id);
            edges.push((*p2_id, *child_id));
        }
    }

    let mut dot_graph = Vec::new();
    dot_graph.push(format!("digraph Phylogeny {{"));
    dot_graph.push(format!("  rankdir=LR;")); // Left to Right ranking for generations
    dot_graph.push(format!("  node [shape=circle, style=filled, fillcolor=lightblue];"));

    for node_id in nodes {
        // Could add more info to node labels, e.g., fitness if stored with ID
        dot_graph.push(format!("  {}_[label=\"{}\"];", node_id, node_id));
    }

    for (p_id, c_id) in edges {
        dot_graph.push(format!("  {}_ -> {}_;", p_id, c_id));
    }

    dot_graph.push(format!("}}"));
    dot_graph.join("\n")
}

// System (or function called by a system) to write the DOT file
pub fn write_phylogeny_to_dot_file(engine: &EvolutionEngine, config: &SimulationConfig, path: &str) -> std::io::Result<()> {
    let dot_string = generate_dot_string(engine, config);
    let mut file = File::create(path)?;
    file.write_all(dot_string.as_bytes())?;
    println!("Phylogeny DOT file written to {}", path);
    Ok(())
} 