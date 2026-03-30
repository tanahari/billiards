use rand::prelude::*;
use rand::seq::SliceRandom;

pub mod model;
pub mod math;
pub mod physics;
mod evaluation;

const POPULATION_SIZE: usize = 50;
const MUTATION_RATE: f64 = 0.1;
const GENERATIONS: usize = 100;

#[derive(Clone, Debug)]
struct Individual {
    target_order: Vec<u8>,
    pocket_selection: Vec<u8>,
    fitness: f64,
}

impl Individual {
    fn new_random(rng: &mut impl Rng) -> Self {
        let mut order: Vec<u8> = (1..=evaluation::NUM_BALLS).collect();
        order.shuffle(rng);

        let pockets: Vec<u8> = (0..evaluation::NUM_BALLS)
            .map(|_| rng.gen_range(0..evaluation::NUM_POCKETS)) // ★変更
            .collect();

        let fitness = evaluation::evaluate_individual(&order, &pockets);

        Individual {
            target_order: order,
            pocket_selection: pockets,
            fitness,
        }
    }
}

fn main() {
    let mut rng = rand::thread_rng(); // ★変更

    let mut population: Vec<Individual> = (0..POPULATION_SIZE)
        .map(|_| Individual::new_random(&mut rng))
        .collect();

    println!("ボーラードGA（角度最小化）を開始します...");

    for generation in 1..=GENERATIONS {
        population.sort_by(|a, b| a.fitness.partial_cmp(&b.fitness).unwrap());

        let best = &population[0];
        println!(
            "世代 {:3}: 最小スコア = {:.4} (順序: {:?}, ポケット: {:?})",
            generation, best.fitness, best.target_order, best.pocket_selection
        );

        if best.fitness <= 0.0 {
            println!("最適解を発見しました！");
            break;
        }

        let mut next_generation = Vec::new();

        next_generation.push(population[0].clone());
        next_generation.push(population[1].clone());

        while next_generation.len() < POPULATION_SIZE {
            let parent1 = &population[rng.gen_range(0..10)]; // ★変更
            let parent2 = &population[rng.gen_range(0..10)]; // ★変更

            let mut child_order = parent1.target_order.clone();

            let cross_point = rng.gen_range(0..evaluation::NUM_BALLS as usize); // ★変更
            let mut child_pockets = parent1.pocket_selection.clone();
            child_pockets[cross_point..].copy_from_slice(&parent2.pocket_selection[cross_point..]);

            if rng.gen_bool(MUTATION_RATE) { // ★変更
                let idx1 = rng.gen_range(0..evaluation::NUM_BALLS as usize); // ★変更
                let idx2 = rng.gen_range(0..evaluation::NUM_BALLS as usize); // ★変更
                child_order.swap(idx1, idx2);
            }

            for pocket in child_pockets.iter_mut() {
                if rng.gen_bool(MUTATION_RATE) { // ★変更
                    *pocket = rng.gen_range(0..evaluation::NUM_POCKETS); // ★変更
                }
            }

            let fitness = evaluation::evaluate_individual(&child_order, &child_pockets);

            next_generation.push(Individual {
                target_order: child_order,
                pocket_selection: child_pockets,
                fitness,
            });
        }
        population = next_generation;
    }
}