use billiard_ga::model::{TableState, ShotInput};
use billiard_ga::physics::simulate_shot;

fn main() {
    // 物理エンジンの結合テスト用
    let state = TableState {
        cue_ball_pos: (400.0, 300.0),
        remaining_balls: vec![(1, (500.0, 300.0))],
    };

    let input = ShotInput {
        target_ball_id: 1,
        target_pocket_id: 2,
    };

    let result = simulate_shot(&state, &input);
    println!("Simulation Result: {:?}", result);
}