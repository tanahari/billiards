// src/main.rs
use billiard_ga::model::{TableState, ShotInput};
use billiard_ga::physics::simulate_sequence;

fn main() {
    println!("--- ビリヤードGA 物理エンジン テスト開始 ---");

    // 1. 初期盤面のセットアップ（テスト用に的球を3つ配置）
    let state = TableState {
        cue_ball_pos: (400.0, 300.0), // 手球の初期位置
        remaining_balls: vec![
            (1, (500.0, 300.0)), // 的球ID: 1
            (2, (600.0, 200.0)), // 的球ID: 2
            (3, (300.0, 400.0)), // 的球ID: 3
        ],
    };

    // 2. GAからの指令（3手分の配列を用意する）
    let inputs = [
        ShotInput { target_ball_id: 1, target_pocket_id: 2 }, // 1手目
        ShotInput { target_ball_id: 2, target_pocket_id: 5 }, // 2手目
        ShotInput { target_ball_id: 3, target_pocket_id: 0 }, // 3手目
    ];

    // 3. シミュレーション実行（配列の参照を渡す）
    let result = simulate_sequence(&state, &inputs);

    // 4. 結果の出力
    println!("Simulation Result: {:#?}", result);
}