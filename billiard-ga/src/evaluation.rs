use crate::model::{TableState, ShotInput};
use crate::physics::simulate_shot;
use nalgebra::Point2;

pub const NUM_BALLS: u8 = 3;
pub const NUM_POCKETS: u8 = 6;

/// GAの `main.rs` から呼ばれる評価の窓口
pub fn evaluate_individual(target_order: &[u8], pocket_selection: &[u8]) -> f64 {
    let mut state = get_initial_state();
    
    let mut angles: Vec<f64> = Vec::new();
    // ★ 変更: all_success(bool) をやめ、何球入ったか(usize)をカウントする
    let mut success_count = 0;

    for i in 0..(NUM_BALLS as usize) {
        let target_id = target_order[i];
        let pocket_id = pocket_selection[i];

        // 角度を計算して記録（たとえ失敗するショットでも、狙った角度は評価に使う）
        let cut_angle = calculate_cut_angle(&state, target_id, pocket_id);
        angles.push(cut_angle);

        let input = ShotInput {
            target_ball_id: target_id,
            target_pocket_id: pocket_id,
        };

        // 物理エンジンを実行
        let result = simulate_shot(&state, &input);

        // 失敗判定
        if !result.is_success || result.is_scratch || !result.is_valid {
            break; // 失敗したらその後の球は打てないのでループを抜ける
        }

        // ★ 変更: 成功したらカウントを増やす
        success_count += 1;
        
        // 盤面の更新
        state.cue_ball_pos = result.end_cue_ball_pos;
        state.remaining_balls.retain(|&(id, _)| id != target_id);
    }

    // 3. 採点（成功した球数と、そこまでの角度データを渡す）
    calculate_fitness(success_count, &angles)
}

/// 評価関数（ルール・価値観）
fn calculate_fitness(success_count: usize, angles: &[f64]) -> f64 {
    // 外した球数を計算（最大3）
    let missed_balls = (NUM_BALLS as usize) - success_count;
    
    // 1球外すごとに 3000点 のペナルティ（3球失敗=9000, 2球失敗=6000...）
    let mut score = (missed_balls as f64) * 3000.0; 

    // 挑戦したショットの角度の合計を足す（小さいほど優秀）
    let angle_sum: f64 = angles.iter().sum();
    score += angle_sum;

    score
}

// --- 以下、計算用のヘルパー関数（変更なし） ---

fn get_initial_state() -> TableState {
    TableState {
        cue_ball_pos: (100.0, 100.0),
        remaining_balls: vec![
            (1, (100.0, 200.0)),
            (2, (150.0, 250.0)),
            (3, (300.0, 100.0)),
        ],
    }
}

fn calculate_cut_angle(state: &TableState, target_id: u8, pocket_id: u8) -> f64 {
    let cue_pos = Point2::new(state.cue_ball_pos.0, state.cue_ball_pos.1);
    let target_tuple = state.get_ball_pos(target_id);
    let target_pos = Point2::new(target_tuple.0, target_tuple.1);
    
    let pocket_positions = [
        (0.0, 0.0), (500.0, 0.0), (1000.0, 0.0),
        (0.0, 500.0), (500.0, 500.0), (1000.0, 500.0),
    ];
    let safe_id = (pocket_id as usize).min(5);
    let pocket_pos = Point2::new(pocket_positions[safe_id].0, pocket_positions[safe_id].1);

    let ball_radius = 2.85;

    let target_to_pocket = (pocket_pos - target_pos).normalize();
    let ghost_ball_pos = target_pos - target_to_pocket * (ball_radius * 2.0);
    let cue_dir = (ghost_ball_pos - cue_pos).normalize();

    let dot_product = cue_dir.dot(&target_to_pocket).clamp(-1.0, 1.0);
    let angle_rad = dot_product.acos();
    
    angle_rad.to_degrees() as f64
}