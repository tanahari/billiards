// src/physics/mod.rs

pub mod world;
pub mod engine;

use nalgebra::Point2;
use crate::model::{ShotInput, ShotResult, TableState};
use crate::math::{calculate_aim_angle, is_shot_possible, calculate_cue_next_pos};
use self::world::PhysicsWorld;
use self::engine::run_simulation;

/// GAから呼ばれる「3ショット一括シミュレーション」の窓口
/// 3手分の入力を受け取り、一気にシミュレーションして最終的なスコアを返します。
pub fn simulate_sequence(state: &TableState, inputs: &[ShotInput; 3]) -> ShotResult {
    let ball_radius = 2.85;
    let v0 = 5.0; 
    let mu = 0.05;

    // 現在の状態を追跡するための変数
    let mut current_cue_pos = Point2::new(state.cue_ball_pos.0, state.cue_ball_pos.1);
    let mut total_base_score = 0.0;
    let mut success_count = 0;
    let mut is_all_valid = true;
    let mut final_is_scratch = false;

    // 3回のショットを順番に実行
    for input in inputs {
        // 1. 理論座標の取得（的球は本来 state から ID で取得しますが、ここではロジックを示します）
        let target_ball_tuple = state.get_ball_pos(input.target_ball_id);
        let target_ball_pos = Point2::new(target_ball_tuple.0, target_ball_tuple.1);
        let target_pocket_pos = get_pocket_pos(input.target_pocket_id);
        
        let other_balls: Vec<Point2<f32>> = state.get_all_ball_positions()
            .into_iter().map(|(x, y)| Point2::new(x, y)).collect();

        // 2. 【検証】打てるかどうかチェック
        if !is_shot_possible(current_cue_pos, target_ball_pos, target_pocket_pos, ball_radius, &other_balls) {
            is_all_valid = false;
            break; // 打てない手があれば、その時点で連撃終了
        }

        // 3. 角度とボーナス用 cosθ の計算
        let target_to_pocket = (target_pocket_pos - target_ball_pos).normalize();
        let ghost_ball_pos = target_ball_pos - target_to_pocket * (ball_radius * 2.0);
        let cue_dir = (ghost_ball_pos - current_cue_pos).normalize();
        let cos_theta = cue_dir.dot(&target_to_pocket);
        let aim_angle = calculate_aim_angle(current_cue_pos, target_ball_pos, target_pocket_pos, ball_radius);

        // 4. 【物理実行】
        let mut world = PhysicsWorld::new();
        run_simulation(&mut world, aim_angle, v0);

        // 5. 【個別評価】
        let shot_result = evaluate_single_shot(&world, input.target_ball_id, cos_theta, current_cue_pos, ghost_ball_pos, v0, mu);
        
        // スコアと成功数を積み上げ
        total_base_score += shot_result.score;
        if shot_result.is_success {
            success_count += 1;
        }
        
        // 状態更新：手球の位置を次の開始地点へ
        current_cue_pos = Point2::new(shot_result.end_cue_ball_pos.0, shot_result.end_cue_ball_pos.1);
        
        if shot_result.is_scratch {
            final_is_scratch = true;
            break; // スクラッチしたら即終了
        }
    }

    // 6. 【最終スコア計算】成功数に応じた倍率を適用
    // 3球入ったら×3、2球なら×2、1球なら×1
    let multiplier = success_count as f32;
    let final_score = total_base_score * multiplier;

    ShotResult {
        is_success: success_count == 3,
        score: final_score,
        end_cue_ball_pos: (current_cue_pos.x, current_cue_pos.y),
        is_scratch: final_is_scratch,
        is_valid: is_all_valid,
    }
}

/// 内部用：1ショットごとの純粋な評価（ボーナス込み）
fn evaluate_single_shot(
    world: &PhysicsWorld,
    target_id: u8,
    cos_theta: f32,
    cue_start_pos: Point2<f32>,
    contact_pos: Point2<f32>,
    v0: f32,
    mu: f32
) -> ShotResult {
    let target_id_u128 = target_id as u128;
    let is_scratch = world.rigid_body_set.iter().find(|(_, body)| body.user_data == 0).is_none();

    if is_scratch {
        return ShotResult {
            is_success: false, score: 0.0, end_cue_ball_pos: (cue_start_pos.x, cue_start_pos.y),
            is_scratch: true, is_valid: true,
        };
    }

    let target_body = world.rigid_body_set.iter().find(|(_, body)| body.user_data == target_id_u128);
    let is_success = target_body.is_none();

    let mut score = 0.0;
    if is_success {
        score = 1000.0 + (cos_theta * 100.0); // 基礎点 + 精度ボーナス
    }

    let end_cue_pos = calculate_cue_next_pos(cue_start_pos, contact_pos, v0, mu, 1.0);

    ShotResult {
        is_success, score, end_cue_ball_pos: (end_cue_pos.x, end_cue_pos.y),
        is_scratch: false, is_valid: true,
    }
}

fn get_pocket_pos(id: u8) -> Point2<f32> {
    let positions = [(0.0, 0.0), (500.0, 0.0), (1000.0, 0.0), (0.0, 500.0), (500.0, 500.0), (1000.0, 500.0)];
    let p = positions[(id as usize).min(5)];
    Point2::new(p.0, p.1)
}