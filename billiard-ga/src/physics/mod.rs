// src/physics/mod.rs

pub mod world;
pub mod engine;

use nalgebra::Point2;
use crate::model::{ShotInput, ShotResult, TableState};
use crate::math::{is_shot_possible, calculate_cue_next_pos};

/// GAから呼ばれる「数学的」なシミュレーションと評価（爆速版）
pub fn simulate_shot(state: &TableState, input: &ShotInput) -> ShotResult {
    let ball_radius = 2.85;
    let v0 = 50.0; // 数学的な移動距離計算に使うパワー（適度な値）
    let mu = 0.05;

    // 1. 状態(state)から座標を取得
    let cue_pos = Point2::new(state.cue_ball_pos.0, state.cue_ball_pos.1);
    
    let target_ball_tuple = state.get_ball_pos(input.target_ball_id);
    let target_ball_pos = Point2::new(target_ball_tuple.0, target_ball_tuple.1);
    
    let target_pocket_pos = get_pocket_pos(input.target_pocket_id);
    
    let other_balls: Vec<Point2<f32>> = state.get_all_ball_positions()
        .into_iter()
        .map(|(x, y)| Point2::new(x, y))
        .collect();

    // 2. 【検証】数学的にショット可能か？（障害物や無理な角度の判定）
    if !is_shot_possible(cue_pos, target_ball_pos, target_pocket_pos, ball_radius, &other_balls) {
        return ShotResult {
            is_success: false, // 不可能な場合は失敗
            score: 0.0, 
            end_cue_ball_pos: (cue_pos.x, cue_pos.y),
            is_scratch: false,
            is_valid: false,
        };
    }

    // 3. 物理エンジンを使わず、数学的に「入った」と仮定して処理を進める
    let target_to_pocket = (target_pocket_pos - target_ball_pos).normalize();
    let ghost_ball_pos = target_ball_pos - target_to_pocket * (ball_radius * 2.0);

    // 4. お友達の秀逸な数式を使って、ショット後の手球の停止位置を計算する
    let end_cue_pos = calculate_cue_next_pos(cue_pos, ghost_ball_pos, v0, mu, 1.0);

    ShotResult {
        is_success: true, // 数学的に可能なら「入った」とみなす！
        score: 0.0,       // GA側で計算するのでスコアは不要
        end_cue_ball_pos: (end_cue_pos.x, end_cue_pos.y),
        is_scratch: false,
        is_valid: true,
    }
}

// ポケットID(0~5)を座標(Point2)に変換するヘルパー関数
fn get_pocket_pos(id: u8) -> Point2<f32> {
    let positions = [
        (0.0, 0.0), (500.0, 0.0), (1000.0, 0.0),
        (0.0, 500.0), (500.0, 500.0), (1000.0, 500.0),
    ];
    let safe_id = (id as usize).min(5); 
    let p = positions[safe_id];
    Point2::new(p.0, p.1)
}