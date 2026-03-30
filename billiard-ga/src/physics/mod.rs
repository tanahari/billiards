// src/physics/mod.rs

pub mod world;
pub mod engine;

use nalgebra::Point2;
use crate::model::{ShotInput, ShotResult, TableState};
use crate::math::{calculate_aim_angle, is_shot_possible, calculate_cue_next_pos};
use self::engine::run_simulation;
use self::world::PhysicsWorld;

// --- 物理定数 ---
const BALL_RADIUS: f32 = 2.85;
const SHOT_POWER_V0: f32 = 5.0;
const TABLE_FRICTION_MU: f32 = 0.05;
const CUE_BALL_ID: u128 = 0;

/// GAから呼ばれる「3ショット一括シミュレーション」
pub fn simulate_sequence(initial_state: &TableState, inputs: &[ShotInput; 3]) -> ShotResult {
    let mut current_state = initial_state.clone();
    
    let mut total_base_score = 0.0;
    let mut success_count = 0;
    let mut is_valid = true;
    let mut is_scratch = false;
    
    let mut last_cue_pos = Point2::new(initial_state.cue_ball_pos.0, initial_state.cue_ball_pos.1);

    for input in inputs {
        // 1. 【宇宙の創生】現在の状態からシミュレーション空間を構築
        let mut world = PhysicsWorld::new(&current_state);

        // ==========================================
        // 2. 【現状観測】
        // ==========================================
        
        let Some(current_cue_pos) = get_ball_pos(&world, CUE_BALL_ID) else {
            is_scratch = true;
            break;
        };

        let target_id_u128 = input.target_ball_id as u128;
        let Some(target_ball_pos) = get_ball_pos(&world, target_id_u128) else {
            is_valid = false;
            break; 
        };

        let other_balls: Vec<Point2<f32>> = world.rigid_body_set.iter()
            .filter_map(|(_, b)| {
                let id = b.user_data;
                if id != CUE_BALL_ID && id != target_id_u128 {
                    Some(Point2::new(b.position().translation.x, b.position().translation.y))
                } else {
                    None
                }
            })
            .collect();

        let target_pocket_pos = get_pocket_pos(input.target_pocket_id);

        // ==========================================
        // 3. 【検閲・計算・実行】
        // ==========================================
        if !is_shot_possible(current_cue_pos, target_ball_pos, target_pocket_pos, BALL_RADIUS, &other_balls) {
            is_valid = false;
            break; 
        }

        let target_to_pocket = (target_pocket_pos - target_ball_pos).normalize();
        let ghost_ball_pos = target_ball_pos - target_to_pocket * (BALL_RADIUS * 2.0);
        let cue_dir = (ghost_ball_pos - current_cue_pos).normalize();
        let cos_theta = cue_dir.dot(&target_to_pocket);
        let aim_angle = calculate_aim_angle(current_cue_pos, target_ball_pos, target_pocket_pos, BALL_RADIUS);

        run_simulation(&mut world, aim_angle, SHOT_POWER_V0);

        // ==========================================
        // 4. 【評価】
        // ==========================================
        let shot_result = evaluate_single_shot(&world, input.target_ball_id, cos_theta, current_cue_pos, ghost_ball_pos);
        
        total_base_score += shot_result.score;
        if shot_result.is_success { 
            success_count += 1; 
        }
        last_cue_pos = Point2::new(shot_result.end_cue_ball_pos.0, shot_result.end_cue_ball_pos.1);
        
        if shot_result.is_scratch {
            is_scratch = true;
            break; 
        }

        // 次のループのための状態更新
        current_state = extract_state(&world);
    }

    // 成功回数によるボーナス係数 (仕様に合わせて調整)
    let multiplier = match success_count {
        3 => 2.0,
        2 => 1.5,
        1 => 1.0,
        _ => 0.0,
    };

    ShotResult {
        is_success: success_count == 3,
        score: total_base_score * multiplier,
        end_cue_ball_pos: (last_cue_pos.x, last_cue_pos.y),
        is_scratch,
        is_valid,
    }
}

/// 内部用：1ショットごとの純粋な評価（ボーナス込み）
fn evaluate_single_shot(
    world: &PhysicsWorld,
    target_id: u8,
    cos_theta: f32,
    cue_start_pos: Point2<f32>,
    contact_pos: Point2<f32>,
) -> ShotResult {
    let is_scratch = get_ball_pos(world, CUE_BALL_ID).is_none();

    if is_scratch {
        return ShotResult {
            is_success: false, 
            score: 0.0, 
            end_cue_ball_pos: (cue_start_pos.x, cue_start_pos.y),
            is_scratch: true, 
            is_valid: true,
        };
    }

    let is_success = get_ball_pos(world, target_id as u128).is_none();
    
    let score = if is_success {
        1000.0 + (cos_theta * 100.0) 
    } else {
        0.0
    };

    let end_cue_pos = calculate_cue_next_pos(cue_start_pos, contact_pos, SHOT_POWER_V0, TABLE_FRICTION_MU, 1.0);

    ShotResult {
        is_success, 
        score, 
        end_cue_ball_pos: (end_cue_pos.x, end_cue_pos.y),
        is_scratch: false, 
        is_valid: true,
    }
}

// --- ヘルパー関数 ---

fn get_ball_pos(world: &PhysicsWorld, ball_id: u128) -> Option<Point2<f32>> {
    world.rigid_body_set.iter()
        .find(|(_, body)| body.user_data == ball_id)
        .map(|(_, body)| Point2::new(body.position().translation.x, body.position().translation.y))
}

fn get_pocket_pos(id: u8) -> Point2<f32> {
    const POSITIONS: [(f32, f32); 6] = [
        (0.0, 0.0), (500.0, 0.0), (1000.0, 0.0), 
        (0.0, 500.0), (500.0, 500.0), (1000.0, 500.0)
    ];
    let p = POSITIONS.get(id as usize).copied().unwrap_or(POSITIONS[5]);
    Point2::new(p.0, p.1)
}

/// 物理世界の現在状態からTableStateを抽出する
fn extract_state(world: &PhysicsWorld) -> TableState {
    let mut cue_ball_pos = (0.0, 0.0);
    let mut remaining_balls = Vec::new();

    for (_, body) in world.rigid_body_set.iter() {
        let id = body.user_data as u8;
        let pos = body.position().translation;
        if id == 0 {
            cue_ball_pos = (pos.x, pos.y);
        } else {
            remaining_balls.push((id, (pos.x, pos.y)));
        }
    }

    TableState {
        cue_ball_pos,
        remaining_balls,
    }
}