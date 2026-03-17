// src/physics/mod.rs

pub mod world;
pub mod engine;

use nalgebra::Point2;
use crate::model::{ShotInput, ShotResult, TableState};
use crate::math::{calculate_aim_angle, is_shot_possible, calculate_cue_next_pos};
use self::world::PhysicsWorld;
use self::engine::run_simulation;

// --- 物理定数 ---
const BALL_RADIUS: f32 = 2.85;
const SHOT_POWER_V0: f32 = 5.0;
const TABLE_FRICTION_MU: f32 = 0.05;
const CUE_BALL_ID: u128 = 0;

/// GAから呼ばれる「3ショット一括シミュレーション」
pub fn simulate_sequence(state: &TableState, inputs: &[ShotInput; 3]) -> ShotResult {
    // 1. 【宇宙の創生】この world が 3ショットの間、記憶を保持し続ける
    let mut world = PhysicsWorld::new();
    
    let mut total_base_score = 0.0;
    let mut success_count = 0;
    let mut is_valid = true;
    let mut is_scratch = false;
    
    // 初期状態の手球位置
    let mut last_cue_pos = Point2::new(state.cue_ball_pos.0, state.cue_ball_pos.1);

    for input in inputs {
        // ==========================================
        // 2. 【現状観測】「今の宇宙」から真実の座標を抜き出す
        // ==========================================
        
        // 手球の現在位置をスキャン (なければスクラッチとして終了)
        let Some(current_cue_pos) = get_ball_pos(&world, CUE_BALL_ID) else {
            is_scratch = true;
            break;
        };

        // ターゲット球の現在位置をスキャン (なければGAの選択ミスとして終了)
        let target_id_u128 = input.target_ball_id as u128;
        let Some(target_ball_pos) = get_ball_pos(&world, target_id_u128) else {
            is_valid = false;
            break; 
        };

        // 障害物判定用：ターゲットと手球以外のすべての球の座標
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
    }

    ShotResult {
        is_success: success_count == 3,
        score: total_base_score * (success_count as f32),
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

    // ターゲット球が存在しなければポケットイン成功
    let is_success = get_ball_pos(world, target_id as u128).is_none();
    
    let score = if is_success {
        1000.0 + (cos_theta * 100.0) // 基礎点 + 精度ボーナス
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

/// 指定したIDの球の現在位置を抽出する
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
    // 安全に取得し、範囲外の場合は適当なフォールバック（ここでは6番目のポケット）を返す
    let p = POSITIONS.get(id as usize).copied().unwrap_or(POSITIONS[5]);
    Point2::new(p.0, p.1)
}