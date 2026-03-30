pub fn simulate_shot(state: &TableState, input: &ShotInput) -> ShotResult {
    let ball_radius = 2.85;
    let v0 = 5.0; 
    let mu = 0.05;

    // 1. 【本物】状態(state)から座標を動的に取得する
    let cue_pos = Point2::new(state.cue_ball_pos.0, state.cue_ball_pos.1);

    let target_ball_tuple = state.get_ball_pos(input.target_ball_id);
    let target_ball_pos = Point2::new(target_ball_tuple.0, target_ball_tuple.1);

    let target_pocket_pos = get_pocket_pos(input.target_pocket_id);

    // 障害物判定のために、すべての的球の座標を Point2 の配列に変換
    let other_balls: Vec<Point2<f32>> = state.get_all_ball_positions()
        .into_iter()
        .map(|(x, y)| Point2::new(x, y))
        .collect();

    // 2. 【検証 (Validation)】
    if !is_shot_possible(cue_pos, target_ball_pos, target_pocket_pos, ball_radius, &other_balls) {
        return ShotResult {
            is_success: false,
            score: -500.0, 
            end_cue_ball_pos: (cue_pos.x, cue_pos.y),
            is_scratch: false,
            is_valid: false,
        };
    }

    // 3. 難易度ボーナス用の cosθ と、衝突地点（ゴーストボール）の計算
    let target_to_pocket = (target_pocket_pos - target_ball_pos).normalize();
    let ghost_ball_pos = target_ball_pos - target_to_pocket * (ball_radius * 2.0);
    let cue_dir = (ghost_ball_pos - cue_pos).normalize();
    let cos_theta = cue_dir.dot(&target_to_pocket);

    // 4. 打ち出し角度を計算し、シミュレーション実行
    let aim_angle = calculate_aim_angle(cue_pos, target_ball_pos, target_pocket_pos, ball_radius);

    let mut world = PhysicsWorld::new();
    // ※ 注意: 本当はここで world に対して state の座標通りに球を再配置する処理が必要です。
    // 今回の MVP では world.rs の初期配置を使ってテストする想定で進めます。

    run_simulation(&mut world, aim_angle, v0);

    // 5. 【評価と状態更新 (State Update)】
    // 衝突地点として ghost_ball_pos を渡す
    evaluate_final_state(&world, input, cos_theta, cue_pos, ghost_ball_pos, v0, mu)
}
/// 最終状態を観測し、仕様書通りの Score を計算する
fn evaluate_final_state(
    world: &PhysicsWorld,
    input: &ShotInput,
    cos_theta: f32,
    cue_start_pos: Point2<f32>,
    contact_pos: Point2<f32>, // ゴーストボールの座標を本物として受け取る
    v0: f32,
    mu: f32
) -> ShotResult {
    let target_id = input.target_ball_id as u128;

    // スクラッチ判定
    let is_scratch = world.rigid_bodyset.iter().find(|(, body)| body.user_data == 0).is_none();

    if is_scratch {
        return ShotResult {
            is_success: false,
            score: 0.0,
            end_cue_ball_pos: (cue_start_pos.x, cue_start_pos.y),
            is_scratch: true,
            is_valid: true,
        };
    }

    // 的球の成功判定
    let target_body = world.rigid_bodyset.iter().find(|(, body)| body.user_data == target_id);
    let is_success = target_body.is_none();

    // スコア加算ロジック
    let mut score = 0.0;
    if is_success {
        let base_score = 1000.0;
        // 【修正】仕様書通りに × 3.0 を適用
        let bonus = cos_theta * 100.0 * 3.0; 
        score = base_score + bonus;
    }

    // 状態更新：手球の最終座標をストップ/ドリフトの数式で上書き
    let end_cue_pos = calculate_cue_next_pos(cue_start_pos, contact_pos, v0, mu, 1.0);

    ShotResult {
        is_success,
        score,
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
    // IDが範囲外の場合はフェールセーフで 0番のポケットを返す
    let safe_id = (id as usize).min(5); 
    let p = positions[safe_id];
    Point2::new(p.0, p.1)
}
// src/model.rs
use serde::{Deserialize, Serialize};

/// GAからの要求（意図）
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ShotInput {
    pub target_ball_id: u8,
    pub target_pocket_id: u8, 
}

/// 計算結果（評価）
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShotResult {
    pub is_success: bool,
    pub score: f32, 
    pub end_cue_ball_pos: (f32, f32),
    pub is_scratch: bool, 
    pub is_valid: bool,
}

/// 【追加】現在の盤面の状態（現実）
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TableState {
    pub cue_ball_pos: (f32, f32), // 手球の現在位置
    // 盤面にある的球のリスト（ID, (x, y)）
    pub remaining_balls: Vec<(u8, (f32, f32))>, 
}

impl TableState {
    // IDから的球の座標を探す便利関数（mod.rsで使います）
    pub fn get_ball_pos(&self, target_id: u8) -> (f32, f32) {
        self.remainingballs.iter()
            .find(|(id, )| id == targetid)
            .map(|(, pos)|pos)
            .unwrap_or((0.0, 0.0)) // 見つからなかった場合のフェールセーフ
    }

    // すべての的球の座標だけを抽出する便利関数（障害物判定に使います）
    pub fn get_all_ball_positions(&self) -> Vec<(f32, f32)> {
        self.remainingballs.iter().map(|(, pos)| *pos).collect()
    }
}