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
        self.remaining_balls.iter()
            .find(|(id, _)| *id == target_id)
            .map(|(_, pos)| *pos)
            .unwrap_or((0.0, 0.0)) // 見つからなかった場合のフェールセーフ
    }

    // すべての的球の座標だけを抽出する便利関数（障害物判定に使います）
    pub fn get_all_ball_positions(&self) -> Vec<(f32, f32)> {
        self.remaining_balls.iter().map(|(_, pos)| *pos).collect()
    }
}