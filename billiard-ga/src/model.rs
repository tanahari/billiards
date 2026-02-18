// src/model.rs
use serde::{Deserialize, Serialize};

// GAからの要求
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ShotInput {
    // 遺伝子情報のコンパクト化のために u8 を採用
    pub target_ball_id: u8,   // 狙う的球ID [cite: 18-20]
    pub target_pocket_id: u8, // 狙うポケットID [cite: 23-29]
}

// 計算結果
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ShotResult {
    pub is_success: bool,
    pub fitness: f32,
    pub end_cue_ball_pos: (f32, f32),
    pub hit_target_first: bool,
}