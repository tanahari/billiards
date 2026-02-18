use nalgebra::Point2;

/// 的球をポケットに落とすための手球の打ち出し角度を計算する
pub fn calculate_aim_angle(
    cue_pos: Point2<f32>,
    target_pos: Point2<f32>,
    pocket_pos: Point2<f32>,
    ball_radius: f32,
) -> f32 {
    // 1. 的球からポケットへの方向ベクトル
    let target_to_pocket = pocket_pos - target_pos;
    let direction = target_to_pocket.normalize();

    // 2. ゴーストボールの中心座標 (的球の背後 2r の位置)
    let ghost_ball_pos = target_pos - direction * (ball_radius * 2.0);

    // 3. 手球からゴーストボールへのベクトル
    let cue_to_ghost = ghost_ball_pos - cue_pos;

    // 4. 角度をラジアンで返す (右向きが0)
    f32::atan2(cue_to_ghost.y, cue_to_ghost.x)
}

/// 物理的にショットが可能か（厚みと障害物）を判定する
pub fn is_shot_possible(
    cue_pos: Point2<f32>,
    target_pos: Point2<f32>,
    pocket_pos: Point2<f32>,
    ball_radius: f32,
    other_balls: &[Point2<f32>],
) -> bool {
    // 的球からポケットへの方向を取得 [cite: 77]
    let target_to_pocket = (pocket_pos - target_pos).normalize();
    
    // ゴーストボール（衝突時の手球中心）の位置 [cite: 111, 112]
    let ghost_ball_pos = target_pos - target_to_pocket * (ball_radius * 2.0);

    // 手球の進行ベクトル
    let cue_to_ghost = ghost_ball_pos - cue_pos;
    let cue_dir = cue_to_ghost.normalize();

    // --- 制約①：厚み（カット角）の判定 [cite: 78, 83] ---
    // 手球の進行方向と、的球がポケットへ向かう方向の内積
    if cue_dir.dot(&target_to_pocket) <= 0.05 { 
        return false; // 90度以上のカットは物理的に不可能 [cite: 78]
    }

// --- 制約②：他球との干渉判定（カプセル衝突判定） ---
    for &obstacle_pos in other_balls {
        // 的球「および手球自身」は障害物から除外する
        if (obstacle_pos - target_pos).norm() < 0.001 || (obstacle_pos - cue_pos).norm() < 0.001 {
            continue;
        }

        // 手球 -> ゴーストボールの軌道に障害物はないか？
        if distance_segment_to_point(cue_pos, ghost_ball_pos, obstacle_pos) < ball_radius * 2.0 - 1e-4 {
            return false; 
        }

        // 的球 -> ポケットの軌道に障害物はないか？
        if distance_segment_to_point(target_pos, pocket_pos, obstacle_pos) < ball_radius * 2.0 - 1e-4 {
            return false; 
        }
    }

    true
}

/// 設計書 [A. 物理制約] に基づく、衝突後の手球の最終座標を計算する
pub fn calculate_cue_next_pos(
    cue_start_pos: Point2<f32>, 
    contact_pos: Point2<f32>,   // 衝突地点（ゴーストボールの座標）
    v0: f32,                    // 初速 (定数 5.0) [cite: 14, 57]
    mu: f32,                    // 摩擦係数 (0.05) [cite: 13, 61]
    k_spin: f32,                // 回転の持ち係数 (例: 1.0) 
) -> Point2<f32> {
    let g = 9.81; // 重力加速度

    // 1. 閾値計算（バックスピンが解ける限界距離）[cite: 58-61]
    // 物理的に正しい停止距離の公式を使用
    let l_limit = (v0.powi(2) / (2.0 * mu * g)) * k_spin;

    // 2. 手球の元の進行方向（cue_dir）と移動距離を計算
    let cue_vector = contact_pos - cue_start_pos;
    let distance = (contact_pos - cue_start_pos).norm();

    // 進行方向の単位ベクトル（これがドリフトの方向になる）
    let cue_dir = cue_vector.normalize();

    // 3. ショット挙動判定 [cite: 62]
    if distance < l_limit {
        // 【Stop Mode】 [cite: 63]
        // 手球は衝突地点でほぼ停止する [cite: 64-65]
        contact_pos
    } else {
        // 【Drift Mode】 [cite: 66]
        // 衝突地点で止まらず、的球の進行方向へズルズル進む [cite: 67-69]
        
        // 余剰エネルギー分の計算（ここでは距離の差分に比例すると仮定）
        let excess_distance = distance - l_limit;
        
        // ドリフト係数（どれくらいズルズル進むかの割合。適宜調整）
        let drift_factor = 0.3; 
        
        // DriftVector = 進行方向 × 余剰距離 × 係数
        let drift_vector = cue_dir * (excess_distance * drift_factor);
        
        // 次手座標 [cite: 70]
        contact_pos + drift_vector
    }
}

// ヘルパー関数
/// 点と線分の最短距離を計算する内部関数
fn distance_segment_to_point(a: Point2<f32>, b: Point2<f32>, p: Point2<f32>) -> f32 {
    let v = b - a; // 線分のベクトル
    let w = p - a; // 起点から対象点へのベクトル

    // 1. 線分 AB の長さの2乗
    let v_sq_norm = v.norm_squared();

    // 始点と終点が同じ（移動距離がゼロ）の場合は点 A との距離を返す
    if v_sq_norm < 1e-6 {
        return (p - a).norm();
    }

    // 2. 正射影の比率 t を計算し、[0.0, 1.0] にクランプする
    let t = (w.dot(&v) / v_sq_norm).clamp(0.0, 1.0);

    // 3. 線分上の最も近い点 Q を求める
    let q = a + v * t;

    // 4. 点 P と点 Q の距離を返す
    (p - q).norm()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point2;

    #[test]
    fn test_ghost_ball_and_angle() {
        let cue = Point2::new(100.0, 100.0);
        let target = Point2::new(200.0, 100.0);
        let pocket = Point2::new(300.0, 100.0);
        let r = 2.85;

        // 真っ直ぐのショットなら、角度は 0（右方向）になるはず
        let angle = calculate_aim_angle(cue, target, pocket, r);
        assert!((angle - 0.0).abs() < 1e-5);

        // 真っ直ぐなら当然可能判定
        assert!(is_shot_possible(cue, target, pocket, r, &[]));
    }

    #[test]
    fn test_impossible_cut_angle() {
        let cue = Point2::new(100.0, 100.0);
        let target = Point2::new(200.0, 100.0);
        let pocket = Point2::new(100.0, 200.0); // 的球の「後ろ」にポケットがある（180度カット）
        let r = 2.85;

        // これは物理的に不可能なはず
        assert!(!is_shot_possible(cue, target, pocket, r, &[]));
    }

    #[test]
    fn test_obstacle_interference() {
        let cue = Point2::new(0.0, 0.0);
        let target = Point2::new(100.0, 0.0);
        let pocket = Point2::new(200.0, 0.0);
        let r = 5.0;

        // 経路上（x=50, y=2）に別の球を置く（半径5同士なら、中心距離10未満で衝突）
        let obstacles = vec![Point2::new(50.0, 2.0)];
        
        // 障害物に当たるので false になるはず
        assert!(!is_shot_possible(cue, target, pocket, r, &obstacles));
    }
}