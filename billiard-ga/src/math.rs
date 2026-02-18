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
    if cue_dir.dot(&target_to_pocket) <= 0.0 {
        return false; // 90度以上のカットは物理的に不可能 [cite: 78]
    }

    // --- 制約②：他球との干渉判定（カプセル衝突判定） [cite: 76] ---
    for &obstacle_pos in other_balls {
        // 的球そのものは障害物から除外する必要がある
        if (obstacle_pos - target_pos).norm() < 0.001 {
            continue;
        }

        // 線分（手球の軌跡）と他の球の中心との距離をチェック
        if distance_segment_to_point(cue_pos, ghost_ball_pos, obstacle_pos) < ball_radius * 2.0 {
            return false; // 経路上に他の球がある（障害物判定） [cite: 76]
        }
    }

    true
}

// ヘルパー関数
/// 点と線分の最短距離を計算する内部関数
fn distance_segment_to_point(a: Point2<f32>, b: Point2<f32>, p: Point2<f32>) -> f32 {
    let v = b - a; // 線分のベクトル
    let w = p - a; // 起点から対象点へのベクトル

    // 1. 線分 AB の長さの2乗
    let v_sq_norm = v.norm_squared();

    // 始点と終点が同じ（移動距離がゼロ）の場合は点 A との距離を返す
    if v_sq_norm == 0.0 {
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