// src/physics/engine.rs
use rapier2d::prelude::*;
use crate::physics::world::PhysicsWorld;

/// シミュレーションを実行し、全ての球が止まるまで時間を進める
pub fn run_simulation(world: &mut PhysicsWorld, angle: f32, power: f32) {
    // 1. 手球（ID: 0）に力積（Impulse）を与える
    apply_initial_impulse(world, angle, power);

    // 2. 時間発展ループ（Headlessシミュレーション）
    let mut ticks = 0;
    let max_ticks = 20000; // タイムアウト設定（約5分間止まらない事態を回避）

    while ticks < max_ticks {
        // 物理計算を1ステップ進める (dt = 1/60s)
        world.physics_pipeline.step(
            &world.gravity,
            &world.integration_parameters,
            &mut world.island_manager,
            &mut world.broad_phase,
            &mut world.narrow_phase,
            &mut world.rigid_body_set,
            &mut world.collider_set,
            &mut world.impulse_joint_set,
            &mut world.multibody_joint_set,
            &mut world.ccd_solver,
            None, // 物理イベントは手動でチェックするためNone
            &(),   // フックなし
        );

        // A. ポケット判定と球の削除
        check_and_handle_pockets(world);

        // B. 全球の停止判定
        if is_all_stopped(world) {
            break;
        }

        ticks += 1;
    }
}

/// 指定した角度と強さで手球を突き出す
fn apply_initial_impulse(world: &mut PhysicsWorld, angle: f32, power: f32) {
    let impulse = vector![angle.cos() * power, angle.sin() * power];
    
    // 全ての剛体から ID:0 (手球) を探して力を適用
    for (_handle, body) in world.rigid_body_set.iter_mut() {
        if body.user_data == 0 {
            // wake_up: true を指定することで、静止状態から物理演算を再開させる
            body.apply_impulse(impulse, true);
            break;
        }
    }
}

/// 各球の座標をチェックし、ポケットに入っていたら世界から削除する
fn check_and_handle_pockets(world: &mut PhysicsWorld) {
    let pocket_radius_sq = 15.0f32.powi(2); // 判定半径を少し広めに設定（15.0）
    let pocket_positions = [
        (0.0, 0.0), (500.0, 0.0), (1000.0, 0.0),
        (0.0, 500.0), (500.0, 500.0), (1000.0, 500.0),
    ];

    // 削除対象のハンドルを一時保存
    let mut to_remove = Vec::new();

    for (handle, body) in world.rigid_body_set.iter() {
        let pos = body.translation();
        for &(px, py) in &pocket_positions {
            let dist_sq = (pos.x - px).powi(2) + (pos.y - py).powi(2);
            if dist_sq < pocket_radius_sq {
                to_remove.push(handle);
                break;
            }
        }
    }

    // まとめて物理世界から削除
    for handle in to_remove {
        world.rigid_body_set.remove(
            handle,
            &mut world.island_manager,
            &mut world.collider_set,
            &mut world.impulse_joint_set,
            &mut world.multibody_joint_set,
            true, // 削除された剛体に付随するジョイントなども削除
        );
    }
}

/// すべての球が「スリープ」または「極低速」になったか判定する
fn is_all_stopped(world: &PhysicsWorld) -> bool {
    let velocity_threshold = 0.01; // $10^{-2}$ 以下の速度は停止とみなす

    for (_handle, body) in world.rigid_body_set.iter() {
        // スリープ（物理エンジンが「もう動かない」と判断）していればスキップ
        if body.is_sleeping() {
            continue;
        }
        // スリープしていなくても、速度が閾値を超えていれば「まだ動いている」
        if body.linvel().norm() > velocity_threshold {
            return false;
        }
    }
    true
}