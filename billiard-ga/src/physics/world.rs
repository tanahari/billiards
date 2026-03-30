// src/physics/world.rs
use rapier2d::prelude::*;

pub struct PhysicsWorld {
    pub gravity: Vector<f32>,
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,
    pub integration_parameters: IntegrationParameters,
    pub physics_pipeline: PhysicsPipeline,
    pub island_manager: IslandManager,
    pub broad_phase: BroadPhase,
    pub narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    pub multibody_joint_set: MultibodyJointSet,
    pub ccd_solver: CCDSolver,
}

impl PhysicsWorld {
    /// 新しい物理世界（ビリヤード台）を初期化する
    pub fn new() -> Self {
        let rigid_body_set = RigidBodySet::new();
        let collider_set = ColliderSet::new();

        // ビリヤードは真上から見下ろすため、重力はゼロ
        let integration_parameters = IntegrationParameters {
            dt: 1.0 / 60.0,
            ..Default::default()
        };

        let mut world = Self {
            gravity: vector![0.0, 0.0],
            rigid_body_set,
            collider_set,
            integration_parameters,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
        };

        world.setup_table();
        world
    }

    /// 設計書に基づく盤面のセットアップ
    fn setup_table(&mut self) {
        let ball_radius = 2.85; // 球の半径（適宜調整）
        let pocket_radius = 10.0; // ポケットの判定半径

        // 1. ポケットの配置（センサーとして定義）[cite: 1, 22-30]
        let pocket_positions = [
            (0.0, 0.0), (500.0, 0.0), (1000.0, 0.0),
            (0.0, 500.0), (500.0, 500.0), (1000.0, 500.0),
        ];
        for &(px, py) in &pocket_positions {
            let pocket_collider = ColliderBuilder::ball(pocket_radius)
                .translation(vector![px, py])
                .sensor(true) // 物理的な衝突はせず、重なりだけを検知する
                .build();
            self.collider_set.insert(pocket_collider);
        }

        // 2. 四方の壁（クッション）の配置
        // 1000x500 の領域を囲むように厚み10.0の壁を置く
        let thickness = 10.0;
        let wall_restitution = 0.8; // クッションの反発係数

        let walls = [
            // 下の壁 (中心 x=500, y=-10)
            (vector![500.0, -thickness], vector![500.0, thickness]),
            // 上の壁 (中心 x=500, y=510)
            (vector![500.0, 500.0 + thickness], vector![500.0, thickness]),
            // 左の壁 (中心 x=-10, y=250)
            (vector![-thickness, 250.0], vector![thickness, 250.0]),
            // 右の壁 (中心 x=1010, y=250)
            (vector![1000.0 + thickness, 250.0], vector![thickness, 250.0]),
        ];

        for (pos, half_extents) in walls.iter() {
            let wall_collider = ColliderBuilder::cuboid(half_extents.x, half_extents.y)
                .translation(*pos)
                .restitution(wall_restitution)
                .friction(0.0) // 壁との摩擦はゼロ（綺麗な反射のため）
                .build();
            self.collider_set.insert(wall_collider);
        }

        // 3. 球の配置（手球: 0, 的球: 1, 2, 3）
        // 設計書の初期配置座標 [cite: 1, 15-21]
        let initial_balls = [
            (0, 100.0, 100.0), // 手球 [cite: 1, 21]
            (1, 100.0, 200.0), // 的球1 [cite: 1, 18]
            (2, 150.0, 250.0), // 的球2 [cite: 1, 19]
            (3, 300.0, 100.0), // 的球3 [cite: 1, 20]
        ];

        for &(id, x, y) in &initial_balls {
            let ball_body = RigidBodyBuilder::dynamic()
                .translation(vector![x, y])
                .locked_axes(LockedAxes::ROTATION_LOCKED) // 回転のロック
                .linear_damping(0.05) // 設計書のラシャの摩擦係数 
                .user_data(id as u128) // IDを埋め込んでおく（後で判定に使うため）
                .ccd_enabled(true)
                .build();
            let body_handle = self.rigid_body_set.insert(ball_body);

            let ball_collider = ColliderBuilder::ball(ball_radius)
                .restitution(0.9) // 球同士の反発係数
                .friction(0.0)
                .build();
            self.collider_set.insert_with_parent(ball_collider, body_handle, &mut self.rigid_body_set);
        }
    }
}