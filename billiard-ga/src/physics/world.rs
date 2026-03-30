// src/physics/world.rs
use rapier2d::prelude::*;
use crate::model::TableState;

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
    /// 外部から与えられた状態(TableState)を元に、物理世界を初期化する
    pub fn new(state: &TableState) -> Self {
        let rigid_body_set = RigidBodySet::new();
        let collider_set = ColliderSet::new();

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

        world.setup_table(state);
        world
    }

    /// 状態データに基づく盤面のセットアップ
    fn setup_table(&mut self, state: &TableState) {
        let pocket_radius = 10.0;

        // 1. ポケットの配置
        let pocket_positions = [
            (0.0, 0.0), (500.0, 0.0), (1000.0, 0.0),
            (0.0, 500.0), (500.0, 500.0), (1000.0, 500.0),
        ];
        for &(px, py) in &pocket_positions {
            let pocket_collider = ColliderBuilder::ball(pocket_radius)
                .translation(vector![px, py])
                .sensor(true)
                .build();
            self.collider_set.insert(pocket_collider);
        }

        // 2. 四方の壁の配置
        let thickness = 10.0;
        let wall_restitution = 0.8;
        let walls = [
            (vector![500.0, -thickness], vector![500.0, thickness]),
            (vector![500.0, 500.0 + thickness], vector![500.0, thickness]),
            (vector![-thickness, 250.0], vector![thickness, 250.0]),
            (vector![1000.0 + thickness, 250.0], vector![thickness, 250.0]),
        ];

        for (pos, half_extents) in walls.iter() {
            let wall_collider = ColliderBuilder::cuboid(half_extents.x, half_extents.y)
                .translation(*pos)
                .restitution(wall_restitution)
                .friction(0.0)
                .build();
            self.collider_set.insert(wall_collider);
        }

        // 3. 球の配置（TableStateから動的に配置）
        
        // 手球 (ID: 0) の配置
        self.spawn_ball(0, state.cue_ball_pos.0, state.cue_ball_pos.1);

        // 的球の配置
        // state.remaining_balls は Vec<(u8, (f32, f32))> なので、要素を分解して渡す
        for ball in &state.remaining_balls { 
            let id = ball.0;
            let x = (ball.1).0;
            let y = (ball.1).1;
            self.spawn_ball(id, x, y); 
        }
    }

    /// 物理空間に球を1つ具現化するヘルパー関数
    fn spawn_ball(&mut self, id: u8, x: f32, y: f32) {
        let ball_radius = 2.85;

        let ball_body = RigidBodyBuilder::dynamic()
            .translation(vector![x, y])
            .locked_axes(LockedAxes::ROTATION_LOCKED)
            .linear_damping(0.05)
            .user_data(id as u128)
            .ccd_enabled(true)
            .build();
        let body_handle = self.rigid_body_set.insert(ball_body);

        let ball_collider = ColliderBuilder::ball(ball_radius)
            .restitution(0.9)
            .friction(0.0)
            .build();
        self.collider_set.insert_with_parent(ball_collider, body_handle, &mut self.rigid_body_set);
    }
}