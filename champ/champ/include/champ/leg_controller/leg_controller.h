/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

// ... (License information) ...

            ┌────────────────────────┐
cmd_vel ───►│ LegController.velocity─┼─► 四腿足端期望位姿 (Desired foot positions for 4 legs)
            │   Command()            │
            └────┬──────────┬────────┘
                 │          │
                 │          ├─► TrajectoryPlanner ×4
                 │                ‣ swing / stance 曲线生成 (swing/stance curve generation)
                 │
                 └─► PhaseGenerator
                       ‣ 计算 4 腿摆动/支撑相位 (calculates 4-leg swing/stance phases)

*/

// Header Guard: Prevents the file from being included multiple times
// 头文件保护宏：防止文件被重复包含
#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

// Include necessary dependencies
// 包含必要的依赖文件
#include <geometry/geometry.h>                         // For geometric transformations and calculations / 用于几何变换和计算
#include <quadruped_base/quadruped_base.h>             // Defines the robot's physical properties / 定义了机器人的物理属性
#include <quadruped_base/quadruped_components.h>       // Defines components like legs / 定义了腿等组件
#include <leg_controller/trajectory_planner.h>         // Class for planning a single leg's trajectory / 用于规划单腿轨迹的类
#include <leg_controller/phase_generator.h>            // Class for generating gait phases (the "metronome") / 用于生成步态相位的类 (步态“节拍器”)

// All code is within the "champ" namespace to avoid name conflicts
// 所有代码都位于 "champ" 命名空间内，以避免命名冲突
namespace champ
{
    // The LegController class orchestrates the entire process of converting velocity commands to foot positions.
    // LegController 类负责协调从速度指令到足端位置的整个过程。
    class LegController
    {
        // ─────────────────────────── 数据成员 (Data Members) ───────────────────────────
        // These members are private by default.
        // 默认情况下，这些成员是私有的。
            QuadrupedBase *base_;                           // 指向机器人模型 & 参数 / A pointer to the robot's model & parameters.

            champ::TrajectoryPlanner *trajectory_planners_[4]; // 按 LF, RF, LH, RH 存放指针 / An array of pointers to TrajectoryPlanners for each leg (LF, RF, LH, RH).

            // ---------- 工具：限幅 (Utility: Capping/Clamping) ----------
            // This function ensures the velocity command does not exceed the robot's physical limits.
            // 此函数确保速度指令不超过机器人的物理极限。
            float capVelocities(float velocity, float min_velocity, float max_velocity)
            {
                // clamp(x, lo, hi) - A compact way to write if/else if/else.
                // 一种紧凑的 if/else if/else 写法。
                return ((velocity)<(min_velocity)?(min_velocity):((velocity)>(max_velocity)?(max_velocity):(velocity)));
            }

        public:
            // ─────────── 构造函数 (Constructor) ───────────
            // This is called when a LegController object is created. It initializes all the components.
            // 当 LegController 对象被创建时，此函数会被调用，用于初始化所有组件。
            LegController(QuadrupedBase &quadruped_base, PhaseGenerator::Time time = PhaseGenerator::now()):
                base_(&quadruped_base),                     // Point `base_` to the provided robot base object. / 将 `base_` 指针指向传入的机器人对象。
                phase_generator(quadruped_base, time),      // 初始化相位生成器 / Initialize the phase generator.
                lf(base_->lf),                              // 四条腿各建一个 TrajectoryPlanner / Create a TrajectoryPlanner for each of the four legs.
                rf(base_->rf),
                lh(base_->lh),
                rh(base_->rh)
            {
                unsigned int total_legs = 0;

                // 保存指针便于 for 循环统一调用 / Store the pointers to the planners in the array for easy access in loops.
                trajectory_planners_[total_legs++] = &lf;
                trajectory_planners_[total_legs++] = &rf;
                trajectory_planners_[total_legs++] = &lh;
                trajectory_planners_[total_legs++] = &rh;

                // leg_controller.h  ─── 新增示例 (放在 trajectory_planners_[] 数组填完之后)
// ───────────── 左前腿 LF ─────────────
const float lf_cp_x[12] = { -0.18f, -0.32f, -0.35f, -0.36f, -0.36f,
                             0.00f,  0.00f,  0.00f,
                             0.36f,  0.36f,  0.33f,  0.18f };

const float lf_cp_y[12] = { -0.50f, -0.50f, -0.38f, -0.38f, -0.38f,
                            -0.34f, -0.34f, -0.32f,
                            -0.32f, -0.32f, -0.50f, -0.50f };

// ───────────── 右前腿 RF ─────────────
const float rf_cp_x[12] = { -0.16f, -0.28f, -0.31f, -0.32f, -0.32f,
                             0.00f,  0.00f,  0.00f,
                             0.32f,  0.32f,  0.29f,  0.16f };

const float rf_cp_y[12] = { -0.50f, -0.50f, -0.37f, -0.37f, -0.37f,
                            -0.33f, -0.33f, -0.31f,
                            -0.31f, -0.31f, -0.50f, -0.50f };

// ───────────── 左后腿 LH ─────────────
const float lh_cp_x[12] = { -0.20f, -0.34f, -0.38f, -0.39f, -0.39f,
                             0.00f,  0.00f,  0.00f,
                             0.39f,  0.39f,  0.35f,  0.20f };

const float lh_cp_y[12] = { -0.50f, -0.50f, -0.40f, -0.40f, -0.40f,
                            -0.36f, -0.36f, -0.34f,
                            -0.34f, -0.34f, -0.50f, -0.50f };

// ───────────── 右后腿 RH ─────────────
const float rh_cp_x[12] = { -0.19f, -0.33f, -0.37f, -0.38f, -0.38f,
                             0.00f,  0.00f,  0.00f,
                             0.38f,  0.38f,  0.34f,  0.19f };

const float rh_cp_y[12] = { -0.50f, -0.50f, -0.39f, -0.39f, -0.39f,
                            -0.35f, -0.35f, -0.33f,
                            -0.33f, -0.33f, -0.50f, -0.50f };
lf.setReferenceControlPoints(lf_cp_x, lf_cp_y);
rf.setReferenceControlPoints(rf_cp_x, rf_cp_y);
lh.setReferenceControlPoints(lh_cp_x, lh_cp_y);
rh.setReferenceControlPoints(rh_cp_x, rh_cp_y);


            }

            // ─────────── 静态：零位→目标落点 → 步长 ↔ 轨迹方向 (Static: Zero-pos to Target -> Step ↔ Trajectory Dir) ───────────
            // A static function belongs to the class, not an object. It calculates step length and direction for a single leg.
            // 静态函数属于类本身，而非某个对象。它为单条腿计算步长和轨迹方向。
            static void transformLeg(float &step_length, float &rotation, QuadrupedLeg &leg,
                              float step_x, float step_y, float theta)
            {
                // ① 用零姿态足端做刚体变换（平移 + 绕 Z 旋转）/ Perform a rigid body transformation (translate + rotate around Z) on the zero-stance foot position.
                geometry::Transformation transformed_stance = leg.zero_stance();
                transformed_stance.Translate(step_x, step_y, 0.0f);
                transformed_stance.RotateZ(theta);

                // ② 计算零点 → 新落点 的位移 Δx, Δy / Calculate the displacement (Δx, Δy) from the zero position to the new footfall location.
                float delta_x = transformed_stance.X() - leg.zero_stance().X();
                float delta_y = transformed_stance.Y() - leg.zero_stance().Y();

                // ③ Raibert 公式只给半周期，×2 得整步长 / Raibert's formula gives a half-period step, so multiply by 2 to get the full step length.
                step_length = sqrtf(pow(delta_x, 2) + pow(delta_y, 2)) * 2.0f;

                // ④ 足端轨迹在水平面的旋转角（供 TrajectoryPlanner 投影）/ The rotation angle of the foot trajectory in the horizontal plane (for the TrajectoryPlanner to project).
                rotation = atan2f(delta_y, delta_x);
            }

            // ─────────── 静态：Raibert 落点启发式 (Static: Raibert's Heuristic) ───────────
            // This simple but effective formula calculates where the next footstep should land to maintain balance.
            // 这个简单而有效的公式计算下一落脚点的位置以保持平衡。
            static float raibertHeuristic (float stance_duration, float target_velocity)
            {
                // Δp = 0.5 * T_stance * v
                return (stance_duration / 2.0f) * target_velocity;
            }

            // ─────────── 主入口：速度指令 → 足端期望 (Main Entry Point: Velocity Command -> Desired Foot Positions) ───────────
            // This is the core function that translates a velocity command into desired foot positions.
            // 这是将速度指令转换为期望足端位置的核心函数。
            void velocityCommand(geometry::Transformation (&foot_positions)[4],
                                 champ::Velocities &req_vel,
                                 PhaseGenerator::Time time = PhaseGenerator::now())
            {
                // 1) 安全限幅 ★：防止指令超出机器极限 / Safety Limiting ★: Prevent commands from exceeding the robot's limits.
                req_vel.linear.x  = capVelocities(req_vel.linear.x,
                                                  -base_->gait_config.max_linear_velocity_x,
                                                   base_->gait_config.max_linear_velocity_x);
                req_vel.linear.y  = capVelocities(req_vel.linear.y,
                                                  -base_->gait_config.max_linear_velocity_y,
                                                   base_->gait_config.max_linear_velocity_y);
                req_vel.angular.z = capVelocities(req_vel.angular.z,
                                                  -base_->gait_config.max_angular_velocity_z,
                                                   base_->gait_config.max_angular_velocity_z);

                // 2) 将角速度映射为切向线速度（绕质心转）/ Map angular velocity to tangential linear velocity (for turning around the center of mass).
                float tangential_velocity = req_vel.angular.z * base_->lf.center_to_nominal();

                // 3) 合速度 (用于选步频 & 相位) / Combine velocities to get a magnitude (used for selecting step frequency & phase).
                float velocity =  sqrtf(pow(req_vel.linear.x, 2) +
                                        pow(req_vel.linear.y + tangential_velocity, 2));

                // 4) Raibert 估算三自由度落点位移 / Use Raibert's heuristic to estimate the 3-DOF footfall displacement.
                float step_x     = raibertHeuristic(base_->gait_config.stance_duration,
                                                    req_vel.linear.x);
                float step_y     = raibertHeuristic(base_->gait_config.stance_duration,
                                                    req_vel.linear.y);
                float step_theta = raibertHeuristic(base_->gait_config.stance_duration,
                                                    tangential_velocity);

                // 5) 将平旋落点位移换算成 等效足端旋转角 θ / Convert the rotational footfall displacement into an equivalent body rotation angle θ.
                float theta = sinf((step_theta / 2) / base_->lf.center_to_nominal()) * 2;

                // 6) 逐腿计算步长 + 轨迹朝向 / For each leg, calculate its step length and trajectory orientation.
                float step_lengths[4]         = {0.0f, 0.0f, 0.0f, 0.0f};
                float trajectory_rotations[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                float sum_of_steps            = 0.0f;

                for(unsigned int i = 0; i < 4; i++)
                {
                    transformLeg(step_lengths[i],            // ← 输出：步长 L_i / Output: Step length L_i
                                 trajectory_rotations[i],    // ← 输出：旋转角 ψ_i / Output: Rotation angle ψ_i
                                 *base_->legs[i],            // ← 当前腿几何 / Input: Current leg geometry
                                 step_x, step_y, theta);     // ← 公共落点偏移 / Input: Common footfall offset
                    sum_of_steps += step_lengths[i];
                }

                // 7) 更新相位信号（摆动/支撑 sawtooth）/ Update the phase signals (swing/stance sawtooth waves).
                phase_generator.run(velocity,              // 机器人线速度 / Robot's linear velocity
                                    sum_of_steps / 4.0f,   // 平均步长 / Average step length
                                    time);                 // 当前时刻 / Current time

                // 8) TrajectoryPlanner 根据相位生成足端期望 / Use TrajectoryPlanners to generate desired foot positions based on the phase.
                for(unsigned int i = 0; i < 4; i++)
                {
                    trajectory_planners_[i]->generate(
                        foot_positions[i],              // 输出：期望足端位姿 / Output: Desired foot position
                        step_lengths[i],                // 输入：步长 / Input: Step length
                        trajectory_rotations[i],        // 输入：旋转角 / Input: Rotation angle
                        phase_generator.swing_phase_signal[i],  // 相位：摆动 / Input Phase: Swing
                        phase_generator.stance_phase_signal[i]  // 相位：支撑 / Input Phase: Stance
                    );
                }
            }

            // ─────────── 组件实例 (Component Instances) ───────────
            champ::PhaseGenerator   phase_generator;   // 步态相位时钟 / The gait phase clock (metronome).
            champ::TrajectoryPlanner lf, rf, lh, rh;   // 四条腿轨迹规划器 / Trajectory planners for the four legs.
    };
}

#endif // LEG_CONTROLLER_H