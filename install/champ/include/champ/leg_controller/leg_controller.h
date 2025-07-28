#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_components.h>
#include <leg_controller/trajectory_planner.h>
#include <leg_controller/phase_generator.h>
#include <vector>          // ★为了 std::vector


namespace champ
{
    class LegController
    {
        // ─────────────────────────── 数据成员 ───────────────────────────
            QuadrupedBase *base_;                           // 指向机器人模型 & 参数

            champ::TrajectoryPlanner *trajectory_planners_[4]; // 按 LF, RF, LH, RH 存放指针

            // ---------- 工具：限幅 ----------
            float capVelocities(float velocity, float min_velocity, float max_velocity)
            {
                // clamp(x, lo, hi)
                return ((velocity)<(min_velocity)?(min_velocity):((velocity)>(max_velocity)?(max_velocity):(velocity)));
            }

        public:
            // ─────────── 构造函数 ───────────
            LegController(QuadrupedBase &quadruped_base, PhaseGenerator::Time time = PhaseGenerator::now()):
                base_(&quadruped_base),
                phase_generator(quadruped_base, time),  // 初始化相位生成器
                lf(base_->lf),                          // 四条腿各建一个 TrajectoryPlanner
                rf(base_->rf),
                lh(base_->lh),
                rh(base_->rh)
            {
                unsigned int total_legs = 0;

                // 保存指针便于 for 循环统一调用
                trajectory_planners_[total_legs++] = &lf;
                trajectory_planners_[total_legs++] = &rf;
                trajectory_planners_[total_legs++] = &lh;
                trajectory_planners_[total_legs++] = &rh;
/* =====  前/后腿模板  =====  (可以随时改点数/坐标) */
static const std::vector<float> front_x = {
    -0.15,-0.12,-0.09,-0.06,-0.03, 0.00, 0.03, 0.06, 0.09, 0.12, 0.15,
     0.15, 0.10, 0.05, 0.00,-0.05,-0.10,-0.15
};
static const std::vector<float> front_y = {
    -0.45,-0.45,-0.45,-0.45,-0.45,-0.45,-0.45,-0.45,-0.45,-0.45,-0.45,
    -0.45,-0.36,-0.29,-0.25,-0.29,-0.36,-0.45
};
static const std::vector<float> rear_x  = {
    -0.13,-0.10,-0.07,-0.04,-0.02, 0.02, 0.04, 0.07, 0.10, 0.13,
     0.13, 0.09, 0.06, 0.02,-0.02,-0.06,-0.09,-0.13
};
static const std::vector<float> rear_y  = {
    -0.48,-0.48,-0.48,-0.48,-0.48,-0.48,-0.48,-0.48,-0.48,-0.48,
    -0.48,-0.40,-0.34,-0.31,-0.34,-0.40,-0.48,-0.48
};

/* 把模板送进 4 条腿 —— 自动重采样成 20 控制点 */
lf.fitTemplate(front_x, front_y);   // 左前
rf.fitTemplate(front_x, front_y);   // 右前
lh.fitTemplate(rear_x,  rear_y);    // 左后
rh.fitTemplate(rear_x,  rear_y);    // 右后



            }

            // ─────────── 静态：零位→目标落点 → 步长 ↔ 轨迹方向 ───────────
            static void transformLeg(float &step_length, float &rotation, QuadrupedLeg &leg,
                              float step_x, float step_y, float theta)
            {
                // ① 用零姿态足端做刚体变换（平移 + 绕 Z 旋转）
                geometry::Transformation transformed_stance = leg.zero_stance();
                transformed_stance.Translate(step_x, step_y, 0.0f);
                transformed_stance.RotateZ(theta);

                // ② 计算零点 → 新落点 的位移 Δx, Δy
                float delta_x = transformed_stance.X() - leg.zero_stance().X();
                float delta_y = transformed_stance.Y() - leg.zero_stance().Y();

                // ③ Raibert 公式只给半周期，×2 得整步长
                step_length = sqrtf(pow(delta_x, 2) + pow(delta_y, 2)) * 2.0f;

                // ④ 足端轨迹在水平面的旋转角（供 TrajectoryPlanner 投影）
                rotation = atan2f(delta_y, delta_x);
            }

            // ─────────── 静态：Raibert 落点启发式 ───────────
            static float raibertHeuristic (float stance_duration, float target_velocity)
            {
                // Δp = 0.5 * T_stance * v
                return (stance_duration / 2.0f) * target_velocity;
            }

            // ─────────── 主入口：速度指令 → 足端期望 ───────────
            void velocityCommand(geometry::Transformation (&foot_positions)[4],
                                 champ::Velocities &req_vel,
                                 PhaseGenerator::Time time = PhaseGenerator::now())
            {
                // 1) 安全限幅 ★：防止指令超出机器极限
                req_vel.linear.x  = capVelocities(req_vel.linear.x,
                                                  -base_->gait_config.max_linear_velocity_x,
                                                   base_->gait_config.max_linear_velocity_x);
                req_vel.linear.y  = capVelocities(req_vel.linear.y,
                                                  -base_->gait_config.max_linear_velocity_y,
                                                   base_->gait_config.max_linear_velocity_y);
                req_vel.angular.z = capVelocities(req_vel.angular.z,
                                                  -base_->gait_config.max_angular_velocity_z,
                                                   base_->gait_config.max_angular_velocity_z);

                // 2) 将角速度映射为切向线速度（绕质心转）
                float tangential_velocity = req_vel.angular.z * base_->lf.center_to_nominal();

                // 3) 合速度 (用于选步频 & 相位)
                float velocity =  sqrtf(pow(req_vel.linear.x, 2) +
                                        pow(req_vel.linear.y + tangential_velocity, 2));

                // 4) Raibert 估算三自由度落点位移
                float step_x     = raibertHeuristic(base_->gait_config.stance_duration,
                                                    req_vel.linear.x);
                float step_y     = raibertHeuristic(base_->gait_config.stance_duration,
                                                    req_vel.linear.y);
                float step_theta = raibertHeuristic(base_->gait_config.stance_duration,
                                                    tangential_velocity);

                // 5) 将平旋落点位移换算成 等效足端旋转角 θ
                float theta = sinf((step_theta / 2) / base_->lf.center_to_nominal()) * 2;

                // 6) 逐腿计算步长 + 轨迹朝向
                float step_lengths[4]         = {0.0f, 0.0f, 0.0f, 0.0f};
                float trajectory_rotations[4] = {0.0f, 0.0f, 0.0f, 0.0f};
                float sum_of_steps            = 0.0f;

                for(unsigned int i = 0; i < 4; i++)
                {
                    transformLeg(step_lengths[i],            // ← 输出：步长 L_i
                                 trajectory_rotations[i],    // ← 输出：旋转角 ψ_i
                                 *base_->legs[i],            // ← 当前腿几何
                                 step_x, step_y, theta);     // ← 公共落点偏移
                    sum_of_steps += step_lengths[i];
                }

                // 7) 更新相位信号（摆动/支撑 sawtooth）
                phase_generator.run(velocity,              // 机器人线速度
                                    sum_of_steps / 4.0f,   // 平均步长
                                    time);                 // 当前时刻

                // 8) TrajectoryPlanner 根据相位生成足端期望
                for(unsigned int i = 0; i < 4; i++)
                {
                    trajectory_planners_[i]->generate(
                        foot_positions[i],              // 输出：期望足端位姿
                        step_lengths[i],                // 输入：步长
                        trajectory_rotations[i],        // 输入：旋转角
                        phase_generator.swing_phase_signal[i],  // 相位：摆动
                        phase_generator.stance_phase_signal[i]  // 相位：支撑
                    );
                }
            }

            // ─────────── 组件实例 ───────────
            champ::PhaseGenerator   phase_generator;   // 步态相位时钟
            champ::TrajectoryPlanner lf, rf, lh, rh;   // 四条腿轨迹规划器
    };
}

#endif // LEG_CONTROLLER_H
