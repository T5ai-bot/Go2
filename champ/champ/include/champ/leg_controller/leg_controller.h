#ifndef LEG_CONTROLLER_H
#define LEG_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_components.h>
#include <leg_controller/trajectory_planner.h>
#include <leg_controller/phase_generator.h>
#include <vector>     // std::vector
#include <cmath>      // sqrtf, powf, sinf, cosf, atan2f

namespace champ
{
// ------------------------- 你的 20 点轨迹（保留原样） -------------------------
static const std::vector<float> front_x = {
    -0.200, -0.180, -0.160, -0.140, -0.120, -0.100, -0.080, -0.060,
    -0.040, -0.020,  0.000,  0.020,  0.040,  0.060,  0.080,  0.100,
     0.120,  0.140,  0.160,  0.200
};

// 说明：这里名称写的是 y，但在 TrajectoryPlanner 中会投影/映射到足端轨迹坐标系；
// 我们在下方额外对“竖直 Z”做摆动期抬升增强，绕开 YAML 与控制点符号差异。
static const std::vector<float> front_y = {
     0.000,  0.010,  0.020,  0.035,  0.050,  0.065,  0.075,  0.080,
     0.080,  0.075,  0.070,  0.060,  0.045,  0.030,  0.020,  0.010,
     0.005,  0.002,  0.001,  0.000
};

static const std::vector<float> rear_x = {
    -0.200, -0.180, -0.160, -0.140, -0.120, -0.100, -0.080, -0.060,
    -0.040, -0.020,  0.000,  0.020,  0.040,  0.060,  0.080,  0.100,
     0.120,  0.140,  0.160,  0.200
};

static const std::vector<float> rear_y = {
     0.000,  0.008,  0.018,  0.030,  0.045,  0.058,  0.068,  0.072,
     0.072,  0.068,  0.062,  0.052,  0.040,  0.025,  0.015,  0.008,
     0.004,  0.002,  0.001,  0.000
};

// ============================================================================

class LegController
{
    // ───────── 数据成员 ─────────
    QuadrupedBase *base_;
    champ::TrajectoryPlanner *trajectory_planners_[4];

    // 工具：限幅
    static float capVelocities(float v, float lo, float hi)
    {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }

public:
    // ───────── 构造 ─────────
    LegController(QuadrupedBase &quadruped_base,
                  PhaseGenerator::Time time = PhaseGenerator::now()):
        base_(&quadruped_base),
        phase_generator(quadruped_base, time),
        // 前腿使用前腿模板，后腿使用后腿模板
        lf(base_->lf, front_x, front_y),
        rf(base_->rf, front_x, front_y),
        lh(base_->lh, rear_x,  rear_y),
        rh(base_->rh, rear_x,  rear_y)
    {
        unsigned int k = 0;
        trajectory_planners_[k++] = &lf;
        trajectory_planners_[k++] = &rf;
        trajectory_planners_[k++] = &lh;
        trajectory_planners_[k++] = &rh;
    }

    // ───────── 零位 → 落脚点位移 和 轨迹朝向 ─────────
    static void transformLeg(float &step_length, float &rotation, QuadrupedLeg &leg,
                             float step_x, float step_y, float theta)
    {
        geometry::Transformation T = leg.zero_stance();
        T.Translate(step_x, step_y, 0.0f);
        T.RotateZ(theta);

        const float dx = T.X() - leg.zero_stance().X();
        const float dy = T.Y() - leg.zero_stance().Y();

        step_length = std::sqrt(dx*dx + dy*dy) * 2.0f;   // Raibert 半周期 → ×2
        rotation    = std::atan2(dy, dx);
    }

    // ───────── Raibert 启发式 ─────────
    static float raibertHeuristic(float stance_duration, float v)
    {
        return 0.5f * stance_duration * v;
    }

    // ───────── 主入口：速度 → 足端轨迹 ─────────
    void velocityCommand(geometry::Transformation (&foot_positions)[4],
                         champ::Velocities &req_vel,
                         PhaseGenerator::Time time = PhaseGenerator::now())
    {
        // 1) 限幅
        req_vel.linear.x  = capVelocities(req_vel.linear.x,
                                           -base_->gait_config.max_linear_velocity_x,
                                            base_->gait_config.max_linear_velocity_x);
        req_vel.linear.y  = capVelocities(req_vel.linear.y,
                                           -base_->gait_config.max_linear_velocity_y,
                                            base_->gait_config.max_linear_velocity_y);
        req_vel.angular.z = capVelocities(req_vel.angular.z,
                                           -base_->gait_config.max_angular_velocity_z,
                                            base_->gait_config.max_angular_velocity_z);

        // 2) 角速度 → 切向速度（绕质心转动）
        const float r = base_->lf.center_to_nominal();
        const float v_tan = req_vel.angular.z * r;

        // 3) 合速度（定步频/相位）
        const float velocity = std::sqrt(req_vel.linear.x * req_vel.linear.x +
                                         (req_vel.linear.y + v_tan) * (req_vel.linear.y + v_tan));

        // 4) Raibert 落点
        const float step_x     = raibertHeuristic(base_->gait_config.stance_duration, req_vel.linear.x);
        const float step_y     = raibertHeuristic(base_->gait_config.stance_duration, req_vel.linear.y);
        const float step_theta = raibertHeuristic(base_->gait_config.stance_duration, v_tan);

        // 5) 等效平面旋转角（小角近似正弦展开）
        const float theta = std::sin((step_theta / 2.0f) / r) * 2.0f;

        // 6) 逐腿步长 & 轨迹朝向
        float step_lengths[4]         = {0.f, 0.f, 0.f, 0.f};
        float trajectory_rotations[4] = {0.f, 0.f, 0.f, 0.f};
        float sum_steps = 0.0f;

        for (unsigned int i = 0; i < 4; ++i)
        {
            transformLeg(step_lengths[i], trajectory_rotations[i], *base_->legs[i],
                         step_x, step_y, theta);
            sum_steps += step_lengths[i];
        }

        // 7) 相位更新（trot 相位定义在 PhaseGenerator 内部配置）
        phase_generator.run(velocity, sum_steps / 4.0f, time);

        // 8) 轨迹生成（先按你给的 20 点生成）
        for (unsigned int i = 0; i < 4; ++i)
        {
            trajectory_planners_[i]->generate(
                foot_positions[i],
                step_lengths[i],
                trajectory_rotations[i],
                phase_generator.swing_phase_signal[i],
                phase_generator.stance_phase_signal[i]
            );

            // 9) ★★★ 摆动期抬脚增强（只改本文件，不依赖 YAML）★★★
            // 余弦半波：s=0/1 时增量为 0，中间最高，避免边界跳变
            const float s  = phase_generator.swing_phase_signal[i];
            const float st = phase_generator.stance_phase_signal[i];

            if (s > 0.0f && st == 0.0f)
            {
                const float LIFT_BOOST = 0.06f;              // 额外抬高 ~6 cm；需要更高可改 0.08f
                const float PI_F       = 3.14159265f;
                const float boost      = LIFT_BOOST * (1.0f - std::cos(PI_F * s)) * 0.5f;

                // 只在竖直 Z 方向抬升，不改变 X/Y 落点
                foot_positions[i].Translate(0.0f, 0.0f, boost);
            }
        }
    }

    // ───────── 组件实例 ─────────
    champ::PhaseGenerator   phase_generator;
    champ::TrajectoryPlanner lf, rf, lh, rh;
};

} // namespace champ

#endif // LEG_CONTROLLER_H
