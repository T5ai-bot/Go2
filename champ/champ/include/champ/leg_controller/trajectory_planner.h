#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>

namespace champ
{
    class TrajectoryPlanner
    {
        // ───────────────────── 成员变量 ─────────────────────
        QuadrupedLeg *leg_;                    // 指向本腿几何 & Gait 配置

        unsigned int total_control_points_;    // Bézier 控制点个数 (12)

        geometry::Transformation prev_foot_position_; // 上一帧足端位置（防止静态漂移）

        /* 预计算阶乘表，用于组合数 nCi
         * factorial_[k] = k!
         * n=11 ⇒ n! = 39916800
         */
        float factorial_[13];

        /* 参考控制点（标准化曲线）
         *   X 轴：-0.15 ~ 0.15 m
         *   Y 轴：-0.5  ~ -0.3214 (再缩放到 [-swing_height, 0])
         */
        float ref_control_points_x_[12];
        float ref_control_points_y_[12];

        /* 经实时缩放后的控制点（运行时使用） */
        float control_points_x_[12];
        float control_points_y_[12];

        float height_ratio_;   // = 实际 swing_height / 0.15
        float length_ratio_;   // = 实际 step_length / 0.4

        bool  run_once_;       // 首帧标志：保证 prev_foot_position_ 有效

        /* ────────── 内部函数：高度缩放 ──────────
         * 根据 gait_config->swing_height 缩放 Y 坐标
         */
        void updateControlPointsHeight(float swing_height)
        {
            float new_height_ratio = swing_height / 0.15f;          // 0.15 m 是参考高度

            if(height_ratio_ != new_height_ratio)                   // 避免重复运算
            {
                height_ratio_ = new_height_ratio;
                for(unsigned int i = 0; i < 12; i++)
                {
                    // Y 方向始终朝 -Z，所以是负数
                    control_points_y_[i] = -((ref_control_points_y_[i] * height_ratio_)
                                             + (0.5f * height_ratio_));
                }
            }
        }

        /* ────────── 内部函数：步长缩放 ──────────
         * 把参考曲线 X 轴按 step_length 线性拉伸
         */
        void updateControlPointsLength(float step_length)
        {
            float new_length_ratio = step_length / 0.4f;            // 0.4 m 是参考步长

            if(length_ratio_ != new_length_ratio)
            {
                length_ratio_ = new_length_ratio;
                for(unsigned int i = 0; i < 12; i++)
                {
                    if(i == 0)          // 第一控制点固定在 -L/2
                        control_points_x_[i] = -step_length / 2.0f;
                    else if(i == 11)    // 最后一点固定在 +L/2
                        control_points_x_[i] =  step_length / 2.0f;
                    else                // 中间点按比例缩放
                        control_points_x_[i] = ref_control_points_x_[i] * length_ratio_;
                }
            }
        }

        public:
            // ───────────────────── 构造函数 ─────────────────────
            TrajectoryPlanner(QuadrupedLeg &leg):
                leg_(&leg),
                total_control_points_(12),
                factorial_{1.0,1.0,2.0,6.0,24.0,120.0,720.0,5040.0,
                           40320.0,362880.0,3628800.0,39916800.0,479001600.0},
                ref_control_points_x_{-0.15, -0.2805, -0.3,  -0.3,  -0.3,
                                       0.0,   0.0,     0.0,   0.3032,0.3032,
                                       0.2826, 0.15},
                ref_control_points_y_{-0.5, -0.5,   -0.3611,-0.3611,-0.3611,
                                      -0.3611,-0.3611,-0.3214,-0.3214,-0.3214,
                                      -0.5,   -0.5},
                height_ratio_(0.0f),
                length_ratio_(0.0f),
                run_once_(false)
            {}

            /* ────────────────────────────────────────────────
             * generate()
             *   输入：  step_length          本周期步长 L
             *           rotation             轨迹投影旋转角 ψ
             *           swing_phase_signal   摆动相位 s_swing ∈ [0,1]
             *           stance_phase_signal  支撑相位 s_stance ∈ [0,1]
             *   输出：  foot_position        修改为新的足端期望
             * ──────────────────────────────────────────────── */
            void generate(geometry::Transformation &foot_position,
                          float   step_length,
                          float   rotation,
                          float   swing_phase_signal,
                          float   stance_phase_signal)
            {
                /* 1. 根据当前 gait_config 更新控制点高度 */
                updateControlPointsHeight(leg_->gait_config->swing_height);

                /* 2. 首帧初始化 prev_foot_position_ */
                if(!run_once_)
                {
                    run_once_ = true;
                    prev_foot_position_ = foot_position;
                }

                /* 3. 若 step_length = 0 → 原地踏步，不改变足端 */
                if(step_length == 0.0f)
                {
                    prev_foot_position_ = foot_position;
                    leg_->gait_phase(1);   // 置为“支撑”
                    return;
                }

                /* 4. 步长缩放控制点（x 方向） */
                updateControlPointsLength(step_length);

                int   n = total_control_points_ - 1;  // Bézier 阶数 (11)
                float x = 0.0f;                       // 轨迹局部坐标
                float y = 0.0f;

                /* ───── A. 支撑期 ─────
                 * s_stance > s_swing ⇒ 正处于支撑
                 * 使用倒 U 曲线扫地，保证接地顺滑
                 */
                if(stance_phase_signal > swing_phase_signal)
                {
                    leg_->gait_phase(1);   // 支撑

                    x = (step_length / 2) * (1 - (2 * stance_phase_signal));
                    y = -leg_->gait_config->stance_depth
                        * cosf((M_PI * x) / step_length);
                }
                /* ───── B. 摆动期 ─────
                 * 使用 11 阶 Bézier 曲线抬脚 → 前移 → 落脚
                 */
                else if(stance_phase_signal < swing_phase_signal)
                {
                    leg_->gait_phase(0);   // 摆动

                    for(unsigned int i = 0; i < total_control_points_; i++)
                    {
                        // 组合数 C(n,i) = n! / (i!(n-i)!)
                        float coeff = factorial_[n] /
                                      (factorial_[i] * factorial_[n - i]);

                        x += coeff * pow(swing_phase_signal,    i)
                                   * pow((1 - swing_phase_signal), (n - i))
                                   * control_points_x_[i];

                        y -= coeff * pow(swing_phase_signal,    i)
                                   * pow((1 - swing_phase_signal), (n - i))
                                   * control_points_y_[i];
                    }
                    /* 若想改成“简易余弦摆动”，即可用注释中的公式替换 */
                }

                /* ───── C. 投影到实际腿系 ───── */
                foot_position.X() += x * cosf(rotation);
                foot_position.Y() += x * sinf(rotation);
                foot_position.Z() += y;

                /* ───── D. 相位同步保护 ─────
                 * 若刚好回到 saw-tooth 零点，重置到上一帧位置，
                 * 避免数值抖动
                 */
                if((swing_phase_signal == 0.0f && stance_phase_signal == 0.0f)
                    && step_length > 0.0f)
                {
                    foot_position = prev_foot_position_;
                }

                /* 记录上一帧 */
                prev_foot_position_ = foot_position;
            }
    };
}

#endif // TRAJECTORY_PLANNER_H
