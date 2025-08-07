#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>
#include <vector>          // ★新增
#include <cassert>

namespace champ
{
    class TrajectoryPlanner
    {
        // ───────────────────── 成员变量 ─────────────────────
        QuadrupedLeg *leg_;                    // 指向本腿几何 & Gait 配置
        unsigned int total_control_points_;    // Bézier 控制点个数
        geometry::Transformation prev_foot_position_; // 上一帧足端位置（防止静态漂移）

        // ★ 修正：使用double防止大数溢出
        double factorial_[21];                 // 0! 到 20! 的预计算阶乘表

        // ★ 修正：将数组大小改为20以匹配逻辑
        float ref_control_points_x_[20];
        float ref_control_points_y_[20];
        float control_points_x_[20];
        float control_points_y_[20];

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
                for(unsigned int i = 0; i < total_control_points_; i++)
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
                // ★ 修正：使用 total_control_points_
                for(unsigned int i = 0; i < total_control_points_; i++)
                {
                    if(i == 0)          // 第一控制点固定在 -L/2
                        control_points_x_[i] = -step_length / 2.0f;
                    // ★ 修正：最后一个点的索引
                    else if(i == total_control_points_ - 1)
                        control_points_x_[i] =  step_length / 2.0f;
                    else                // 中间点按比例缩放
                        control_points_x_[i] = ref_control_points_x_[i] * length_ratio_;
                }
            }
        }

    public:
        // ───────────────────── 构造函数 ─────────────────────
        // ★ 修正：使用新的构造函数，接收轨迹模板并初始化所有成员
        TrajectoryPlanner(QuadrupedLeg &leg,
                          const std::vector<float>& template_x,
                          const std::vector<float>& template_y):
            leg_(&leg),
            total_control_points_(20),
            // ★ 修正：提供完整的21个阶乘值(0! 到 20!)
            factorial_{
                1.0, 1.0, 2.0, 6.0, 24.0, 120.0, 720.0, 5040.0,
                40320.0, 362880.0, 3628800.0, 39916800.0, 479001600.0,
                6227020800.0, 87178291200.0, 1307674368000.0,
                20922789888000.0, 355687428096000.0, 6402373705728000.0,
                121645100408832000.0, 2432902008176640000.0
            },
            height_ratio_(0.0f),
            length_ratio_(0.0f),
            run_once_(false)
        {
            // 在构造时，直接调用fitTemplate来加载和转换轨迹
            fitTemplate(template_x, template_y);
        }

        /* ────────────────────────────────────────────────
         * generate()
         * 输入：  step_length          本周期步长 L
         * rotation             轨迹投影旋转角 ψ
         * swing_phase_signal   摆动相位 s_swing ∈ [0,1]
         * stance_phase_signal  支撑相位 s_stance ∈ [0,1]
         * 输出：  foot_position        修改为新的足端期望
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

            int   n = total_control_points_ - 1;  // Bézier 阶数
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
             * 使用 Bézier 曲线抬脚 → 前移 → 落脚
             */
            else if(stance_phase_signal < swing_phase_signal)
            {
                leg_->gait_phase(0);   // 摆动
                for(unsigned int i = 0; i < total_control_points_; i++)
                {
                    // 组合数 C(n,i) = n! / (i!(n-i)!)
                    double coeff = factorial_[n] / (factorial_[i] * factorial_[n - i]);
                    x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_x_[i];
                    y -= coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_y_[i];
                }
            }

            /* ───── C. 投影到实际腿系 ───── */
            foot_position.X() += x * cosf(rotation);
            foot_position.Y() += x * sinf(rotation);
            foot_position.Z() += y;

            /* ───── D. 相位同步保护 ───── */
            if((swing_phase_signal == 0.0f && stance_phase_signal == 0.0f) && step_length > 0.0f)
            {
                foot_position = prev_foot_position_;
            }
            prev_foot_position_ = foot_position;
        }

        /* ─────────────  把任意长度模板拉伸/压缩 → 20 点 ───────────── */
        void fitTemplate(const std::vector<float>& tx, const std::vector<float>& ty)
        {
            assert(tx.size() == ty.size() && tx.size() >= 2);

            const size_t N = total_control_points_;
            const size_t M = tx.size();

            for (size_t i = 0; i < N; ++i)
            {
                double u = static_cast<double>(i) / (N - 1);   // 0-1
                double s = u * (M - 1);
                size_t k  = static_cast<size_t>(s);
                double w = s - k;
                if (k >= M - 1) { k = M - 2; w = 1.0; }

                ref_control_points_x_[i] = static_cast<float>((1 - w) * tx[k] + w * tx[k + 1]);
                ref_control_points_y_[i] = static_cast<float>((1 - w) * ty[k] + w * ty[k + 1]);
            }
            /* 运行时数组初始化 */
            for (size_t i = 0; i < N; ++i)
            {
                control_points_x_[i] = ref_control_points_x_[i];
                control_points_y_[i] = ref_control_points_y_[i];
            }
        }
    };
}

#endif // TRAJECTORY_PLANNER_H