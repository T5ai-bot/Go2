#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>
#include <vector>          // ★新增
#include <cassert>
#include <algorithm>  // for std::minmax_element
#include <cmath>      // for fabsf
namespace champ
{
    class TrajectoryPlanner
    {
        // ───────────────────── 成员变量 ─────────────────────
        QuadrupedLeg *leg_;                    // 指向本腿几何 & Gait 配置
        unsigned int total_control_points_;    // Bézier 控制点个数
        geometry::Transformation prev_foot_position_; // 上一帧足端位置（防止静态漂移）
// 模板高度范围（用于把模板y归一化到[0,1]）        
        float ref_y_min_ = 0.0f;
        float ref_y_max_ = 1.0f;        
float ref_y_ground_ = 0.0f;   // 端点平均
float ref_y_peak_   = 1.0f;   // 中段极值

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
            // 峰-地面振幅
            float denom = ref_y_peak_ - ref_y_ground_;
            float inv   = (std::fabs(denom) > 1e-6f) ? (1.0f / denom) : 0.0f;
            for (unsigned int i = 0; i < total_control_points_; ++i) {
                float y = ref_control_points_y_[i];
                // 归一化：ground→0, peak→1；自动适配正/负峰
                float y_norm = (inv != 0.0f) ? (y - ref_y_ground_) * inv : 0.0f;
                // 裁剪到 [0,1]，保持形状不变（只裁边界）

                // 不要 clamp 到 [0,1] pace 同侧同相，落脚时的“顺滑度”比 trot 更敏感，别把端点附近的斜率抹掉。
                // if (y_norm < 0.0f) y_norm = 0.0f;
                // if (y_norm > 1.0f) y_norm = 1.0f;

                // 仍沿用你的管线：Y 表示竖直、向下为负；最终会加到 Z
                control_points_y_[i] = -(y_norm * swing_height);
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
            if(stance_phase_signal >= swing_phase_signal) //把“=”并入支撑：
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
                // for(unsigned int i = 0; i < total_control_points_; i++)
                // {
                //     // 组合数 C(n,i) = n! / (i!(n-i)!)
                //     double coeff = factorial_[n] / (factorial_[i] * factorial_[n - i]);
                //     x += coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_x_[i];
                //     y -= coeff * pow(swing_phase_signal, i) * pow((1 - swing_phase_signal), (n - i)) * control_points_y_[i];
                // }
auto bezier_decasteljau = [&](float s, const float* px, const float* py, unsigned n, float& bx, float& by){
    double X[21], Y[21]; // n<=20 足够
    for (unsigned i = 0; i <= n; ++i) { X[i] = px[i]; Y[i] = py[i]; }
    for (unsigned r = 1; r <= n; ++r)
        for (unsigned i = 0; i <= n - r; ++i) {
            X[i] = (1.0 - s) * X[i] + s * X[i + 1];
            Y[i] = (1.0 - s) * Y[i] + s * Y[i + 1];
        }
    bx += (float)X[0];
    by -= (float)Y[0]; // 你原本的符号约定保留
};
// 摆动段：
bezier_decasteljau(swing_phase_signal, control_points_x_, control_points_y_, n, x, y);

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
            // === 标定地面与峰值 ===
            if (total_control_points_ > 0) {
                // 1) 地面：端点平均（可避免中段异常点影响）
                float y0  = ref_control_points_y_[0];
                float yN  = ref_control_points_y_[total_control_points_-1];
                ref_y_ground_ = 0.5f * (y0 + yN);

                // 2) 峰值：在中段 [20%, 80%] 搜索极值
                unsigned i_beg = (unsigned)std::floor(0.20f * (total_control_points_-1));
                unsigned i_end = (unsigned)std::ceil (0.80f * (total_control_points_-1));
                if (i_end <= i_beg) { i_beg = 0; i_end = total_control_points_-1; }

                float y_min =  1e9f, y_max = -1e9f;
                for (unsigned i = i_beg; i <= i_end; ++i) {
                    float y = ref_control_points_y_[i];
                    if (y < y_min) y_min = y;
                    if (y > y_max) y_max = y;
                }
                // 模板可能向下为负（y_min 更“低”）或向上为正（y_max 更“高”）
                // 谁远离 ground 就取谁做峰值
                float d_min = std::fabs(y_min - ref_y_ground_);
                float d_max = std::fabs(y_max - ref_y_ground_);
                ref_y_peak_ = (d_min >= d_max) ? y_min : y_max;
            }       
// 在 fitTemplate() 后强制单调 
for (size_t i = 1; i < N; ++i)
    if (ref_control_points_x_[i] <= ref_control_points_x_[i-1])
        ref_control_points_x_[i] = ref_control_points_x_[i-1] + 1e-4f;
            
        }
    };
}

#endif // TRAJECTORY_PLANNER_H