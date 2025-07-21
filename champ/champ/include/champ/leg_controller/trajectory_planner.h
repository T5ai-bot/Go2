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
*/

#ifndef TRAJECTORY_PLANNER_H
#define TRAJECTORY_PLANNER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_leg.h>

#include <cstring>

namespace champ
{
class TrajectoryPlanner
{
    // ───────────────────── 成员变量 | data members ─────────────────────
    QuadrupedLeg *leg_;                    // 本腿几何 & gait_config | leg kinematics & parameters

    unsigned int total_control_points_;    // Bézier 控制点数量 (12) | number of CPs

    geometry::Transformation prev_foot_position_; // 上一帧足端，用于数值回滚 | last pose

    /* 0!…12! 预计算表 | factorial table for nCi */
    float factorial_[13];

    /* ── 模板控制点 (参考曲线，单位 m) ──
     * X: [-0.15, 0.15]  前后
     * Y: [-0.5, -0.3214] 下蹲高度 (负数 = 向下)
     */
    float ref_control_points_x_[12];
    float ref_control_points_y_[12];

    /* 运行时缩放后的控制点 | scaled CPs */
    float control_points_x_[12];
    float control_points_y_[12];

    float height_ratio_;   // = swing_height / 0.15
    float length_ratio_;   // = step_length / 0.4

    bool  run_once_;       // 首帧标志 | first-run flag

    /* ────────── 缩放: 抬脚高度 | height scaling ────────── */
    void updateControlPointsHeight(float swing_height)
    {
        float new_height_ratio = swing_height / 0.15f;    // 0.15 m = template H

        if(height_ratio_ != new_height_ratio)             // 若发生变化再计算 | recalc only if changed
        {
            height_ratio_ = new_height_ratio;
            for(unsigned int i = 0; i < 12; ++i)
            {
                // Y 方向始终朝 -Z ⇒ 负数 | Y is neg. (downwards)
                control_points_y_[i] = -((ref_control_points_y_[i] * height_ratio_)
                                         + (0.5f * height_ratio_));
            }
        }
    }

    /* ────────── 缩放: 步长 | length scaling ────────── */
    void updateControlPointsLength(float step_length)
    {
        float new_length_ratio = step_length / 0.4f;      // 0.4 m = template stride

        if(length_ratio_ != new_length_ratio)
        {
            length_ratio_ = new_length_ratio;
            for(unsigned int i = 0; i < 12; ++i)
            {
                if(i == 0)          // 起点固定在 -L/2 | start CP
                    control_points_x_[i] = -step_length / 2.0f;
                else if(i == 11)    // 终点固定在 +L/2 | end CP
                    control_points_x_[i] =  step_length / 2.0f;
                else                // 其他点按比例缩放 | scale linearly
                    control_points_x_[i] = ref_control_points_x_[i] * length_ratio_;
            }
        }
    }

public:

    void setReferenceControlPoints(const float cp_x[12],
                                   const float cp_y[12])
    {
        std::memcpy(ref_control_points_x_, cp_x, 12 * sizeof(float));
        std::memcpy(ref_control_points_y_, cp_y, 12 * sizeof(float));
        // 让缩放表失效，下一帧自动重算
        length_ratio_ = 0.0f;
        height_ratio_ = 0.0f;
    }


    // ───────────────────── 构造函数 | constructor ─────────────────────
    explicit TrajectoryPlanner(QuadrupedLeg &leg)
        : leg_(&leg),
          total_control_points_(12),
          factorial_{1,1,2,6,24,120,720,5040,
                     40320,362880,3628800,39916800,479001600},
          ref_control_points_x_{-0.15, -0.2805, -0.3,  -0.3,  -0.3,
                                 0.0,   0.0,     0.0,   0.3032,0.3032,
                                 0.2826, 0.15},
          ref_control_points_y_{-0.5,  -0.5,    -0.3611,-0.3611,-0.3611,
                               -0.3611,-0.3611, -0.3214,-0.3214,-0.3214,
                               -0.5,   -0.5},
          height_ratio_(0.0f),
          length_ratio_(0.0f),
          run_once_(false)
    {}

    /* ────────────────────────────────────────────────
     * generate()
     *   输入 | inputs:
     *      step_length          本周期步长 L
     *      rotation             轨迹方向 ψ (水平面)
     *      swing_phase_signal   摆动相位 s_swing ∈ [0,1]
     *      stance_phase_signal  支撑相位 s_stance ∈ [0,1]
     *   输出 | output:
     *      foot_position        期望足端位姿 (in/out)
     * ──────────────────────────────────────────────── */
    void generate(geometry::Transformation &foot_position,
                  float step_length,
                  float rotation,
                  float swing_phase_signal,
                  float stance_phase_signal)
    {
        // 1) 更新抬脚高度 | update Y scaling
        updateControlPointsHeight(leg_->gait_config->swing_height);

        // 2) 首帧记录当前位置 | init prev_ pose
        if(!run_once_)
        {
            run_once_ = true;
            prev_foot_position_ = foot_position;
        }

        // 3) 步长为 0 → 原地踏步，只维持支撑 | in-place stepping
        if(step_length == 0.0f)
        {
            prev_foot_position_ = foot_position;
            leg_->gait_phase(1);   // 标记支撑 | mark stance
            return;
        }

        // 4) 根据步长缩放 X 方向控制点 | scale stride
        updateControlPointsLength(step_length);

        const int n = total_control_points_ - 1; // Bézier 阶数 (11)
        float x = 0.0f;   // 局部坐标 local X
        float y = 0.0f;   // local Z (down is neg)

        /* ───────── A. 支撑期 | stance ───────── */
        if(stance_phase_signal > swing_phase_signal)
        {
            leg_->gait_phase(1); // 支撑 | mark stance

            // x 在 [-L/2, L/2] 线性移动 | slide along ground
            x = (step_length / 2) * (1 - (2 * stance_phase_signal));

            // y 用浅余弦曲线，保持足端几乎贴地 | shallow U
            y = -leg_->gait_config->stance_depth *
                cosf((M_PI * x) / step_length);
        }
        /* ───────── B. 摆动期 | swing ───────── */
        else if(stance_phase_signal < swing_phase_signal)
        {
            leg_->gait_phase(0); // 摆动

            // Bézier 曲线求和 | compute 11th-order Bézier
            for(unsigned int i = 0; i < total_control_points_; ++i)
            {
                float coeff = factorial_[n] /
                              (factorial_[i] * factorial_[n - i]);

                x += coeff * pow(swing_phase_signal, i) *
                           pow(1 - swing_phase_signal, n - i) *
                           control_points_x_[i];

                y -= coeff * pow(swing_phase_signal, i) *
                           pow(1 - swing_phase_signal, n - i) *
                           control_points_y_[i];
            }
            /* 若想改成余弦抬脚，可替换为:
             *  x = L*(s_swing-0.5);
             *  y = H * (1 - cos(π*s_swing)) / 2;
             */
        }

        /* ───────── C. 将局部轨迹投影到机器人坐标系 ───────── */
        foot_position.X() += x * cosf(rotation); // 旋转后投影 X | rotate & project
        foot_position.Y() += x * sinf(rotation); // 投影到 Y
        foot_position.Z() += y;                  // 垂直方向直接加 y

        /* ───────── D. Saw-tooth 周期切换的数值保护 ───────── */
        if((swing_phase_signal == 0.0f && stance_phase_signal == 0.0f)
           && step_length > 0.0f)
        {
            foot_position = prev_foot_position_; // 回滚 | reset to last pose
        }

        // 记录上一帧位置 | store for next frame
        prev_foot_position_ = foot_position;
    }
};
} // namespace champ

#endif // TRAJECTORY_PLANNER_H
