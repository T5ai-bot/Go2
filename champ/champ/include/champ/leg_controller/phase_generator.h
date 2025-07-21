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

/********************************************************************************
* Copyright (c) 2019-2020, Juan Miguel Jimeno
* BSD-3-Clause license, see original header.
********************************************************************************/

#ifndef PHASE_GENERATOR_H
#define PHASE_GENERATOR_H

#include <macros/macros.h>                 // 提供 time_us() | generic μs timer
#include <quadruped_base/quadruped_base.h> // gait_config, leg geometry, etc.

namespace champ
{

class PhaseGenerator
{
public:
    using Time = unsigned long int;              // µs 时间戳 | micro-second stamp
    static inline Time now() { return time_us(); } // 获取当前 µs | wall-clock

private:
    champ::QuadrupedBase *base_;     // 机器人模型 & 参数 | robot description
    Time  last_touchdown_;           // 上次“落脚”基准 | start of current stride 上次起步时刻
    bool  has_swung_;                // 首拍标志 | first-swing flag

public:
    // ───────── 构造函数 | ctor ─────────
    PhaseGenerator(champ::QuadrupedBase &base,
                   Time time = now())
        : base_(&base),
          last_touchdown_(time),
          has_swung_(false),
          has_started(false),
          stance_phase_signal{0,0,0,0},
          swing_phase_signal{0,0,0,0}
    {}

    /* ───────────────────────────────────────────────
     * run() = “滴答”一次，更新 4 条腿相位信号
     *  - target_velocity = 0 时立即停相位
     *  - step_length 暂未使用，预留步频自适应
     * ─────────────────────────────────────────────── */
    void run(float target_velocity,
             float /*step_length*/,
             Time  time = now())
    {
        // ───── 1. 步态周期参数 | gait timing ─────
        unsigned long elapsed_time_ref = 0;               // 全局 saw-tooth
        const float swing_phase_period  = 0.25f * SECONDS_TO_MICROS;     // 固定 0.25 s | swing
        const float stance_phase_period = base_->gait_config.stance_duration
                                          * SECONDS_TO_MICROS;           // 从参数表
        const float stride_period = stance_phase_period + swing_phase_period;

        float leg_clocks[4] = {0,0,0,0}; // 四条腿局部时钟 | per-leg clock

        // ───── 2. 若速度为 0 → 立即停走 ─────
        if(target_velocity == 0.0f)
        {
            elapsed_time_ref = 0;
            last_touchdown_  = 0;
            has_swung_       = false;

            for(int i = 0; i < 4; ++i)
            {
                leg_clocks[i]         = 0;
                stance_phase_signal[i] = 0;
                swing_phase_signal[i]  = 0;
            }
            return; // 直接退出
        }

        // ───── 3. 第一次收到指令 → 启动全局时钟 ─────
        if(!has_started)
        {
            has_started   = true;
            last_touchdown_ = time; // 把现在当作 new stride 的起点
        }

        // ───── 4. 跨越一个 stride → 重置 saw-tooth ─────
        if((time - last_touchdown_) >= stride_period)
            last_touchdown_ = time;

        // 计算当前 saw-tooth 值 (0..stride] | elapsed time within stride
        elapsed_time_ref = time - last_touchdown_;
        if(elapsed_time_ref >= stride_period) elapsed_time_ref = stride_period;

        // ───── 5. 为 trot 设置 4 条腿局部时钟 ─────
        /*  拓扑:  LF & RH 同步, RF & LH 同步, 相差半周期  */
        leg_clocks[0] = elapsed_time_ref - 0.0f  * stride_period; // LF
        leg_clocks[1] = elapsed_time_ref - 0.5f  * stride_period; // RF
        leg_clocks[2] = elapsed_time_ref - 0.5f  * stride_period; // LH
        leg_clocks[3] = elapsed_time_ref - 0.0f  * stride_period; // RH
        // 若要 Walk/Pace，需要改这四行

        // ───── 6. 映射到 0-1 相位 ─────
        for(int i = 0; i < 4; ++i)
        {
            /* 支撑期 (0, stance] → 0-1 */
            if(leg_clocks[i] > 0 && leg_clocks[i] < stance_phase_period)
                stance_phase_signal[i] = leg_clocks[i] / stance_phase_period;
            else
                stance_phase_signal[i] = 0;

            /* 摆动期分两段，同样映射到 0-1 */
            if(leg_clocks[i] > -swing_phase_period && leg_clocks[i] < 0)
                swing_phase_signal[i] = (leg_clocks[i] + swing_phase_period)
                                         / swing_phase_period; // 上一周期尾巴
            else if(leg_clocks[i] > stance_phase_period && leg_clocks[i] < stride_period)
                swing_phase_signal[i] = (leg_clocks[i] - stance_phase_period)
                                         / swing_phase_period; // 本周期头部
            else
                swing_phase_signal[i] = 0;
        }

        // ───── 7. 首拍保护：先抬对角腿 ─────
        if(!has_swung_ && stance_phase_signal[0] < 0.5f)
        {
            /* 让 LF & RH 保持支撑, RF & LH 进入摆动 */
            stance_phase_signal[0] = 0;
            stance_phase_signal[3] = 0;
            swing_phase_signal[1]  = 0;
            swing_phase_signal[2]  = 0;
        }
        else
        {
            has_swung_ = true; // 以后完全由相位时钟驱动
        }
    }

    // ───── 公共状态量 | public fields ─────
    bool  has_started;            // 是否已启动 gait
    float stance_phase_signal[4]; // 支撑相位 0-1
    float swing_phase_signal[4];  // 摆动相位 0-1
};

} // namespace champ

#endif // PHASE_GENERATOR_H

