#ifndef PHASE_GENERATOR_H
#define PHASE_GENERATOR_H

#include <macros/macros.h>
#include <quadruped_base/quadruped_base.h>

namespace champ
{
    class PhaseGenerator
    {
        public:
            typedef unsigned long int Time;
            static inline Time now() { return time_us(); }   // 获取当前时间 (µs)

        private:
            champ::QuadrupedBase *base_;   // 机器人参数 (stance_duration 等)

            Time  last_touchdown_;         // 上一次“落脚”基准时刻 (µs)
            bool  has_swung_;              // 首拍标志：启动时先抬对角腿

        public:
            PhaseGenerator(champ::QuadrupedBase &base, Time time = now()):
                base_(&base),
                last_touchdown_(time),      // 初始化基准时刻
                has_swung_(false),
                has_started(false),
                stance_phase_signal{0.0f,0.0f,0.0f,0.0f},
                swing_phase_signal{0.0f,0.0f,0.0f,0.0f}
            {}

            /* ───────────────────────────────────────────────
             * 主循环：根据时间 & 速度 → 4 条腿相位
             * target_velocity 为 0 时立即清零相位（防抖）
             * step_length 目前未用，可拓展“步长 ↔ 步频”自适应
             * ─────────────────────────────────────────────── */
            void run(float target_velocity, float /*step_length*/, Time time = now())
            {
                // ───── 1. 基本时间参数 ─────
                unsigned long elapsed_time_ref = 0;          // 全局“节拍器”计数
                float swing_phase_period  = 0.25f * SECONDS_TO_MICROS;        // 摆动期固定 0.25 s
                float stance_phase_period = base_->gait_config.stance_duration * SECONDS_TO_MICROS;
                float stride_period       = stance_phase_period + swing_phase_period;

                float leg_clocks[4] = {0.0f,0.0f,0.0f,0.0f}; // 四腿局部时钟

                // ───── 2. 若速度为 0 → 立即停相位 ─────
                if(target_velocity == 0.0f)
                {
                    elapsed_time_ref = 0;
                    last_touchdown_  = 0;
                    has_swung_       = false;

                    for(unsigned int i = 0; i < 4; i++)
                    {
                        leg_clocks[i]          = 0.0f;
                        stance_phase_signal[i] = 0.0f;
                        swing_phase_signal[i]  = 0.0f;
                    }
                    return;
                }

                // ───── 3. 第一次收到移动指令 → 启动时钟 ─────
                if(!has_started)
                {
                    has_started     = true;
                    last_touchdown_ = time;   // 把当前时刻当作“落脚基准”
                }

                // ───── 4. 若跨周期则“归零”时钟，保持 sawtooth ─────
                if((time - last_touchdown_) >= stride_period)
                {
                    last_touchdown_ = time;   // 新一轮支撑期起点
                }

                // 计算当前全局相位计数 (0‒stride]
                elapsed_time_ref = time - last_touchdown_;
                if(elapsed_time_ref >= stride_period)
                    elapsed_time_ref = stride_period; // 安全钳

                // ───── 5. 四腿局部时钟，决定步态拓扑 ─────
                /*   LF    RF
                 *     \  /
                 *     /  \
                 *   LH    RH
                 * pace
                 * 对角跑：LF/RH 相位同，RF/LH 相位同且滞后 0.5 周期
                 */
// leg_clocks[0] = elapsed_time_ref - (0.0f * stride_period);   // LF
// leg_clocks[1] = elapsed_time_ref - (0.5f * stride_period);   // RF
// leg_clocks[2] = elapsed_time_ref - (0.0f * stride_period);   // LH
// leg_clocks[3] = elapsed_time_ref - (0.5f * stride_period);   // RH
// trot
leg_clocks[0] = elapsed_time_ref - (0.0f * stride_period);   // LF
leg_clocks[1] = elapsed_time_ref - (0.5f * stride_period);   // RF
leg_clocks[2] = elapsed_time_ref - (0.5f * stride_period);   // LH
leg_clocks[3] = elapsed_time_ref - (0.0f * stride_period);   // RH

                // ───── 6. Saw-tooth → 归一化相位 (0‒1) ─────
                for(int i = 0; i < 4; i++)
                {
                    /* 支撑期：时钟 ∈ (0, stance] → 线性映射到 0‒1
                     * 摆动期分两段：
                     *   a) 时钟 ∈ (-swing, 0)          （上一周期末尾）
                     *   b) 时钟 ∈ (stance, stride)     （本周期摆动）
                     * 两段都映射到 0‒1
                     */
                    if(leg_clocks[i] > 0 && leg_clocks[i] < stance_phase_period)
                        stance_phase_signal[i] = leg_clocks[i] / stance_phase_period;
                    else
                        stance_phase_signal[i] = 0;

                    if(leg_clocks[i] > -swing_phase_period && leg_clocks[i] < 0)
                        swing_phase_signal[i] = (leg_clocks[i] + swing_phase_period) / swing_phase_period;
                    else if(leg_clocks[i] > stance_phase_period && leg_clocks[i] < stride_period)
                        swing_phase_signal[i] = (leg_clocks[i] - stance_phase_period) / swing_phase_period;
                    else
                        swing_phase_signal[i] = 0;
                }

                // ───── 7. 首拍保护：先抬对角腿防止蹲死 ─────
                if(!has_swung_ && stance_phase_signal[0] < 0.5f)
                {
                    // 进入运动第一半拍：强制 LF/RH 置 0（支撑），RF/LH 摆动
                    stance_phase_signal[0] = 0.0f;
                    stance_phase_signal[3] = 0.0f;
                    swing_phase_signal[1]  = 0.0f;
                    swing_phase_signal[2]  = 0.0f;
                }
                else
                {
                    has_swung_ = true;   // 以后正常跟随时钟
                }
            }

            // ───── 公共状态量 ─────
            bool  has_started;               // 已经开始运动
            float stance_phase_signal[4];    // 支撑相位 0‒1
            float swing_phase_signal[4];     // 摆动相位 0‒1
    };
}

#endif // PHASE_GENERATOR_H
