#pragma once
#include <array>
#include <rclcpp/time.hpp>

namespace champ::motion_matching {
struct MMOutput {
    rclcpp::Time stamp;                       // 时间戳
    std::array<float,4> stance_phase;         // 当前支撑相位
    std::array<float,4> swing_phase;          // 当前摆动相位
    std::array<float,4> step_len;             // 单腿步长
    float root_height = 0.0f;                 // CoM 高度偏移
    float yaw_rate    = 0.0f;                 // 偏航角速度
    uint8_t gait_type = 0;                    // Walk/Trot/Pace…
};
}  // namespace champ::motion_matching
