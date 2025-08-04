#pragma once
#include <array>
#include <vector>
#include <Eigen/Core>

namespace champ::motion_matching {
struct Plan {
    std::array<float,4> swing;          // φ_swing_i ∈ [0,1]
    std::array<float,4> stance;         // φ_stance_i
    std::vector<Eigen::Vector3f> v_ref; // 未来 Nh 帧 CoM 速度
    std::vector<float>          h_ref;  // 未来 Nh 帧 CoM 高度
};
}  // namespace champ::motion_matching
