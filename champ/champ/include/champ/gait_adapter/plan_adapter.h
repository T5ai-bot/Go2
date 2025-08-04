#pragma once
#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <champ/motion_matching/Plan.h>
#include <champ/motion_matching/MMOutput.h>

namespace champ::gait_adapter {

inline champ::motion_matching::MMOutput
toMMOutput(const champ::motion_matching::Plan &p,
           float stance_duration,   // 机器人 gait 参数
           float /*hip_radius*/)    // 预留
{
    using champ::motion_matching::MMOutput;
    MMOutput out;

    /* ① 相位拷贝 */
    for(int i=0;i<4;++i){
        out.stance_phase[i] = p.stance[i];
        out.swing_phase [i] = p.swing [i];
    }

    /* ② 步长/偏航估算（用 v_ref[0]） */
    if(!p.v_ref.empty()){
        float v_xy  = p.v_ref.front().head<2>().norm();
        float stride = v_xy * stance_duration * 2.f;
        out.step_len.fill(stride);

        float yaw = std::atan2(p.v_ref.front().y(), p.v_ref.front().x());
        out.yaw_rate = yaw / stance_duration / 2.f;
    }else{
        out.step_len.fill(0.f);
        out.yaw_rate = 0.f;
    }

    /* ③ CoM 高度 */
    out.root_height = p.h_ref.empty() ? 0.f : p.h_ref.front();
    return out;
}

}  // namespace champ::gait_adapter
