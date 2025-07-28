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

#ifndef BODY_CONTROLLER_H
#define BODY_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_leg.h>
#include <kinematics/kinematics.h>

namespace champ
{
    class BodyController
    {
        QuadrupedBase *base_;   // pointer to robot base   // 中文：机器人整体模型指针

        public:
            BodyController(QuadrupedBase &quadruped_base):
                base_(&quadruped_base)  // store the pointer  // 中文：保存指针以便后用
            {
            }

            // propagate pose command to all 4 legs             // 中文：把机体姿态指令分发到 4 条腿
            void poseCommand(geometry::Transformation (&foot_positions)[4],
                             const champ::Pose &req_pose)
            {
                for(int i = 0; i < 4; i++)
                {
                    poseCommand(foot_positions[i], *base_->legs[i], req_pose);
                }
            }

            // convert body pose to single-leg foot pose        // 中文：将机体姿态转换成单腿足端坐标
            static void poseCommand(geometry::Transformation &foot_position,
                                    champ::QuadrupedLeg &leg,
                                    const champ::Pose &req_pose)
            {
                float req_translation_x = -req_pose.position.x;   // negate body X   // 中文：机体前移→脚向后移

                float req_translation_y = -req_pose.position.y;   // negate body Y   // 中文：机体左移→脚向右移

                // desired Z; negative because feet sit below body     // 中文：机体上下补偿，脚在 Z 负方向
                float req_translation_z = -(leg.zero_stance().Z() + req_pose.position.z);
                float max_translation_z = -leg.zero_stance().Z() * 0.65; // limit 65 % leg length // 中文：最多下探 65%

                // there shouldn't be any negative translation when
                // the legs are already fully stretched
                // 中文：若腿已完全伸直，不允许再向上抬脚
                if(req_translation_z < 0.0)
                {
                    req_translation_z = 0.0;
                }
                else if(req_translation_z > max_translation_z)
                {
                    req_translation_z = max_translation_z;
                }

                // create a new foot position from position of legs when stretched out
                // 中文：从名义零落脚点开始计算
                foot_position = leg.zero_stance();

                // move the foot position to desired body position of the robot
                // 中文：根据机体平移量移动足端
                foot_position.Translate(req_translation_x, req_translation_y, req_translation_z);

                // rotate the leg opposite the required orientation of the body
                // 中文：按机体旋转的反方向旋转足端，实现抵消
                foot_position.RotateZ(-req_pose.orientation.yaw);
                foot_position.RotateY(-req_pose.orientation.pitch);
                foot_position.RotateX(-req_pose.orientation.roll);

                // convert to hip-frame coordinates for IK         // 中文：转换到髋坐标系供逆运动学使用
                champ::Kinematics::transformToHip(foot_position, leg);
            }
    };
}

#endif
