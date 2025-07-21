#ifndef KINEMATICS_H            // 头文件保护 | header guard
#define KINEMATICS_H

#ifdef __unix__                 // 如果在类 Unix 系统 | on Unix-like OS
    #include <cmath>            // 数学库 | math functions
    using namespace std;        // 直接使用 std 命名空间 | bring std into scope
#endif

#include <macros/macros.h>      // 通用宏 | project-wide macros
#include <geometry/geometry.h>  // 位姿变换工具 | 3-D transformations
#include <quadruped_base/quadruped_base.h> // 机器人结构参数 | robot model

namespace champ               // CHAMP 命名空间 | namespace
{
    class Kinematics          // 运动学类 | kinematics helper
    {
        champ::QuadrupedBase *base_; // 机器人总参数指针 | ptr to robot base
        
        public:
            // 构造：存指针 | ctor: store pointer
            Kinematics(champ::QuadrupedBase &quadruped_base):
                base_(&quadruped_base)
            {
            }

            /*------------------------------------------------------
             * inverse() - 批量逆解四条腿
             * 输入: foot_positions[4]  (期望足端位姿)
             * 输出: joint_positions[12] (四腿×3 关节角)
             *-----------------------------------------------------*/
            void inverse(float (&joint_positions)[12], geometry::Transformation (&foot_positions)[4])
            {
                float calculated_joints[12];         // 临时数组 | temp buffer

                for(unsigned int i = 0; i < 4; i++)  // 遍历 4 条腿 | loop legs
                {
                    // 逐腿逆解 | per-leg IK
                    inverse(calculated_joints[(i*3)],
                            calculated_joints[(i*3) + 1],
                            calculated_joints[(i*3) + 2],
                            *base_->legs[i],
                            foot_positions[i]);
                    
                    // 检查 NaN —— 若任何一腿解算失败则整帧作废
                    // check NAN: if any leg invalid → discard
                    if(isnan(calculated_joints[(i*3)]) ||
                       isnan(calculated_joints[(i*3) + 1]) ||
                       isnan(calculated_joints[(i*3) + 2]))
                    {
                        return;                       // 直接返回 | abort
                    }
                }
                
                // 拷贝到输出 | copy to out-buffer
                for(unsigned int i = 0; i < 12; i++)
                {
                    joint_positions[i] = calculated_joints[i];
                }
            }

            /*------------------------------------------------------
             * inverse() - 单腿解析几何逆解
             *  @hip_joint/out  : 髋侧摆角 Hip-roll
             *  @upper_leg_joint: 大腿俯仰 Upper-leg pitch
             *  @lower_leg_joint: 小腿俯仰 Knee pitch
             *-----------------------------------------------------*/
            static void inverse(float &hip_joint, float &upper_leg_joint, float &lower_leg_joint, 
                                champ::QuadrupedLeg &leg, geometry::Transformation &foot_position)
            {
                geometry::Transformation temp_foot_pos = foot_position;        // 拷贝足端位姿 | copy pose

                float l0 = 0.0f;   // Hip-Y 偏置累加 | vertical hip offset

                for(unsigned int i = 1; i < 4; i++)   // 累加三段 y 长度 | sum link-y
                {
                    l0 += leg.joint_chain[i]->y();
                }

                // ---- 计算等效连杆长度 l1 / l2 及其安装偏角 α / β | compute link lens
                float l1 = -sqrtf(pow(leg.lower_leg.x(), 2) + pow(leg.lower_leg.z(), 2));
                float ik_alpha = acosf(leg.lower_leg.x() / l1) - (M_PI / 2); 

                float l2 = -sqrtf(pow(leg.foot.x(), 2) + pow(leg.foot.z(), 2));
                float ik_beta = acosf(leg.foot.x() / l2) - (M_PI / 2); 

                // ---- 提取足端坐标 | fetch XYZ
                float x = temp_foot_pos.X();
                float y = temp_foot_pos.Y();
                float z = temp_foot_pos.Z();
            
                // ---- Hip-roll 几何计算 | hip abduction angle
                hip_joint = -(atanf(y / z) -
                              ((M_PI/2) - acosf(-l0 / sqrtf(pow(y, 2) + pow(z, 2)))));

                // ---- 把坐标旋转到解算平面 | rotate into sagital plane
                temp_foot_pos.RotateX(-hip_joint);
                temp_foot_pos.Translate(-leg.upper_leg.x(), 0.0f, -leg.upper_leg.z());

                x = temp_foot_pos.X();
                y = temp_foot_pos.Y();
                z = temp_foot_pos.Z();

                // ---- 可达性检测 | reachability check
                float target_to_foot = sqrtf(pow(x, 2) + pow(z,2));
                if(target_to_foot >= (abs(l1) + abs(l2)))
                    return;                             // 超界直接退出 | unreachable

                // ---- 二连杆解析 IK | 2-link planar IK
                // 来源链接见注释 | source in code
                lower_leg_joint = leg.knee_direction() *
                    acosf((pow(z, 2) + pow(x, 2) - pow(l1 ,2) - pow(l2 ,2)) /
                          (2 * l1 * l2));

                upper_leg_joint = (atanf(x / z) -
                    atanf((l2 * sinf(lower_leg_joint)) /
                          (l1 + (l2 * cosf(lower_leg_joint)))));

                // ---- 加回安装偏角补偿 | add install offsets
                lower_leg_joint += ik_beta - ik_alpha;
                upper_leg_joint += ik_alpha;

                // ---- 角度翻折到合法区 | sane angle fix
                if(leg.knee_direction() < 0)           // 反向膝 | outward knee
                {
                    if(upper_leg_joint < 0)
                        upper_leg_joint += M_PI;
                }
                else                                   // 正向膝 | inward knee
                {
                    if(upper_leg_joint > 0)
                        upper_leg_joint += M_PI;
                }
            }

            /*----------------------------------------------
             * forward() 2-关节版本：给 θ1/θ2 求末端
             * 此处仅用于调试 | debug helper
             *---------------------------------------------*/
            static void forward(geometry::Transformation foot_position, const champ::QuadrupedLeg &leg, 
                                const float upper_leg_theta, 
                                const float lower_leg_theta )
            {
                foot_position.Translate(leg.foot.x(),  leg.foot.y(),  leg.foot.z()); // 末段
                foot_position.RotateY(lower_leg_theta);                              // 绕膝

                foot_position.Translate(leg.lower_leg.x(), leg.lower_leg.y(), leg.lower_leg.z());
                foot_position.RotateY(upper_leg_theta);                              // 绕髋pitch

                foot_position.Translate(leg.upper_leg.x(), leg.upper_leg.y(), leg.upper_leg.z());
            }

            /* forward() 3-关节版本：Hip+Upper+Lower */
            static void forward(geometry::Transformation foot_position, const champ::QuadrupedLeg &leg, 
                                const float hip_theta, 
                                const float upper_leg_theta, 
                                const float lower_leg_theta)
            {
                foot_position = Identity<4,4>();          // 单位矩阵 | reset

                foot_position.Translate(leg.foot.x(),  leg.foot.y(),  leg.foot.z());
                foot_position.RotateY(lower_leg_theta);  

                foot_position.Translate(leg.lower_leg.x(), leg.lower_leg.y(), leg.lower_leg.z());
                foot_position.RotateY(upper_leg_theta);   

                foot_position.Translate(leg.upper_leg.x(), leg.upper_leg.y(), leg.upper_leg.z());
                foot_position.RotateY(hip_theta);   

                foot_position.Translate(leg.hip.x(), leg.hip.y(), leg.hip.z());
            }

            /*---- frame utils: base ↔ hip ----*/
            static void transformToHip(geometry::Transformation &foot_position, const champ::QuadrupedLeg &leg)
            {
                foot_position.Translate(-leg.hip.x(), -leg.hip.y(), -leg.hip.z()); // 到髋坐标 | to hip frame
            }

            static void transformToBase(geometry::Transformation &foot_position, const champ::QuadrupedLeg &leg)
            {
                foot_position.Translate( leg.hip.x(),  leg.hip.y(),  leg.hip.z()); // 回底座 | to base frame
            }
    };
}

#endif  // KINEMATICS_H
