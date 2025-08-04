#!/usr/bin/env python3
import ast, json, random, numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from champ.motion_matching import MotionMatchingModule, MMOutput, Plan

class TxtDB:
    def __init__(self, folder):                         # 读取所有 .txt
        self.files = list(Path(folder).glob("*.txt"))
    def random_plan(self) -> Plan:                      # 随机挑一段
        data = {}
        for line in random.choice(self.files).read_text().splitlines():
            if ':' in line:
                k,v = line.split(':',1); data[k.strip()] = ast.literal_eval(v.strip())
        p = Plan(); p.swing=data['swing']; p.stance=data['stance']
        p.v_ref=[np.array(v) for v in data['v_ref']]; p.h_ref=data['h_ref']; return p

def to_mm(plan) -> MMOutput:                            # 极简适配
    mm = MMOutput()
    mm.stance_phase = plan.stance
    mm.swing_phase  = plan.swing
    vxy = np.linalg.norm(plan.v_ref[0][:2])
    stride = vxy*0.30*2
    mm.step_len = [stride]*4
    mm.yaw_rate = 0.0
    return mm

class Bridge(Node):
    def __init__(self):
        super().__init__('mm_bridge')
        db = self.declare_parameter('db_path').get_parameter_value().string_value
        self.db = TxtDB(db); self.timer = self.create_timer(0.01,self.tick)
    def tick(self):
        plan = self.db.random_plan()
        mm   = to_mm(plan)
        MotionMatchingModule.instance().setLatestOutput(mm)   # 写入 C++
def main():
    rclpy.init(); rclpy.spin(Bridge()); rclpy.shutdown()
if __name__ == '__main__': main()
