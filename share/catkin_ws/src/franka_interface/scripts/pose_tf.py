#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np

class PoseTransformer:
    def __init__(self):
        rospy.init_node('pose_transformer_node')

        # 1. 설정 데이터 통합 관리
        self.PRE_GRASP_OFFSET_Z = 0.07  # 7cm 위
        
        # Grasp Pose (객체 좌표계 기준 TCP의 목표 포즈)
        self.grasp_pose_dict = {
            "part7":  {"pos": [-0.026, -0.037, 0.000], "quat": [1.000, -0.001, 0.021, 0.008]},
            "part9":  {"pos": [-0.003, -0.011, 0.015], "quat": [1.000, 0.007, 0.030, 0.003]},
            "part11": {"pos": [0.006, -0.002, 0.008],  "quat": [1.000, -0.004, 0.003, -0.010]},
            "part12": {"pos": [-0.005, -0.001, 0.017], "quat": [-0.704, 0.710, 0.010, -0.000]}
        }

        # Insert Center (객체 좌표계 기준 실제 구멍에 들어갈 중심점 포즈)
        self.insert_center_dict = {
            "part7":  {"pos": [0.0, 0.0, -0.02], "quat": [0, 0, 0, 1]},
            "part9":  {"pos": [0.0, 0.0, -0.02], "quat": [0, 0, 0, 1]},
            "part11": {"pos": [0.0, 0.0, -0.02], "quat": [0, 0, 0, 1]},
            "part12": {"pos": [0.0, 0.0, -0.02], "quat": [0, 0, 0, 1]}
        }
        
        self.br = tf2_ros.TransformBroadcaster()
        rospy.loginfo("Pose Transformer Node with Dict Management Started")

    def send_tf(self, parent, child, pos, quat):
        """TF 메시지 생성 및 발행"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.br.sendTransform(t)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            # 파라미터 서버에서 현재 작업 중인 물체 이름 획득
            selected_peg = rospy.get_param('/selected_peg', 'part11')
            object_frame = f"memory_{selected_peg}"
            
            # 해당 물체에 대한 데이터가 존재할 경우에만 TF 발행
            if selected_peg in self.grasp_pose_dict:
                g_data = self.grasp_pose_dict[selected_peg]
                i_data = self.insert_center_dict.get(selected_peg, {"pos": [0,0,0], "quat": [0,0,0,1]})

                # 1. Grasp TCP 발행
                self.send_tf(object_frame, f"grasp_tcp_{selected_peg}", g_data["pos"], g_data["quat"])

                # 2. Pre-Grasp TCP 발행 (Grasp TCP 대비 Z축 오프셋)
                pre_pos = [g_data["pos"][0], g_data["pos"][1], g_data["pos"][2] + self.PRE_GRASP_OFFSET_Z]
                self.send_tf(object_frame, f"pre_grasp_tcp_{selected_peg}", pre_pos, g_data["quat"])

                # 3. Insert Center 발행 (별도 dict 관리)
                self.send_tf(object_frame, f"insert_center_{selected_peg}", i_data["pos"], i_data["quat"])

            rate.sleep()

if __name__ == '__main__':
    try:
        node = PoseTransformer()
        node.run()
    except rospy.ROSInterruptException:
        pass