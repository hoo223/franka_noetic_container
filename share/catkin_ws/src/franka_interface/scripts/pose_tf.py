#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import os
import json

class PoseTransformer:
    def __init__(self):
        rospy.init_node('pose_transformer_node')
        self.path = os.path.dirname(__file__)
        self.fixed_pose_root = os.path.join(self.path, 'fixed_pose')

        self.PRE_GRASP_OFFSET_Z = 0.07  # 7cm 위
        
        self.br = tf2_ros.TransformBroadcaster()
        rospy.loginfo("Pose Transformer Node with JSON pose files started")

    def load_pose(self, file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)
        pos = data['position']
        ori = data['orientation']
        return [pos['x'], pos['y'], pos['z']], [ori['x'], ori['y'], ori['z'], ori['w']]

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
            selected_peg = str(rospy.get_param('/selected_peg', 'part11'))
            object_frame = f"memory_{selected_peg}"

            if selected_peg == 'part1':
                rate.sleep()
                continue

            grasp_path = os.path.join(self.fixed_pose_root, selected_peg, 'grasp_tcp.json')
            if not os.path.exists(grasp_path):
                rospy.logwarn_throttle(2.0, f"Missing grasp_tcp.json for {selected_peg}: {grasp_path}")
                rate.sleep()
                continue

            try:
                grasp_pos, grasp_quat = self.load_pose(grasp_path)
                self.send_tf(object_frame, f"grasp_tcp_{selected_peg}", grasp_pos, grasp_quat)

                pre_grasp_path = os.path.join(self.fixed_pose_root, selected_peg, 'pre_grasp_tcp.json')
                if os.path.exists(pre_grasp_path):
                    pre_pos, pre_quat = self.load_pose(pre_grasp_path)
                else:
                    pre_pos = [grasp_pos[0], grasp_pos[1], grasp_pos[2] + self.PRE_GRASP_OFFSET_Z]
                    pre_quat = grasp_quat
                self.send_tf(object_frame, f"pre_grasp_tcp_{selected_peg}", pre_pos, pre_quat)

                insert_center_path = os.path.join(self.fixed_pose_root, selected_peg, 'insert_center.json')
                if not os.path.exists(insert_center_path) and selected_peg == 'part11-2':
                    insert_center_path = os.path.join(self.fixed_pose_root, 'part11', 'insert_center.json')

                if os.path.exists(insert_center_path):
                    insert_pos, insert_quat = self.load_pose(insert_center_path)
                    self.send_tf(object_frame, f"insert_center_{selected_peg}", insert_pos, insert_quat)
            except Exception as e:
                rospy.logwarn_throttle(2.0, f"Failed to publish JSON-based pose TF for {selected_peg}: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        node = PoseTransformer()
        node.run()
    except rospy.ROSInterruptException:
        pass
