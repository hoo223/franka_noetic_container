#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_matrix
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import CameraInfo
from termcolor import colored

class ObjectKalmanFilter:
    def __init__(self, process_noise=1e-4, measurement_noise=5e-3):
        self.state = None  # [x, y, z, qx, qy, qz, qw]
        self.P = np.eye(7) * 0.1
        self.Q = np.eye(7) * process_noise
        self.R = np.eye(7) * measurement_noise

    def update(self, measurement):
        if self.state is None:
            self.state = measurement
            return self.state
        P_prior = self.P + self.Q
        K = P_prior @ np.linalg.inv(P_prior + self.R)
        self.state = self.state + K @ (measurement - self.state)
        self.P = (np.eye(7) - K) @ P_prior
        self.state[3:] /= np.linalg.norm(self.state[3:])
        return self.state

class ObjectWorldMemoryNode:
    def __init__(self, targets):
        rospy.init_node("object_world_memory_node")
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        
        self.base_frame = "panda_link0"
        self.cam_optical_frame = "camera_color_optical_frame"
        
        self.iou_threshold = 0.7
        self.cam_stable_threshold = 0.002 
        
        self.targets = targets
        self.filters = {name: ObjectKalmanFilter() for name in self.targets}
        self.prev_cam_pos = None
        self.latest_cam_K = None
        self.last_iou = {name: 0.0 for name in self.targets}

        # 발행자: SAM2 가이드용 u, v 좌표
        self.prompt_pubs = {name: rospy.Publisher(f"/sam2_prompt/{name}", PointStamped, queue_size=1) 
                           for name in self.targets}
        
        # 구독자
        for name in self.targets:
            rospy.Subscriber(f"/object_iou/{name}", Float32, self.iou_callback, callback_args=name)
            rospy.Subscriber(f"/object_pose/{name}", PoseStamped, self.pose_callback, callback_args=name)
        
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.cam_info_callback)

        print(colored(f"[*] Memory Node with Projection Started: {self.targets}", "green", attrs=['bold']))

    def cam_info_callback(self, msg):
        self.latest_cam_K = np.array(msg.K).reshape(3, 3)

    def iou_callback(self, msg, obj_name):
        self.last_iou[obj_name] = msg.data

    def is_camera_moving(self):
        try:
            (trans, _) = self.listener.lookupTransform(self.base_frame, self.cam_optical_frame, rospy.Time(0))
            curr_pos = np.array(trans)
            if self.prev_cam_pos is None:
                self.prev_cam_pos = curr_pos
                return True
            dist = np.linalg.norm(curr_pos - self.prev_cam_pos)
            self.prev_cam_pos = curr_pos
            return dist > self.cam_stable_threshold
        except: return True

    def pose_callback(self, msg, obj_name):
        if self.last_iou[obj_name] < self.iou_threshold or self.is_camera_moving():
            return
        try:
            (trans_bc, quat_bc) = self.listener.lookupTransform(self.base_frame, self.cam_optical_frame, rospy.Time(0))
            T_B_C = quaternion_matrix(quat_bc); T_B_C[:3, 3] = trans_bc
            
            p_co = msg.pose.position
            q_co = msg.pose.orientation
            T_C_O = quaternion_matrix([q_co.x, q_co.y, q_co.z, q_co.w])
            T_C_O[:3, 3] = [p_co.x, p_co.y, p_co.z]

            T_B_O = T_B_C @ T_C_O
            measurement = np.concatenate([T_B_O[:3, 3], quaternion_from_matrix(T_B_O)])
            self.filters[obj_name].update(measurement)
        except: pass

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.latest_cam_K is None:
                rate.sleep(); continue

            for name in self.targets:
                kf = self.filters[name]
                if kf.state is None: continue

                # 1. 칼만 필터 상태 (Base 기준 물체 포즈 T_B_O)
                s = kf.state
                T_B_O = quaternion_matrix([s[3], s[4], s[5], s[6]])
                T_B_O[:3, 3] = [s[0], s[1], s[2]]

                # 2. 현재 카메라 위치 조회 및 상대 포즈 계산 (T_C_O = T_C_B * T_B_O)
                try:
                    (t_bc, q_bc) = self.listener.lookupTransform(self.base_frame, self.cam_optical_frame, rospy.Time(0))
                    T_B_C = quaternion_matrix(q_bc); T_B_C[:3, 3] = t_bc
                    T_C_O = np.linalg.inv(T_B_C) @ T_B_O
                    
                    x_c, y_c, z_c = T_C_O[:3, 3]

                    # 3. 카메라 평면 투영 (u, v)
                    if z_c > 0.1: # 카메라 전방에 있을 때만
                        u = (self.latest_cam_K[0, 0] * x_c / z_c) + self.latest_cam_K[0, 2]
                        v = (self.latest_cam_K[1, 1] * y_c / z_c) + self.latest_cam_K[1, 2]

                        # 4. PointStamped 발행
                        prompt_msg = PointStamped()
                        prompt_msg.header.stamp = rospy.Time.now()
                        prompt_msg.header.frame_id = self.cam_optical_frame
                        prompt_msg.point.x, prompt_msg.point.y, prompt_msg.point.z = u, v, z_c
                        self.prompt_pubs[name].publish(prompt_msg)

                    # 5. TF 발행 (상시)
                    self.broadcaster.sendTransform((s[0], s[1], s[2]), (s[3], s[4], s[5], s[6]), 
                                                    rospy.Time.now(), f"memory_{name}", self.base_frame)
                except: continue
            rate.sleep()

if __name__ == "__main__":
    import sys
    target_args = sys.argv[1] if len(sys.argv) > 1 else "part1"
    target_list = [t.strip() for t in target_args.split(",")]
    node = ObjectWorldMemoryNode(targets=target_list)
    node.run()