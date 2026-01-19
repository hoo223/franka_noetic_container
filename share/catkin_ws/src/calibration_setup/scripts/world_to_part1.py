#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import json
import os
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix

class TFAverager:
    def __init__(self):
        rospy.init_node('tf_average_node')
        
        # # 파라미터 설정
        # self.parent_frame = "world"
        # self.object_name = "part1"
        # self.object_frame = f"object_{self.object_name}"
        # self.save_path = f"/root/share/catkin_ws/src/calibration_setup/scripts/{self.parent_frame}_to_{self.object_name}_transform.json"
        # self.sample_count = 100
        
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # self.avg_transform = None

        # 파라미터 설정
        self.parent_frame = "world"
        self.object_name = "part1"
        self.object_frame = f"object_{self.object_name}"
        self.goal_frame = "goal_pose"
        
        # 경로 설정
        self.save_path = f"/root/share/catkin_ws/src/calibration_setup/scripts/{self.parent_frame}_to_{self.object_name}_transform.json"
        self.goal_pose_path = "/root/share/catkin_ws/src/demo_traj/data/refined_final_peg_pose/test4_20251030_151337_part11_refined_final_pose.txt"
        
        self.sample_count = 100
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.avg_transform = None
        self.goal_transform = None # goal_pose를 위한 변환 정보 저장

    def load_goal_matrix(self):
        """txt 파일에서 4x4 행렬을 읽어와서 TF 포맷으로 변환"""
        if not os.path.exists(self.goal_pose_path):
            rospy.logerr(f"Goal pose file not found at {self.goal_pose_path}")
            return False

        try:
            # 텍스트 파일에서 4x4 행렬 로드 (공백 또는 탭 구분)
            matrix = np.loadtxt(self.goal_pose_path)
            if matrix.shape != (4, 4):
                rospy.logerr("Matrix shape is not 4x4")
                return False

            # Translation 추출
            trans = matrix[:3, 3]
            
            # Rotation Matrix를 Quaternion으로 변환
            # quaternion_from_matrix는 4x4를 입력받음
            q = quaternion_from_matrix(matrix)

            self.goal_transform = {
                'translation': trans.tolist(),
                'rotation': q.tolist()
            }
            rospy.loginfo("Successfully loaded goal matrix from txt file.")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to load goal matrix: {e}")
            return False

    def get_current_tf(self):
        """실시간 TF를 100개 수집하여 평균과 분산 계산"""
        translations = []
        quaternions = []
        
        rospy.loginfo(f"Sampling {self.sample_count} transforms for {self.object_name}...")

        rate = rospy.Rate(20)  # 20Hz로 수집
        while len(translations) < self.sample_count and not rospy.is_shutdown():
            try:
                # world -> object_name 변환 조회
                trans = self.tf_buffer.lookup_transform(self.parent_frame, self.object_frame, rospy.Time(0), rospy.Duration(1.0))

                t = trans.transform.translation
                r = trans.transform.rotation
                translations.append([t.x, t.y, t.z])
                quaternions.append([r.x, r.y, r.z, r.w])
            except Exception as e:  # (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print(f"TF lookup failed: {e}")
                continue
            rate.sleep()

        if len(translations) == self.sample_count:
            # 평균 및 분산 계산
            avg_t = np.mean(translations, axis=0)
            std_t = np.std(translations, axis=0)
            # 쿼터니언 평균 (단순 평균 후 정규화 - 근사치)
            avg_q = np.mean(quaternions, axis=0)
            avg_q /= np.linalg.norm(avg_q)
            
            self.avg_transform = {
                'translation': avg_t.tolist(),
                'rotation': avg_q.tolist(),
                'std_dev': std_t.tolist()
            }
            self.save_to_file()
            return True
        return False

    def save_to_file(self):
        with open(self.save_path, 'w') as f:
            json.dump(self.avg_transform, f)
        rospy.loginfo(f"Saved averaged transform to {self.save_path}")

    def load_from_file(self):
        if os.path.exists(self.save_path):
            with open(self.save_path, 'r') as f:
                self.avg_transform = json.load(f)
            rospy.loginfo(f"Loaded transform from file: {self.save_path}")
            return True
        return False

    def broadcast_loop(self):
        """world -> object 변환과 object -> goal 변환을 동시에 발행"""
        rate = rospy.Rate(10)
        rospy.loginfo("Starting to broadcast fixed transforms (Object and Goal)...")
        
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            transforms = []

            # 1. World -> Object (평균값 기반)
            tf_obj = TransformStamped()
            tf_obj.header.stamp = now
            tf_obj.header.frame_id = self.parent_frame
            tf_obj.child_frame_id = self.object_frame
            tf_obj.transform.translation.x, tf_obj.transform.translation.y, tf_obj.transform.translation.z = self.avg_transform['translation']
            tf_obj.transform.rotation.x, tf_obj.transform.rotation.y, tf_obj.transform.rotation.z, tf_obj.transform.rotation.w = self.avg_transform['rotation']
            transforms.append(tf_obj)

            # 2. Object -> Goal (txt 행렬 기반)
            if self.goal_transform:
                tf_goal = TransformStamped()
                tf_goal.header.stamp = now
                tf_goal.header.frame_id = self.object_frame # 부모가 object_part1
                tf_goal.child_frame_id = self.goal_frame
                tf_goal.transform.translation.x, tf_goal.transform.translation.y, tf_goal.transform.translation.z = self.goal_transform['translation']
                tf_goal.transform.rotation.x, tf_goal.transform.rotation.y, tf_goal.transform.rotation.z, tf_goal.transform.rotation.w = self.goal_transform['rotation']
                transforms.append(tf_goal)

            self.static_broadcaster.sendTransform(transforms)
            rate.sleep()

    def check_tf_once(self):
        """현재 TF가 발행 중인지 딱 한 번 확인"""
        try:
            # 0.5초만 기다려보고 없으면 바로 Exception
            self.tf_buffer.lookup_transform(self.parent_frame, self.object_frame, rospy.Time(0), rospy.Duration(0.5))
            return True
        except:
            return False
        
    def broadcast_static_once(self):
        """루프 없이 단 한 번만 Static TF를 발행"""
        now = rospy.Time.now()
        transforms = []

        # 1. World -> Object (평균값)
        tf_obj = TransformStamped()
        tf_obj.header.stamp = now
        tf_obj.header.frame_id = self.parent_frame
        tf_obj.child_frame_id = self.object_frame  # "grasp_part11" 등으로 일치 확인 필요
        tf_obj.transform.translation.x, tf_obj.transform.translation.y, tf_obj.transform.translation.z = self.avg_transform['translation']
        tf_obj.transform.rotation.x, tf_obj.transform.rotation.y, tf_obj.transform.rotation.z, tf_obj.transform.rotation.w = self.avg_transform['rotation']
        transforms.append(tf_obj)

        # 2. Object -> Goal (txt 행렬 기반)
        if self.goal_transform:
            tf_goal = TransformStamped()
            tf_goal.header.stamp = now
            tf_goal.header.frame_id = self.object_frame 
            tf_goal.child_frame_id = self.goal_frame   # "pre_grasp_part11" 등으로 일치 확인 필요
            tf_goal.transform.translation.x, tf_goal.transform.translation.y, tf_goal.transform.translation.z = self.goal_transform['translation']
            tf_goal.transform.rotation.x, tf_goal.transform.rotation.y, tf_goal.transform.rotation.z, tf_goal.transform.rotation.w = self.goal_transform['rotation']
            transforms.append(tf_goal)

        # 딱 한 번만 전송 (ROS가 이 정보를 래칭(Latching)하여 유지함)
        self.static_broadcaster.sendTransform(transforms)
        rospy.loginfo("Static transforms published once and fixed.")

    def run(self):
        # Goal Matrix (txt) 미리 읽기
        self.load_goal_matrix()

        # 1. 지금 즉시 TF가 나오고 있는가?
        if self.check_tf_once():
            rospy.loginfo("TF detected. Starting to sample 100 points...")
            self.get_current_tf() # 100개 수집 및 저장
        
        # 2. TF가 없다면 파일이 있는가?
        elif self.load_from_file():
            rospy.loginfo("TF not found, but loaded from file.")
        
        # 3. 둘 다 없다면 나올 때까지 대기
        else:
            rospy.logwarn("No TF and no file. Waiting for TF to be published...")
            while not rospy.is_shutdown():
                if self.check_tf_once():
                    self.get_current_tf()
                    break
                rospy.sleep(1.0)

        # 최종 확정된 값 발행
        if self.avg_transform:
            # self.broadcast_loop()
            self.broadcast_static_once() # 루프 대신 한 번만 실행
            rospy.spin() # 이후 노드가 죽지 않게 대기 (CPU 사용량 0%에 수렴)

if __name__ == '__main__':
    averager = TFAverager()
    averager.run()