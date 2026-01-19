#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import json
import os
import tf.transformations as tft # 회전 행렬 <-> 쿼터니언 변환용
from geometry_msgs.msg import TransformStamped

class TFAverager:
    def __init__(self):
        rospy.init_node('tf_average_node')
        
        # 파라미터 설정
        self.parent_frame = "world"
        self.object_name = "part11"
        self.object_frame = f"object_{self.object_name}"
        self.save_path = f"/root/share/catkin_ws/src/calibration_setup/scripts/{self.parent_frame}_to_{self.object_name}_transform.json"
        self.sample_count = 100
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.avg_transform = None

        # Grasping TF 이름 정의
        self.grasp_frame = f"grasp_{self.object_name}"

    def get_current_tf(self):
        """실시간 TF를 100개 수집하여 평균과 분산 계산"""
        translations = []
        quaternions = []
        
        rospy.loginfo(f"Sampling {self.sample_count} transforms for {self.object_name}...")

        rate = rospy.Rate(20)  # 20Hz로 수집
        while len(translations) < self.sample_count and not rospy.is_shutdown():
            try:
                # world -> object_name 변환 조회
                trans = self.tf_buffer.lookup_transform('world', self.object_frame, rospy.Time(0), rospy.Duration(1.0))

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


    def broadcast_transform(self):
        if self.avg_transform is None:
            return

        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = self.object_frame

        t = self.avg_transform['translation']
        r = self.avg_transform['rotation']
        
        static_transformStamped.transform.translation.x = t[0]
        static_transformStamped.transform.translation.y = t[1]
        static_transformStamped.transform.translation.z = t[2]
        static_transformStamped.transform.rotation.x = r[0]
        static_transformStamped.transform.rotation.y = r[1]
        static_transformStamped.transform.rotation.z = r[2]
        static_transformStamped.transform.rotation.w = r[3]

        self.static_broadcaster.sendTransform(static_transformStamped)
        rospy.loginfo(f"Broadcasting fixed transform for {self.object_name}_fixed")

    def broadcast_loop(self):
        """
        1. world -> object_part11 (평균된 고정 위치)
        2. object_part11 -> grasp_part11 (물체 기준 상대 Offset 위치)
        """
        rate = rospy.Rate(10)
        rospy.loginfo(f"Broadcasting TFs: world -> {self.object_frame} -> {self.grasp_frame}")
        
        # 설정하고자 하는 Offset (object_part11 좌표계 기준)
        # 예: 물체 중심에서 물체의 로컬 Z축 방향으로 5cm 위에 손가락이 위치해야 한다면
        offset_x = 0.0
        offset_y = 0.02
        offset_z = 0.01

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            
            # --- 1. World -> Object TF ---
            obj_ts = TransformStamped()
            obj_ts.header.stamp = now
            obj_ts.header.frame_id = "world"
            obj_ts.child_frame_id = self.object_frame
            
            t = self.avg_transform['translation']
            r = self.avg_transform['rotation']
            
            obj_ts.transform.translation.x, obj_ts.transform.translation.y, obj_ts.transform.translation.z = t
            obj_ts.transform.rotation.x, obj_ts.transform.rotation.y, obj_ts.transform.rotation.z, obj_ts.transform.rotation.w = r
            obj_ts.transform.translation.z += 0.1

            # --- 2. Object -> Grasp TF (상대 좌표계) ---
            grasp_ts = TransformStamped()
            grasp_ts.header.stamp = now
            grasp_ts.header.frame_id = self.object_frame  # 부모가 object_part11이 됨
            grasp_ts.child_frame_id = self.grasp_frame
            
            # Translation: object_part11 좌표계 기준의 offset
            grasp_ts.transform.translation.x = offset_x
            grasp_ts.transform.translation.y = offset_y
            grasp_ts.transform.translation.z = offset_z
            
            # Rotation: 제공해주신 align_orientation_to_peg 로직을 사용해 계산
            # object_part11 기준에서의 상대적 회전량을 계산하여 입력
            grasp_quat_local = self.calculate_relative_grasp_orientation(r)
            
            grasp_ts.transform.rotation.x = grasp_quat_local[0]
            grasp_ts.transform.rotation.y = grasp_quat_local[1]
            grasp_ts.transform.rotation.z = grasp_quat_local[2]
            grasp_ts.transform.rotation.w = grasp_quat_local[3]

            # 3. Grasp -> Pre-Grasp (파지 위치에서 10cm 뒤로 후퇴한 위치)
            pre_ts = TransformStamped()
            pre_ts.header.stamp = now
            pre_ts.header.frame_id = self.grasp_frame  # 부모가 Grasp 프레임
            pre_ts.child_frame_id = f"pre_{self.grasp_frame}"
            
            # Grasp 좌표계 기준 Z축(접근 축)으로 -10cm 후퇴
            # Franka 그리퍼 기준, 보통 Z축이 접근 방향입니다.
            pre_ts.transform.translation.x = 0.0
            pre_ts.transform.translation.y = 0.0
            pre_ts.transform.translation.z = -0.05  # 5cm 뒤
            pre_ts.transform.rotation.w = 1.0  # 회전은 Grasp 프레임과 동일하게

            # 리스트로 묶어서 발행
            self.static_broadcaster.sendTransform([obj_ts, grasp_ts, pre_ts])
            rate.sleep()

    def calculate_relative_grasp_orientation(self, object_quat_world):
        """
        물체 좌표계(object_part11) 내에서 
        제공된 peg 정렬 로직을 만족하는 상대적 쿼터니언 계산
        """
        # 1. 물체의 월드 기준 회전 행렬
        R_obj_world = tft.quaternion_matrix(object_quat_world)
        
        # 2. 사용자 로직에 따른 '목표(Grasp)의 월드 기준 회전' 계산
        x_axis_peg = R_obj_world[:3, 0]
        y_axis_peg = R_obj_world[:3, 1]
        
        x_target_w = -y_axis_peg
        y_target_w = -x_axis_peg
        z_target_w = np.cross(x_target_w, y_target_w)
        
        R_grasp_world = np.eye(4)
        R_grasp_world[:3, 0] = x_target_w / np.linalg.norm(x_target_w)
        R_grasp_world[:3, 1] = y_target_w / np.linalg.norm(y_target_w)
        R_grasp_world[:3, 2] = z_target_w / np.linalg.norm(z_target_w)
        
        # 3. 상대 회전 계산: R_relative = R_obj_world.inv() * R_grasp_world
        R_relative = np.dot(tft.inverse_matrix(R_obj_world), R_grasp_world)
        
        return tft.quaternion_from_matrix(R_relative)

    def check_tf_once(self):
        """현재 TF가 발행 중인지 딱 한 번 확인"""
        try:
            # 0.5초만 기다려보고 없으면 바로 Exception
            self.tf_buffer.lookup_transform('world', self.object_frame, rospy.Time(0), rospy.Duration(0.5))
            return True
        except:
            return False

    def publish_static_transforms(self):
        """
        while 루프 없이 '딱 한 번'만 모든 정적 TF를 발행합니다.
        StaticBroadcaster가 데이터를 마스터에 저장하므로 이후 루프가 필요 없습니다.
        """
        if self.avg_transform is None:
            return

        now = rospy.Time.now()
        transforms = []

        # Offset 설정
        offset_x, offset_y, offset_z = 0.0, 0.02, 0.01

        # 1. World -> Object TF
        obj_ts = TransformStamped()
        obj_ts.header.stamp = now
        obj_ts.header.frame_id = "world"
        obj_ts.child_frame_id = self.object_frame
        t, r = self.avg_transform['translation'], self.avg_transform['rotation']
        obj_ts.transform.translation.x, obj_ts.transform.translation.y, obj_ts.transform.translation.z = t
        obj_ts.transform.rotation.x, obj_ts.transform.rotation.y, obj_ts.transform.rotation.z, obj_ts.transform.rotation.w = r
        obj_ts.transform.translation.z += 0.1 # 사용자 요청 오프셋
        transforms.append(obj_ts)

        # 2. Object -> Grasp TF
        grasp_ts = TransformStamped()
        grasp_ts.header.stamp = now
        grasp_ts.header.frame_id = self.object_frame
        grasp_ts.child_frame_id = self.grasp_frame
        grasp_ts.transform.translation.x, grasp_ts.transform.translation.y, grasp_ts.transform.translation.z = offset_x, offset_y, offset_z
        grasp_quat_local = self.calculate_relative_grasp_orientation(r)
        grasp_ts.transform.rotation.x, grasp_ts.transform.rotation.y, grasp_ts.transform.rotation.z, grasp_ts.transform.rotation.w = grasp_quat_local
        transforms.append(grasp_ts)

        # 3. Grasp -> Pre-Grasp TF
        pre_ts = TransformStamped()
        pre_ts.header.stamp = now
        pre_ts.header.frame_id = self.grasp_frame
        pre_ts.child_frame_id = f"pre_{self.grasp_frame}"
        pre_ts.transform.translation.z = -0.05
        pre_ts.transform.rotation.w = 1.0
        transforms.append(pre_ts)

        # 단 한 번 발행
        self.static_broadcaster.sendTransform(transforms)
        rospy.loginfo("All Static TFs (World-Obj-Grasp-Pre) published once.")

    def run(self):
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
            self.publish_static_transforms()
            rospy.spin()

if __name__ == '__main__':
    averager = TFAverager()
    averager.run()