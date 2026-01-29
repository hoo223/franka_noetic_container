#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import tf.transformations as tr

TELEOP = 't'
TELEOP_IMPEDANCE = 'i'

class DualDeviceController:
    def __init__(self):
        rospy.init_node('dual_device_integrator')

        self.selected_peg = rospy.get_param("/selected_peg", "")

        # --- 프레임 설정 ---
        self.base_frame = "panda_link0"
        self.tcp_frame = "panda_hand_tcp"
        self.active_target_frame = rospy.get_param("/target_pose_frame", "insert_center_part11")
        print(f"Control Frame: {self.active_target_frame}")
        
        # 외부에서 들어오는 목표 포즈 저장용
        self.target_pos = None
        self.target_quat = None
        self.T_tcp_target = None  # TCP와 조종 프레임 간의 상대 변환 행렬 캐시
        
        self.current_mode = "UNKNOWN"
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # --- 구독자 (Subscribers) ---
        # 1. 제어 목표 포즈 (이 포즈를 active_target_frame이 추종함)
        rospy.Subscriber("/target_pose", PoseStamped, self.target_pose_callback)
        # 2. 현재 제어 모드
        rospy.Subscriber("/current_mode", String, self.mode_callback)
        # 3. 조종 프레임 변경 (예: "tool_link", "object_frame")
        rospy.Subscriber("/change_target_frame", String, self.target_frame_callback)

        # --- 발행자 (Publishers) ---
        self.pose_pub = rospy.Publisher("/cartesian_pose_controller/tcp_target_pose", PoseStamped, queue_size=1)
        self.imp_pub = rospy.Publisher("/cartesian_impedance_example_controller/equilibrium_pose", PoseStamped, queue_size=1)

        rospy.loginfo("Waiting for TF and Target Pose...")

    def mode_callback(self, msg):
        self.current_mode = msg.data

    def target_frame_callback(self, msg):
        self.active_target_frame = msg.data
        self.T_tcp_target = None # 프레임이 바뀌면 다시 계산하도록 초기화
        rospy.loginfo(f"Control frame switched to: {self.active_target_frame}")

    def target_pose_callback(self, msg):
        """외부에서 들어오는 /target_pose를 업데이트"""
        # 만약 msg의 frame_id가 base_frame과 다르다면 tf 변환이 필요할 수 있으나, 
        # 여기서는 panda_link0 기준으로 들어온다고 가정합니다.
        self.target_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.target_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, 
                                     msg.pose.orientation.z, msg.pose.orientation.w])

    def get_tcp_command(self):
        if self.target_pos is None:
            return None, None

        # 1. TCP와 조종 프레임 간의 상대 관계를 '딱 한 번만' 가져옴
        if self.T_tcp_target is None:
            try:
                # 0.5초 정도 여유를 두고 TF가 들어오길 기다림
                self.listener.waitForTransform(self.tcp_frame, self.active_target_frame, rospy.Time(0), rospy.Duration(0.5))
                (trans_rel, rot_rel) = self.listener.lookupTransform(self.tcp_frame, self.active_target_frame, rospy.Time(0))
                self.T_tcp_target = tr.concatenate_matrices(tr.translation_matrix(trans_rel), tr.quaternion_matrix(rot_rel))
                rospy.loginfo("TF Cache updated.")
            except Exception as e:
                rospy.logwarn(f"TF Lookup failed: {e}")
                return None, None
        
        # 2. 이후 루프에서는 계산된 행렬(T_tcp_target)만 재사용 (TF 조회 안 함)
        T_base_target = tr.concatenate_matrices(tr.translation_matrix(self.target_pos), tr.quaternion_matrix(self.target_quat))
        T_base_tcp = np.dot(T_base_target, tr.inverse_matrix(self.T_tcp_target))
        
        return tr.translation_from_matrix(T_base_tcp), tr.quaternion_from_matrix(T_base_tcp)

    def run(self):
        rate = rospy.Rate(15) # 제어 주기

        while not rospy.is_shutdown():
            if self.current_mode in [TELEOP, TELEOP_IMPEDANCE]:
                # 실제 로봇에게 보낼 TCP 목표 포즈 계산
                pub_pos, pub_quat = self.get_tcp_command()

                if pub_pos is not None:
                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = self.base_frame
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pub_pos
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = pub_quat

                    # 모드에 맞는 토픽으로 발행
                    if self.current_mode == TELEOP_IMPEDANCE:
                        self.imp_pub.publish(msg)
                    else:
                        self.pose_pub.publish(msg)

                    # # 시각화용 TF (현재 내가 어디를 조종하려고 하는지 표시)
                    # self.br.sendTransform(tuple(self.target_pos), tuple(self.target_quat), 
                    #                       rospy.Time.now(), "visual_target_marker", self.base_frame)
            
            rate.sleep()

if __name__ == '__main__':
    node = DualDeviceController()
    node.run()