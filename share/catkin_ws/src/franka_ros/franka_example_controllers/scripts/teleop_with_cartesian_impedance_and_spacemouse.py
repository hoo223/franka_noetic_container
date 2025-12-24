#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import copy
import tf.transformations as tft
import numpy as np
import actionlib

# 메시지 타입 임포트
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from franka_msgs.msg import FrankaState  # Franka 상태 메시지
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

class SpaceNavTeleop:
    def __init__(self):
        rospy.init_node('spacenav_franka_teleop')

        # --- 설정 변수 ---
        self.trans_scale = 0.05  # m/s
        self.rot_scale = 0.5     # rad/s
        self.deadzone = 0.1
        self.publish_rate = 30.0
        
        # [중요] 링크 이름 설정 (기본값 fr3_link0, 파라미터로 변경 가능)
        self.base_frame = rospy.get_param("~link_name", "fr3_link0")

        # --- 초기 상태 변수 ---
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = self.base_frame
        
        # SpaceNav 입력값 저장용
        self.joy_axes = [0.0] * 6
        self.prev_buttons = [0, 0]

        # 1. 초기 위치 설정 (실제 로봇 상태에서 가져오기)
        rospy.loginfo("Waiting for initial robot pose from /franka_state_controller/franka_states...")
        self.wait_for_initial_pose()
        rospy.loginfo("Initial pose acquired. Ready to control.")

        # --- Publisher & Subscriber ---
        self.pose_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose', 
            PoseStamped, 
            queue_size=1
        )
        
        rospy.Subscriber('/spacenav/joy', Joy, self.joy_callback)

        # --- Gripper Action Clients ---
        self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        
        # 그리퍼 서버 연결 대기 (비동기로 하거나, 여기서 짧게 대기)
        # rospy.loginfo("Waiting for gripper action servers...")
        # self.move_client.wait_for_server(rospy.Duration(5.0))
        # self.grasp_client.wait_for_server(rospy.Duration(5.0))

    def wait_for_initial_pose(self):
        """
        /franka_state_controller/franka_states 토픽에서 
        현재 EE 위치(O_T_EE)를 한 번 받아와 target_pose를 초기화함
        """
        msg = rospy.wait_for_message("franka_state_controller/franka_states", FrankaState)
        
        # O_T_EE는 Row-major 1D 배열(16개)로 들어옵니다. 이를 4x4 행렬로 변환 후 전치(Transpose) 필요
        # (Franka 문서는 Column-major라고 명시하지만, numpy reshape 시 주의 필요)
        # 제공해주신 레퍼런스 코드 방식 그대로 적용:
        
        initial_quaternion = tft.quaternion_from_matrix(
            np.transpose(np.reshape(msg.O_T_EE, (4, 4)))
        )
        
        # 쿼터니언 정규화
        initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)

        # target_pose에 초기값 할당
        self.target_pose.pose.orientation.x = initial_quaternion[0]
        self.target_pose.pose.orientation.y = initial_quaternion[1]
        self.target_pose.pose.orientation.z = initial_quaternion[2]
        self.target_pose.pose.orientation.w = initial_quaternion[3]
        
        # 위치 정보 (O_T_EE 배열의 12, 13, 14 인덱스가 x, y, z translation)
        self.target_pose.pose.position.x = msg.O_T_EE[12]
        self.target_pose.pose.position.y = msg.O_T_EE[13]
        self.target_pose.pose.position.z = msg.O_T_EE[14]

    def joy_callback(self, msg):
        self.joy_axes = msg.axes
        
        current_left = msg.buttons[0]
        current_right = msg.buttons[1]

        # 왼쪽 버튼: 열기
        if current_left == 1 and self.prev_buttons[0] == 0:
            self.gripper_command("open")
        
        # 오른쪽 버튼: 닫기 (Force Mode)
        elif current_right == 1 and self.prev_buttons[1] == 0:
            self.gripper_command("close")

        self.prev_buttons = msg.buttons

    def gripper_command(self, cmd):
        if cmd == "open":
            goal = MoveGoal(width=0.08, speed=0.1)
            self.move_client.cancel_all_goals()
            self.grasp_client.cancel_all_goals()
            self.move_client.send_goal(goal)
            rospy.loginfo("Gripper Open")
        elif cmd == "close":
            goal = GraspGoal()
            goal.width = 0.0
            goal.epsilon.inner = 0.005
            goal.epsilon.outer = 0.1 # Blind Grasp
            goal.speed = 0.1
            goal.force = 60.0
            self.move_client.cancel_all_goals()
            self.grasp_client.cancel_all_goals()
            self.grasp_client.send_goal(goal)
            rospy.loginfo("Gripper Grasp")

    def apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        return val

    def update_pose(self, dt):
        # 1. 입력 처리
        vx = -self.apply_deadzone(self.joy_axes[0]) * self.trans_scale
        vy = -self.apply_deadzone(self.joy_axes[1]) * self.trans_scale
        vz = self.apply_deadzone(self.joy_axes[2]) * self.trans_scale
        
        wx = -self.apply_deadzone(self.joy_axes[3]) * self.rot_scale
        wy = self.apply_deadzone(self.joy_axes[4]) * self.rot_scale
        wz = -self.apply_deadzone(self.joy_axes[5]) * self.rot_scale

        if all(v == 0 for v in [vx, vy, vz, wx, wy, wz]):
            return

        # 2. 위치 업데이트
        self.target_pose.pose.position.x += vx * dt
        self.target_pose.pose.position.y += vy * dt
        self.target_pose.pose.position.z += vz * dt

        # 3. 회전 업데이트
        current_quat = [
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w
        ]

        d_euler = [wx * dt, wy * dt, wz * dt]
        d_quat = tft.quaternion_from_euler(d_euler[0], d_euler[1], d_euler[2])
        new_quat = tft.quaternion_multiply(current_quat, d_quat)
        
        norm = np.linalg.norm(new_quat)
        new_quat = new_quat / norm

        self.target_pose.pose.orientation.x = new_quat[0]
        self.target_pose.pose.orientation.y = new_quat[1]
        self.target_pose.pose.orientation.z = new_quat[2]
        self.target_pose.pose.orientation.w = new_quat[3]

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            dt = 1.0 / self.publish_rate
            self.update_pose(dt)
            
            self.target_pose.header.seq += 1
            self.target_pose.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.target_pose)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        teleop = SpaceNavTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass