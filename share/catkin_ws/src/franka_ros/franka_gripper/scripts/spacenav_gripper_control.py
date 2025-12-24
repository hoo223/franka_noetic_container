#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from sensor_msgs.msg import Joy
from franka_gripper.msg import MoveAction, MoveGoal, GraspAction, GraspGoal

class SpacenavGripperNode:
    def __init__(self):
        rospy.init_node('spacenav_gripper_control')

        # 1. Action Clients 설정
        # 열기(Open)를 위한 MoveAction
        self.move_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        # 닫기(Grasp)를 위한 GraspAction
        self.grasp_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

        rospy.loginfo("Waiting for franka_gripper action servers...")
        self.move_client.wait_for_server()
        self.grasp_client.wait_for_server()
        rospy.loginfo("Connected to gripper servers.")

        # 2. 버튼 상태 저장을 위한 변수 (Rising Edge 검출용)
        # [왼쪽버튼, 오른쪽버튼] 초기값 0
        self.prev_buttons = [0, 0]

        # 3. Subscriber 설정
        self.sub = rospy.Subscriber('/spacenav/joy', Joy, self.joy_callback)

        # 그리퍼 설정 값
        self.open_width = 0.08  # 최대 열림 폭 (8cm)
        self.grasp_force = 60.0 # 잡는 힘 (60N) - 필요시 변경
        self.speed = 0.1        # 동작 속도

    def joy_callback(self, msg):
        # msg.buttons 예시: [1, 0] or [0, 1]
        # Spacenav 버튼은 인덱스 0이 왼쪽, 1이 오른쪽인 경우가 많음 (설정에 따라 다를 수 있으니 확인 필요)
        # 요청하신 대로 [1, 0] -> 열림, [0, 1] -> 닫힘으로 매핑

        current_left = msg.buttons[0]
        current_right = msg.buttons[1]

        # --- 왼쪽 버튼: 열기 (Open) ---
        # 이전에 안 눌렸는데(0) 지금 눌렸다면(1) -> Rising Edge
        if current_left == 1 and self.prev_buttons[0] == 0:
            rospy.loginfo("Left Button Pressed: Opening Gripper")
            self.open_gripper()

        # --- 오른쪽 버튼: 닫기 (Close / Grasp) ---
        # 이전에 안 눌렸는데(0) 지금 눌렸다면(1) -> Rising Edge
        elif current_right == 1 and self.prev_buttons[1] == 0:
            rospy.loginfo("Right Button Pressed: Grasping Object")
            self.close_gripper()

        # 현재 버튼 상태 저장
        self.prev_buttons = msg.buttons

    def open_gripper(self):
        # MoveAction: 지정된 폭으로 이동
        goal = MoveGoal()
        goal.width = self.open_width
        goal.speed = self.speed
        
        # 이전 명령 취소 후 새 명령 전송 (반응성 향상)
        self.move_client.cancel_all_goals()
        self.grasp_client.cancel_all_goals()
        
        self.move_client.send_goal(goal)
        # wait_for_result를 쓰면 콜백이 멈추므로 비동기로 보냅니다.

    def close_gripper(self):
        # GraspAction: 힘 제어 모드 (Blind Grasp)
        goal = GraspGoal()
        
        # [핵심] 크기 무관하게 잡기 위한 설정
        goal.width = 0.0          # 목표는 0까지 닫는 것
        goal.epsilon.inner = 0.005
        goal.epsilon.outer = 0.1  # 오차 범위를 10cm로 넓게 잡음 (무조건 성공 처리)
        
        goal.speed = self.speed
        goal.force = self.grasp_force # 설정한 힘 유지

        # 이전 명령 취소 후 새 명령 전송
        self.move_client.cancel_all_goals()
        self.grasp_client.cancel_all_goals()

        self.grasp_client.send_goal(goal)

if __name__ == '__main__':
    try:
        node = SpacenavGripperNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass