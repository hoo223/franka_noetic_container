#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers
from demo_traj.lib.utils import transform2homo, homo2pose
import numpy as np

class ImpedanceSpacenavTest:
    def __init__(self):
        rospy.init_node('impedance_spacenav_test')

        # 1. 초기 자세 저장을 위한 TF 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 2. 제어 명령 Publisher 및 스페이스마우스 구독
        self.target_pub = rospy.Publisher(
            '/cartesian_impedance_example_controller/equilibrium_pose', 
            PoseStamped, queue_size=1)
        
        self.base_pose = None
        self.curr_offset = np.zeros(3)
        
        # 3. 서비스 클라이언트
        self.switch_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.list_srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)

        rospy.loginfo("Waiting for TFs and Spacenav...")
        rospy.sleep(1.0) # TF 안정을 위한 대기

    def get_current_pose(self):
        """현재 로봇의 실제 위치를 World 기준 Matrix로 가져옴"""
        try:
            trans = self.tf_buffer.lookup_transform("panda_link0", "panda_hand_tcp", rospy.Time(0), rospy.Duration(1.0))
            return transform2homo(trans.transform)
        except Exception as e:
            rospy.logerr(f"TF Lookup failed: {e}")
            return None

    def switch_to_impedance(self):
        """컨트롤러 스위칭 로직"""
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            # 현재 구동 중인 컨트롤러 파악
            res_list = self.list_srv()
            to_stop = [c.name for c in res_list.controller if c.state == 'running' and 'trajectory' in c.name]
            
            req = SwitchControllerRequest()
            req.start_controllers = ['cartesian_impedance_example_controller']
            req.stop_controllers = to_stop
            req.strictness = SwitchControllerRequest.STRICT
            
            res = self.switch_srv(req)
            return res.ok
        except Exception as e:
            rospy.logerr(f"Switch failed: {e}")
            return False

    def spacenav_callback(self, msg):
        """Vector3 타입의 스페이스마우스 오프셋 반영"""
        if self.base_pose is None: return
        
        # 스페이스마우스 감도 조절 (Vector3이므로 msg.x, y, z로 접근)
        scale = 0.1 
        
        target_matrix = self.base_pose.copy()
        target_matrix[0, 3] += msg.x * scale
        target_matrix[1, 3] += msg.y * scale
        target_matrix[2, 3] += msg.z * scale

        # 메시지 발행 (equilibrium_pose는 여전히 PoseStamped)
        target_msg = PoseStamped()
        target_msg.header.frame_id = "panda_link0"
        target_msg.header.stamp = rospy.Time.now()
        target_msg.pose = homo2pose(target_matrix)
        
        self.target_pub.publish(target_msg)

    def run(self):
        # 1. 현재 포즈 저장 (이 지점을 0,0,0 기준으로 삼음)
        current_m = self.get_current_pose()
        if current_m is None: return
        self.base_pose = current_m

        # 2. 임피던스 모드로 전환
        if self.switch_to_impedance():
            rospy.loginfo("Switched to Impedance Mode. Use Spacenav to move!")
            # 3. 스페이스마우스 구독 시작
            rospy.Subscriber('/spacenav/offset', Vector3, self.spacenav_callback)
            rospy.spin()
        else:
            rospy.logerr("Could not switch to Impedance Controller.")

if __name__ == '__main__':
    tester = ImpedanceSpacenavTest()
    tester.run()