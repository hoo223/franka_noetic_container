#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped

class SpaceMouseFixedStep:
    def __init__(self):
        rospy.init_node('spacemouse_fixed_step_integrator')

        # --- 사전 정의된 고정 이동 크기 설정 ---
        self.LINEAR_STEP = 0.05   # 초당 5cm 이동 (50Hz 기준 루프당 1mm)
        self.ANGULAR_STEP = 0.4  # 초당 0.001 rad 회전

        self.target_pos = None
        self.target_quat = None

        self.listener = tf.TransformListener()
        self.twist_sub = rospy.Subscriber("/spacenav/twist", Twist, self.twist_callback)
        self.pose_pub = rospy.Publisher("/cartesian_pose_controller/tcp_target_pose", 
                                         PoseStamped, queue_size=1, tcp_nodelay=True)

        self.twist = Twist()
        
        rospy.loginfo("TF로 panda_hand_tcp 위치 읽어오는 중...")
        success = False
        while not success and not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('panda_link0', 'panda_hand_tcp', rospy.Time(0))
                self.target_pos = np.array(trans)
                self.target_quat = np.array(rot)
                success = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.sleep(0.1)
                continue
        
        rospy.loginfo("고정 속도 모드 준비 완료.")

    def twist_callback(self, msg):
        self.twist = msg

    def get_direction(self, x, y, z):
        """입력 벡터의 방향(Unit Vector)만 추출"""
        vec = np.array([x, y, z])
        norm = np.linalg.norm(vec)
        if norm < 0.01: # 아주 미세한 입력은 무시 (실제 0에 가까운 값)
            return np.zeros(3)
        return vec / norm

    def run(self):
        # 통신 성공률 확보를 위해 100Hz 유지
        rate = rospy.Rate(200) 
        dt = 0.005  # 200Hz 간격

        while not rospy.is_shutdown():
            # 1. 위치 업데이트: 입력의 크기에 상관없이 '방향'만 추출하여 고정된 STEP만큼 이동
            # lin_dir = self.get_direction(-self.twist.linear.x, self.twist.linear.y, self.twist.linear.z)
            # self.target_pos += lin_dir * self.LINEAR_STEP * dt

            # 2. 회전 업데이트: 입력의 방향만 추출하여 고정된 STEP만큼 회전
            ang_dir = self.get_direction(-self.twist.angular.x, -self.twist.angular.y, self.twist.angular.z)
            print(ang_dir)
            if np.any(ang_dir):
                angle = self.ANGULAR_STEP * dt
                axis = ang_dir
                
                delta_quat = tf.transformations.quaternion_about_axis(angle, axis)
                self.target_quat = tf.transformations.quaternion_multiply(delta_quat, self.target_quat)
                print(self.target_quat)

            # 3. 메시지 발행
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "panda_link0"
            pose_msg.pose.position.x = self.target_pos[0]
            pose_msg.pose.position.y = self.target_pos[1]
            pose_msg.pose.position.z = self.target_pos[2]
            pose_msg.pose.orientation.x = self.target_quat[0]
            pose_msg.pose.orientation.y = self.target_quat[1]
            pose_msg.pose.orientation.z = self.target_quat[2]
            pose_msg.pose.orientation.w = self.target_quat[3]

            self.pose_pub.publish(pose_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SpaceMouseFixedStep()
        node.run()
    except rospy.ROSInterruptException:
        pass