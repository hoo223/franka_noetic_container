#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import tf2_ros
from geometry_msgs.msg import PoseStamped

class PandaPlanner:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('panda_moveit_planner')
        
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")
        self.arm.set_end_effector_link("panda_hand_tcp") # TCP 프레임 확인 필수
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(1.0)

    def get_pose_from_tf(self, target_frame):
        try:
            trans = self.tf_buffer.lookup_transform("panda_link0", target_frame, rospy.Time(0), rospy.Duration(2.0))
            pose = PoseStamped()
            pose.header.frame_id = "panda_link0"
            pose.pose.position = trans.transform.translation
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            rospy.logerr(f"TF Lookup failed for {target_frame}: {e}")
            return None

    def set_trajectory_time(self, plan, target_duration):
        """궤적 전체의 도달 시간을 target_duration(초)으로 강제 설정"""
        points = plan.joint_trajectory.points
        num_points = len(points)
        
        if num_points == 0: return plan

        for i in range(num_points):
            # 시작부터 끝까지 선형적으로 시간 배분 (0초 ~ target_duration초)
            t = (i / float(num_points - 1)) * target_duration
            points[i].time_from_start = rospy.Duration(t)
            
            # 속도와 가속도는 0으로 초기화 (시간에 맞춰 로봇이 부드럽게 계산함)
            points[i].velocities = [0.0] * len(points[i].positions)
            points[i].accelerations = [0.0] * len(points[i].positions)
            
        return plan

    def execute_sequence(self, obj_name):
        # 1. Pre-Grasp 위치로 이동
        pre_grasp_pose = self.get_pose_from_tf(f"pre_grasp_{obj_name}")
        if pre_grasp_pose:
            rospy.loginfo("Going to Pre-Grasp...")
            self.arm.set_pose_target(pre_grasp_pose)
            success = self.arm.go(wait=True)
            self.arm.stop()
            if not success: return False
        
        # 2. Grasp 위치로 직선(Cartesian) 진입
        grasp_pose_stamped = self.get_pose_from_tf(f"grasp_{obj_name}")
        if grasp_pose_stamped:
            rospy.loginfo("Approaching Grasp Point...")
            waypoints = [grasp_pose_stamped.pose]
            self.arm.set_max_velocity_scaling_factor(0.05)
            (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, False)
            final_plan = self.set_trajectory_time(plan, 2.0) # 2초 동안 이동하도록 설정
            if fraction > 0.9:
                self.arm.execute(final_plan, wait=True)
                rospy.loginfo("Grasp position reached!")
                return True
        return False

if __name__ == '__main__':
    planner = PandaPlanner()
    planner.execute_sequence("part11")