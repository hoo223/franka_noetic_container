#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import threading
import rospy
import moveit_commander

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (PoseStamped transform을 위해 필요)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


class MoveItApproachNode:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('approach_phase_moveit_node', anonymous=False)

        self.group_name = "panda_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.move_group.set_end_effector_link("panda_hand_tcp")

        # 기본 보수 설정
        # self.move_group.set_max_velocity_scaling_factor(0.1)
        # self.move_group.set_max_acceleration_scaling_factor(0.05)
        # self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(5)
        self.move_group.allow_replanning(True)

        self._lock = threading.Lock()
        self.trajectory_data = None  # type: list[PoseStamped] or None
        self.is_executing = False

        # TF2
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.mode_cmd_pub = rospy.Publisher('/set_controller_mode', String, queue_size=1)
        rospy.Subscriber('/target_tcp_path', Path, self.path_callback, queue_size=1)

        rospy.loginfo("--- MoveIt Approach Node Ready ---")
        rospy.loginfo("Planning Frame: %s", self.move_group.get_planning_frame())
        rospy.loginfo("EEF Link: %s", self.move_group.get_end_effector_link())

    def path_callback(self, msg: Path):
        with self._lock:
            if self.is_executing:
                return
            if not msg.poses:
                rospy.logwarn("Received empty Path.")
                return
            # PoseStamped 리스트 그대로 저장 (frame 유지!)
            self.trajectory_data = msg.poses
        rospy.loginfo("Path received: %d points. frame_id=%s",
                      len(msg.poses), msg.header.frame_id)

    def _transform_pose_stamped(self, ps: PoseStamped, target_frame: str, timeout=0.2) -> PoseStamped:
        """
        PoseStamped를 target_frame으로 변환.
        """
        if ps.header.frame_id == target_frame:
            return ps

        # TF가 늦게 들어오는 경우가 많아서, 최신 사용을 위해 stamp=0 권장(필요시 조정)
        ps2 = PoseStamped()
        ps2.header = ps.header
        ps2.pose = ps.pose
        ps2.header.stamp = rospy.Time(0)

        try:
            tfm = self.tf_buffer.lookup_transform(
                target_frame,
                ps2.header.frame_id,
                rospy.Time(0),
                rospy.Duration(timeout)
            )
            out = tf2_geometry_msgs.do_transform_pose(ps2, tfm)
            out.header.frame_id = target_frame
            return out
        except Exception as e:
            raise RuntimeError(f"TF transform failed {ps.header.frame_id} -> {target_frame}: {e}")

    def plan_and_execute_cartesian(self):
        with self._lock:
            if not self.trajectory_data or self.is_executing:
                return
            self.is_executing = True
            poses_in = list(self.trajectory_data)

        try:
            # 더 보수적으로
            self.move_group.set_max_velocity_scaling_factor(0.2)
            self.move_group.set_max_acceleration_scaling_factor(0.1)

            self.move_group.set_start_state_to_current_state()

            planning_frame = self.move_group.get_planning_frame()

            # 1) TF 변환 + 다운샘플
            # 목표: 너무 많은 점은 피하고(계획/시간폭발), 최대 ~50개 정도로 제한
            max_pts = 50
            step = max(1, len(poses_in) // max_pts)

            waypoints = []
            for i in range(0, len(poses_in)):
                ps = self._transform_pose_stamped(poses_in[i], planning_frame)
                waypoints.append(ps.pose)

            # 마지막은 무조건 포함
            ps_last = self._transform_pose_stamped(poses_in[-1], planning_frame)
            waypoints.append(ps_last.pose)

            rospy.loginfo("Cartesian waypoints: %d (from %d points, step=%d)",
                          len(waypoints), len(poses_in), step)

            # 2) Cartesian path
            eef_step = 0.01          # 필요시 0.002~0.01 튜닝
            avoid_collisions = False    

            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,
                eef_step,
                avoid_collisions,
            )

            rospy.loginfo("compute_cartesian_path fraction = %.3f", fraction)

            # 3) fraction 체크 + retime
            # 너무 낮으면(예: 0.9 미만) 안전하게 폴백
            if fraction < 0.90 or len(plan.joint_trajectory.points) == 0:
                rospy.logwarn("Low cartesian fraction (%.3f). Fallback to final pose PTP.", fraction)
                self._fallback_to_final_pose(ps_last.pose)
                return

            # time parameterization (retime)
            # scaling factor로 계획에 속도/가속도 반영
            plan = self.move_group.retime_trajectory(
                self.move_group.get_current_state(),
                plan,
                velocity_scaling_factor=0.3,
                acceleration_scaling_factor=0.18,
                algorithm="time_optimal_trajectory_generation"  # ROS1 MoveIt에서 흔히 사용
            )

            # 4) execute
            ok = self.move_group.execute(plan, wait=True)
            self.move_group.stop()
            rospy.loginfo("Cartesian execute result: %s", str(ok))

        except Exception as e:
            rospy.logerr("Exception in cartesian planning/execution: %s", str(e))
        finally:
            self.move_group.clear_pose_targets()
            with self._lock:
                self.is_executing = False

    def _fallback_to_final_pose(self, final_pose):
        self.move_group.clear_pose_targets()
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_pose_target(final_pose)
        ok = self.move_group.go(wait=True)
        self.move_group.stop()
        rospy.loginfo("Fallback go() result: %s", str(ok))
        self.move_group.clear_pose_targets()

    def run(self):
        while not rospy.is_shutdown():
            with self._lock:
                ready = (self.trajectory_data is not None) and (not self.is_executing)
            if ready:
                print("\n" + "=" * 30)
                val = input("Press 's' to START Cartesian Execution (or 'q' to quit): ")
                print("=" * 30)

                if val.lower() == 's':
                    self.plan_and_execute_cartesian()
                elif val.lower() == 'q':
                    break
            rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        node = MoveItApproachNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
