#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from visualization_msgs.msg import Marker, MarkerArray # 추가
from nav_msgs.msg import Path

from scipy.spatial.transform import Rotation as R

from demo_traj.lib.peg_in_hole import PegInHoleDemo 
from demo_traj.lib.utils import transform2homo, homo2pose, invert_transform, sample_vis_indices

class DemoProcessorNode:
    def __init__(self):
        rospy.init_node('demo_processor_node')

        # 1. 파라미터 설정 (Launch 파일 등에서 변경 가능)
        bag_name = "test4_20251030_151337" 
        hole_name = "part1" 
        peg_name = "part11"
        
        # Publisher 추가
        self.demo_axes_pub = rospy.Publisher('/demo_traj_axes', MarkerArray, queue_size=10, latch=True)
        self.demo_path_pub = rospy.Publisher('/demo_traj_path', Path, queue_size=10, latch=True)
        self.retarget_axes_pub = rospy.Publisher('/target_traj_axes', MarkerArray, queue_size=10, latch=True)
        self.retarget_path_pub = rospy.Publisher('/target_traj_path', Path, queue_size=10, latch=True)
        self.tcp_axes_pub = rospy.Publisher('/target_tcp_axes', MarkerArray, queue_size=10, latch=True)
        self.tcp_path_pub = rospy.Publisher('/target_tcp_path', Path, queue_size=10, latch=True)

        world_to_hole_tf = self.get_object_transform("world", "object_part1")
        world_to_peg_tf = self.get_object_transform("world", "object_part11")
        world_to_base_tf = self.get_object_transform("world", "panda_link0")
        tcp_to_obj_tf = self.get_object_transform("panda_hand_tcp", "object_part11")
    
        if world_to_hole_tf and world_to_peg_tf and tcp_to_obj_tf:
            world_to_hole_homo = transform2homo(world_to_hole_tf)
            world_to_peg_homo = transform2homo(world_to_peg_tf)
            world_to_base_homo = transform2homo(world_to_base_tf)
            base_to_world_homo = invert_transform(world_to_base_homo)
            tcp_to_obj_homo = transform2homo(tcp_to_obj_tf) # 잡고 있는 상태의 상대 자세
            
            # 2. PegInHoleDemo 인스턴스 생성 및 리타겟팅
            self.demo = PegInHoleDemo(
                bag_name=bag_name, 
                hole_part_name=hole_name, 
                peg_part_name=peg_name,
                hole_pose_env_homo=world_to_hole_homo,
                use_refined=True
            )

            # 물체의 리타겟팅 궤적 (world frame)
            peg_poses_world_homo = self.demo.peg_poses_env_homo.copy()
            peg_poses_world_homo_approach = peg_poses_world_homo[:self.demo.key_state_idx]
            # sampled_idxs = sample_vis_indices(peg_poses_world_homo, frame_vis_gap=20, sample_axis='y')
            # self.publish_and_visualize_traj(peg_poses_world_homo, self.demo_path_pub, self.demo_axes_pub, sample_idxs=sampled_idxs)

            peg_poses_world_homo_approach_retarget = self.demo.retarget_trajectory_to_new_pose(
                traj_base_homo=peg_poses_world_homo_approach,
                new_pose_base_homo=world_to_peg_homo,
                retarget_start=True
            )
            peg_poses_world_homo_retarget = peg_poses_world_homo.copy()
            peg_poses_world_homo_retarget[:self.demo.key_state_idx] = peg_poses_world_homo_approach_retarget
            # sampled_idxs = sample_vis_indices(peg_poses_world_homo_retarget, frame_vis_gap=20, sample_axis='y')
            # self.publish_and_visualize_traj(peg_poses_world_homo_retarget, self.retarget_path_pub, self.retarget_axes_pub, sample_idxs=sampled_idxs)

            # 3. TCP 궤적 계산 => panda_link0 기준으로 계산해야 함
            obj_to_tcp_homo = invert_transform(tcp_to_obj_homo)
            peg_poses_world_homo_retarget_approach = peg_poses_world_homo_retarget[:self.demo.key_state_idx]
            tcp_poses_base_homo_retarget_approach = []
            for world_to_obj_homo in peg_poses_world_homo_retarget_approach:
                tcp_poses_base_homo_retarget_approach.append(base_to_world_homo @ world_to_obj_homo @ obj_to_tcp_homo)
            tcp_poses_base_homo_retarget_approach = np.stack(tcp_poses_base_homo_retarget_approach)
            sampled_idxs = sample_vis_indices(tcp_poses_base_homo_retarget_approach, frame_vis_gap=len(tcp_poses_base_homo_retarget_approach), sample_axis='y')
            self.publish_and_visualize_traj(
                tcp_poses_base_homo_retarget_approach, 
                self.tcp_path_pub, 
                self.tcp_axes_pub, 
                sample_idxs=sampled_idxs, 
                frame='panda_link0'
            )

            rospy.loginfo("All trajectories (Object & TCP) published.")
        else:
            rospy.logerr("Failed to look up necessary TFs (object_part1, object_part11, or tcp_to_obj)")

    def publish_and_visualize_traj(self, traj, path_pub, axes_pub=None, sample_idxs=[], frame='world'):
        """궤적 발행 함수 공용화"""
        path_msg = Path()
        path_msg.header.frame_id = frame
        path_msg.header.stamp = rospy.Time.now()
        
        for i, matrix in enumerate(traj):
            ps = PoseStamped()
            ps.header = path_msg.header
            # homo2transform을 이용해 Transform 메시지로 바꾼 뒤 Pose 필드에 매핑
            tf_msg = homo2pose(matrix)
            ps.pose = tf_msg
            path_msg.poses.append(ps)

        path_pub.publish(path_msg)

        if axes_pub:
            marker_array = MarkerArray()
            for i, idx in enumerate(sample_idxs):
                axes_markers = self.create_axes_marker(traj[idx], marker_id_start=i*3, frame_id=frame)
                marker_array.markers.extend(axes_markers)
            axes_pub.publish(marker_array)

    def create_axes_marker(self, matrix, marker_id_start, frame_id="world"):
        """
        4x4 행렬을 받아 해당 위치/자세에 RGB 좌표계 축(Line) 생성
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "trajectory_axes"
        marker.id = marker_id_start
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # 선의 두께 설정
        marker.scale.x = 0.001
        
        # 4x4 행렬에서 위치와 회전 행렬 추출
        pos = matrix[:3, 3]
        rot = matrix[:3, :3]
        
        # 축의 길이 설정 (5cm)
        axis_length = 0.01

        # 각 축의 끝점 계산 (Rotation Matrix의 각 열이 축의 방향임)
        x_axis_end = pos + rot[:, 0] * axis_length
        y_axis_end = pos + rot[:, 1] * axis_length
        z_axis_end = pos + rot[:, 2] * axis_length

        # 1. X축 (Red)
        marker.points.append(Point(pos[0], pos[1], pos[2]))
        marker.points.append(Point(x_axis_end[0], x_axis_end[1], x_axis_end[2]))
        for _ in range(2): marker.colors.append(self.make_color(1, 0, 0, 1))

        # 2. Y축 (Green)
        marker.points.append(Point(pos[0], pos[1], pos[2]))
        marker.points.append(Point(y_axis_end[0], y_axis_end[1], y_axis_end[2]))
        for _ in range(2): marker.colors.append(self.make_color(0, 1, 0, 1))

        # 3. Z축 (Blue)
        marker.points.append(Point(pos[0], pos[1], pos[2]))
        marker.points.append(Point(z_axis_end[0], z_axis_end[1], z_axis_end[2]))
        for _ in range(2): marker.colors.append(self.make_color(0, 0, 1, 1))

        # LINE_LIST는 하나의 마커로 동작하므로 리스트에 담아 반환
        return [marker]

    def make_color(self, r, g, b, a):
        from std_msgs.msg import ColorRGBA
        color = ColorRGBA()
        color.r, color.g, color.b, color.a = r, g, b, a
        return color

    def get_object_transform(self, parent_frame="world", child_frame="object_part"):
        """
        TF 트리에서 두 프레임 사이의 변환을 읽어 geometry_msgs/Transform으로 반환
        """
        # 1. TF 리스너 설정
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # TF가 전송될 때까지 약간의 대기 (노드 시작 직후엔 데이터가 없을 수 있음)
        rospy.sleep(1.0)

        try:
            # 2. 특정 시점의 변환 정보 가져오기 (시간을 0으로 설정하면 가장 최근 데이터를 가져옴)
            # lookup_transform(target_frame, source_frame, time)
            trans_stamped = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0), rospy.Duration(1.0))
            
            # 3. TransformStamped에서 Transform 메시지만 추출
            obj_transform = trans_stamped.transform
            
            rospy.loginfo(f"Successfully looked up transform from {parent_frame} to {child_frame}")
            return obj_transform

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF Lookup failed: {e}")
            return None

    def run(self):
        # 여기서는 일단 데이터를 로드하고 리타겟팅이 잘 되었는지 로그만 확인합니다.
        # 시각화 토픽 발행 코드는 다음 단계에서 추가합니다.
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DemoProcessorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass