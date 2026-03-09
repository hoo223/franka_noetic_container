#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest, ListControllers
import json
import cv2
import tf.transformations as tft


class SocketPoseRegisterNode:
    def __init__(self):
        rospy.init_node('socket_pose_register_node')
        moveit_commander.roscpp_initialize(sys.argv)

        self.path = os.path.dirname(__file__)
        self.fixed_pose_root = os.path.join(self.path, "fixed_pose")
        self.base_frame = "panda_link0"
        self.part_name = "part1"
        self.part_frame = f"object_{self.part_name}"
        self.part_current_file = os.path.join(self.fixed_pose_root, self.part_name, "current.json")
        self.socket_view_tcp_frame = f"socket_view_tcp_{self.part_name}"

        self.socket_view_tcp_pose = {
            "position": {
                "x": 0.08,
                "y": 0.0,
                "z": 0.12,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0,
            },
        }

        self.target_area_ratio = {
            'part1': 0.18,
            'part7': 0.05,
            'part9': 0.05,
            'part11': 0.03,
        }
        self.mask_data = None
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.arm = moveit_commander.MoveGroupCommander("panda_arm")  # type: ignore[operator]
        self.arm.set_end_effector_link("panda_hand_tcp")

        self.pos_controller = "position_joint_trajectory_controller"
        self.imp_controller = "cartesian_impedance_example_controller"
        self.pose_controller = "cartesian_pose_controller"
        self.all_controllers = [self.pos_controller, self.imp_controller, self.pose_controller]
        self.switch_srv = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        self.list_srv = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)

        self.fp_pub = rospy.Publisher('/fp_target_object', String, queue_size=1, latch=False)
        self.sam2_pub = rospy.Publisher('/sam2_target_object', String, queue_size=1, latch=False)
        self.target_frame_pub = rospy.Publisher('/change_target_frame', String, queue_size=1, latch=True)
        self.fixed_poses = {}

        rospy.sleep(0.5)
        rospy.set_param('/selected_peg', self.part_name)
        self.publish_selected_target(repeat=3, interval=0.2)
        rospy.loginfo("Socket Pose Register Node Started")

    def request_pose_estimation(self, socket_name):
        """FoundationPose에 자세 추정 요청"""
        rospy.loginfo(f"Requesting pose estimation for socket: {socket_name}")
        
        msg = String()
        msg.data = socket_name
        self.fp_pub.publish(msg)
        
        rospy.sleep(0.5)

    def collect_pose_samples(self, socket_name, num_samples=10, timeout=10):
        """FoundationPose에서 자세 샘플 수집"""
        poses_in_base = []
        topic_name = f"/object_pose/{socket_name}"
        
        start_time = rospy.Time.now()
        
        rospy.loginfo(f"Collecting {num_samples} samples from {topic_name}...")
        
        while len(poses_in_base) < num_samples:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Timeout while collecting samples.")
                break
            
            try:
                msg = rospy.wait_for_message(topic_name, PoseStamped, timeout=0.5)
                
                pose_base = self.tf_buffer.transform(msg, self.base_frame, timeout=rospy.Duration.from_sec(1.0))
                poses_in_base.append(pose_base)
                
                if len(poses_in_base) >= num_samples:
                    poses_in_base = self.filter_outliers(poses_in_base)
                
                print(f"Collected {len(poses_in_base)}/{num_samples} samples...", end='\r')
                
            except rospy.ROSException:
                continue
            except Exception:
                continue
        
        print("")
        return poses_in_base

    def filter_outliers(self, poses, threshold=0.03):
        """중앙값 기반 이상치 필터링"""
        if len(poses) < 3:
            return poses
        
        positions = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses])
        median_pos = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median_pos, axis=1)
        
        filtered_indices = np.where(distances < threshold)[0]
        filtered_poses = [poses[i] for i in filtered_indices]
        
        rospy.loginfo(f"Filtered outliers: {len(poses)} -> {len(filtered_poses)} samples")
        
        return filtered_poses if len(filtered_poses) >= len(poses) // 2 else poses

    def calculate_average_pose(self, poses):
        """PoseStamped 리스트의 평균 자세 계산"""
        pos_list = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses]
        quat_list = [[p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w] for p in poses]
        
        avg_pos = np.mean(pos_list, axis=0)
        avg_quat = np.mean(quat_list, axis=0)
        avg_quat /= np.linalg.norm(avg_quat)
        
        return {
            'position': {'x': avg_pos[0], 'y': avg_pos[1], 'z': avg_pos[2]},
            'orientation': {'x': avg_quat[0], 'y': avg_quat[1], 'z': avg_quat[2], 'w': avg_quat[3]}
        }

    def save_pose_to_file(self, pose_data, file_path):
        """자세를 JSON 파일로 저장"""
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        with open(file_path, 'w') as f:
            json.dump(pose_data, f, indent=4)

        rospy.loginfo(f"Saved pose to: {file_path}")
        return file_path

    def update_fixed_pose(self, pose_data, parent_frame, child_frame):
        """TF 정보 등록"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = pose_data['position']['x']
        t.transform.translation.y = pose_data['position']['y']
        t.transform.translation.z = pose_data['position']['z']
        t.transform.rotation.x = pose_data['orientation']['x']
        t.transform.rotation.y = pose_data['orientation']['y']
        t.transform.rotation.z = pose_data['orientation']['z']
        t.transform.rotation.w = pose_data['orientation']['w']
        
        self.fixed_poses[child_frame] = t

    def publish_all_static_tfs(self):
        """모든 Static TF 발행"""
        if self.fixed_poses:
            self.static_br.sendTransform(list(self.fixed_poses.values()))
            rospy.loginfo(f"Broadcasted {len(self.fixed_poses)} static transforms")

    def get_pose_from_tf(self, target_frame, parent_frame=None):
        if parent_frame is None:
            parent_frame = self.base_frame
        try:
            trans = self.tf_buffer.lookup_transform(parent_frame, target_frame, rospy.Time(0), rospy.Duration.from_sec(2.0))
            pose = PoseStamped()
            pose.header.frame_id = parent_frame
            pose.pose.position = trans.transform.translation
            pose.pose.orientation = trans.transform.rotation
            return pose
        except Exception as e:
            rospy.logerr(f"TF Lookup failed for {target_frame}: {e}")
            return None

    def move_tcp_to_frame(self, target_frame):
        target_pose_stamped = self.get_pose_from_tf(target_frame)
        if target_pose_stamped is None:
            return False

        self.arm.set_pose_target(target_pose_stamped.pose)
        success = self.arm.go(wait=True)
        self.arm.stop()
        self.arm.clear_pose_targets()
        return success

    def _mask_callback(self, msg):
        try:
            self.mask_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr(f"Mask conversion error: {e}")

    def wait_for_segmentation(self, timeout=2.0):
        mask_topic = f"/sam2_mask/{self.part_name}"
        try:
            rospy.wait_for_message(mask_topic, Image, timeout=timeout)
            rospy.loginfo(f"Segmentation stream is active: {mask_topic}")
            return True
        except rospy.ROSException:
            rospy.logwarn(f"No segmentation message from {mask_topic} within {timeout:.1f}s")
            return False

    def switch_controller(self, target_controller):
        try:
            list_res = self.list_srv()
            running_controllers = [c.name for c in list_res.controller if c.state == 'running']

            if target_controller in running_controllers:
                rospy.loginfo(f"{target_controller} is already running")
                return True

            to_stop = [c for c in running_controllers if c in self.all_controllers]
            req = SwitchControllerRequest()
            req.start_controllers = [target_controller]
            req.stop_controllers = to_stop
            req.strictness = SwitchControllerRequest.STRICT
            req.start_asap = True

            res = self.switch_srv(req)
            if not res.ok:
                rospy.logwarn(f"Failed to switch to controller: {target_controller}")
            return res.ok
        except Exception as e:
            rospy.logwarn(f"Controller switch failed: {e}")
            return False

    def zoom_to_object(self, obj_name, target_area_ratio=0.05):
        rospy.loginfo(f"Starting Zoom to {obj_name}...")
        self.mask_data = None
        sub = rospy.Subscriber(f"/sam2_mask/{obj_name}", Image, self._mask_callback)
        pose_pub = rospy.Publisher('/cartesian_pose_controller/tcp_target_pose', PoseStamped, queue_size=1)

        gain_trans = 0.0003
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.mask_data is None:
                rate.sleep()
                continue

            try:
                trans = self.tf_buffer.lookup_transform(self.base_frame, "panda_hand_tcp", rospy.Time(0))
                mask_np = np.asarray(self.mask_data)
                moments = cv2.moments(mask_np)
                if moments['m00'] <= 0:
                    rate.sleep()
                    continue

                u = int(moments['m10'] / moments['m00'])
                v = int(moments['m01'] / moments['m00'])
                h, w = mask_np.shape[:2]
                err_u, err_v = (w / 2 - u), (h / 2 - v)
                area_ratio = moments['m00'] / (h * w * 255.0)

                d_x_cam = err_u * gain_trans
                d_y_cam = err_v * gain_trans
                d_z_cam = 0.01 if area_ratio < target_area_ratio else 0.0

                q = [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
                rotation_matrix = tft.quaternion_matrix(q)[:3, :3]
                delta_cam = np.array([d_x_cam, d_y_cam, d_z_cam])
                delta_base = np.dot(rotation_matrix, delta_cam)

                target_stamped = PoseStamped()
                target_stamped.header.frame_id = self.base_frame
                target_stamped.header.stamp = rospy.Time.now()
                target_stamped.pose.position.x = trans.transform.translation.x + delta_base[0]
                target_stamped.pose.position.y = trans.transform.translation.y + delta_base[1]
                target_stamped.pose.position.z = trans.transform.translation.z + delta_base[2]
                target_stamped.pose.orientation = trans.transform.rotation
                pose_pub.publish(target_stamped)

                if abs(err_u) < 10 and abs(err_v) < 10 and area_ratio >= target_area_ratio:
                    rospy.loginfo("Zoom reached target")
                    break

            except Exception as e:
                rospy.logerr(f"Zoom Error: {e}")

            rate.sleep()

        sub.unregister()

    def print_memory_part_pose(self):
        memory_frame = f"memory_{self.part_name}"
        pose = self.get_pose_from_tf(memory_frame, self.base_frame)
        if pose is None:
            rospy.logwarn(f"Failed to read pose for {memory_frame} in {self.base_frame}")
            return False

        p = pose.pose.position
        q = pose.pose.orientation
        pose_dict = {
            "position": {
                "x": p.x,
                "y": p.y,
                "z": p.z,
            },
            "orientation": {
                "x": q.x,
                "y": q.y,
                "z": q.z,
                "w": q.w,
            },
        }
        print(json.dumps(pose_dict, indent=4))
        return True

    def activate_socket_view_control(self):
        """object_part1 기준 사전정의 TCP 타겟 프레임 등록/이동/제어프레임 전환"""
        self.fixed_poses = {}
        rospy.set_param('/selected_peg', self.part_name)

        self.update_fixed_pose(
            pose_data=self.socket_view_tcp_pose,
            parent_frame=self.part_frame,
            child_frame=self.socket_view_tcp_frame,
        )
        self.publish_all_static_tfs()

        frame_msg = String()
        frame_msg.data = self.socket_view_tcp_frame
        self.target_frame_pub.publish(frame_msg)
        rospy.set_param('/target_pose_frame', self.socket_view_tcp_frame)

        moved = self.move_tcp_to_frame(self.socket_view_tcp_frame)
        if moved:
            rospy.loginfo(f"Socket view control frame ready: {self.socket_view_tcp_frame}")
        else:
            rospy.logwarn(f"Failed to move to {self.socket_view_tcp_frame}. TF may be missing for {self.part_frame}.")
        return moved

    def register_part1_current_pose(self):
        """part1 포즈를 평균값으로 fixed_pose/part1/current.json에 저장"""
        self.request_pose_estimation(self.part_name)

        poses = self.collect_pose_samples(self.part_name)

        if len(poses) < 3:
            rospy.logerr("Not enough samples collected. Registration failed.")
            return False

        avg_pose_dict = self.calculate_average_pose(poses)

        file_path = self.save_pose_to_file(avg_pose_dict, self.part_current_file)

        self.fixed_poses = {}
        self.update_fixed_pose(
            pose_data=avg_pose_dict,
            parent_frame=self.base_frame,
            child_frame=self.part_frame,
        )

        self.publish_all_static_tfs()

        rospy.loginfo(f"'{self.part_name}' pose registered successfully!")
        print(f"\n Saved file: {file_path}")
        print(f" TF frame: {self.part_frame}")

        return True

    def publish_selected_target(self, repeat=1, interval=0.0):
        rospy.set_param('/selected_peg', self.part_name)
        msg = String()
        msg.data = self.part_name
        for i in range(repeat):
            self.fp_pub.publish(msg)
            self.sam2_pub.publish(msg)
            if interval > 0.0 and i < repeat - 1:
                rospy.sleep(interval)
        rospy.loginfo(f"Published selected target: {self.part_name} (x{repeat})")

    def run(self):
        """메인 실행 루프"""
        print("\n" + "="*50)
        print(" Socket Pose Register Node")
        print("="*50)
        print(" Options:")
        print("  [1] Publish selected target (part1)")
        print("  [2] ZOOM_TO_OBJECT (check /sam2_mask/part1 first)")
        print("  [3] Print part1 pose (memory_part1 wrt panda_link0)")
        print("  [4] Move to predefined socket-view TCP frame")
        print("  [5] Save averaged part1 pose to fixed_pose/part1/current.json")
        print("  [q] Quit")
        print("="*50)

        while not rospy.is_shutdown():
            print("\n>> Select option: ", end='', flush=True)
            option = input().strip().lower()

            if option == '1':
                self.publish_selected_target()
            elif option == '2':
                self.switch_controller(self.pose_controller)
                rospy.sleep(0.5)
                if not self.wait_for_segmentation(timeout=2.0):
                    continue
                self.zoom_to_object(
                    self.part_name,
                    self.target_area_ratio.get(self.part_name, 0.05),
                )
            elif option == '3':
                self.print_memory_part_pose()
            elif option == '4':
                self.activate_socket_view_control()
            elif option == '5':
                self.register_part1_current_pose()
            elif option == 'q':
                rospy.loginfo("Shutting down node...")
                break
            else:
                print("Invalid option. Please enter 1, 2, 3, 4, 5, or q.")

            rospy.sleep(0.1)


if __name__ == '__main__':
    try:
        node = SocketPoseRegisterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except EOFError:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
