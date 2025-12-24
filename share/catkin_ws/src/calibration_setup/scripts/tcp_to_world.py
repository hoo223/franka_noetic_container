#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix

def publish_inverse_tf():
    rospy.init_node('inverse_world_tf_publisher')

    # ==========================================
    # 1. 입력: World 기준 TCP의 위치와 회전
    # ==========================================
    # 위치 (x, y, z) [meter]
    tcp_pos_in_world = np.array([0.675, 0.3, 0.03])
    
    # 회전: World 기준 X축 180도 회전
    # (Roll=180, Pitch=0, Yaw=0) -> Quaternion (x, y, z, w) = (1, 0, 0, 0)
    # 만약 다른 각도라면 tf.transformations.quaternion_from_euler 등을 사용하세요.
    tcp_quat_in_world = np.array([1.0, 0.0, 0.0, 0.0])

    # TF 프레임 이름
    parent_frame = "panda_hand_tcp"  # 기준이 될 로봇 끝단 (TCP)
    child_frame = "world"     # 생성할 월드 좌표계 이름

    # ==========================================
    # 2. 역변환 계산 (World -> TCP  ==>  TCP -> World)
    # ==========================================
    
    # (1) World -> TCP 변환 행렬(4x4) 생성
    T_world_tcp = quaternion_matrix(tcp_quat_in_world) # 회전 행렬 변환
    T_world_tcp[0:3, 3] = tcp_pos_in_world             # 이동 벡터 삽입

    # (2) 역행렬 계산 (Inverse)
    T_tcp_world = np.linalg.inv(T_world_tcp)

    # (3) 결과에서 Translation과 Rotation(Quaternion) 분리
    trans_inv = T_tcp_world[0:3, 3]
    quat_inv = quaternion_from_matrix(T_tcp_world)

    print("=== Calculated Inverse Transform (TCP -> World) ===")
    print(f"Translation: {trans_inv}")
    print(f"Quaternion:  {quat_inv}")

    # ==========================================
    # 3. Static TF 발행
    # ==========================================
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_tf = geometry_msgs.msg.TransformStamped()

    static_tf.header.stamp = rospy.Time.now()
    static_tf.header.frame_id = parent_frame
    static_tf.child_frame_id = child_frame

    static_tf.transform.translation.x = trans_inv[0]
    static_tf.transform.translation.y = trans_inv[1]
    static_tf.transform.translation.z = trans_inv[2]

    static_tf.transform.rotation.x = quat_inv[0]
    static_tf.transform.rotation.y = quat_inv[1]
    static_tf.transform.rotation.z = quat_inv[2]
    static_tf.transform.rotation.w = quat_inv[3]

    broadcaster.sendTransform(static_tf)
    
    print(f"Publishing static TF: '{parent_frame}' -> '{child_frame}'")
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_inverse_tf()
    except rospy.ROSInterruptException:
        pass