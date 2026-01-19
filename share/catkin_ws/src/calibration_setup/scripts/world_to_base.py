#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np
import math

def main():
    rospy.init_node('world_to_base_broadcaster')

    # 1. SVD로 계산된 4x4 행렬 (예시 데이터)
    # 실제로는 Step 1 코드의 결과물인 T_B_W를 여기에 넣으세요.
    T = np.array([
        [7.15082352e-01, -6.99037268e-01, 2.03199652e-03, 1.27435003e-01],
        [6.99039845e-01, 7.15081960e-01, -1.04173968e-03, 1.15195809e-01],
        [-7.24829190e-04, 2.16537619e-03, 9.99997393e-01, -7.63607122e-10],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00],
    ])

    # ==========================================
    # 추가: Yaw 회전 오프셋 설정 (단위: Degree)
    # ==========================================
    # 예: 계산된 결과보다 0.5도 더 회전시키고 싶을 때
    yaw_offset_deg = 0.0
    yaw_offset_rad = math.radians(yaw_offset_deg)

    # Z축 기준 회전 행렬 생성
    T_offset = tf.transformations.rotation_matrix(yaw_offset_rad, (0, 0, 1))

    # 기존 행렬에 오프셋 적용 (로봇 베이스 좌표계 기준 회전)
    T_final = np.dot(T, T_offset)

    # 2. 최종 행렬에서 Quaternion 추출
    quat = tf.transformations.quaternion_from_matrix(T_final)

    # ==========================================
    # 3. TF 메시지 생성
    # ==========================================
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"       # 부모 프레임
    static_transformStamped.child_frame_id = "panda_link0"  # 자식 프레임 (로봇 베이스)

    static_transformStamped.transform.translation.x = T[0, 3]
    static_transformStamped.transform.translation.y = T[1, 3]
    static_transformStamped.transform.translation.z = T[2, 3]

    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    # ==========================================
    # 4. TF 송출
    # ==========================================
    broadcaster.sendTransform(static_transformStamped)
    
    rospy.loginfo("=== World -> Panda Base TF Published ===")
    # rospy.loginfo(f"Translation: x={x_m:.4f}m, y={y_m:.4f}m, z={z_m:.4f}m")
    # rospy.loginfo(f"Rotation: {measured_yaw_deg} deg ({yaw_rad:.4f} rad)")

    rospy.spin()

if __name__ == '__main__':
    main()