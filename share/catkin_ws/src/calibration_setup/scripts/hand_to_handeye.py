#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np

# ==========================================
# 1. Hand-Eye Calibration 결과 입력
# (제공해주신 panda_link0 -> camera 변환 값)
# ==========================================
# Translation (x, y, z)
# Rotation Quaternion (x, y, z, w)
# calib_trans = [0.080, -0.078, 0.025]
# calib_quat = [-0.112, 0.272, 0.886, -0.358]

# calib_trans = [0.080, -0.07, 0.030]
# calib_quat = [-0.116, 0.281, 0.885, -0.353]

calib_trans = [0.063, -0.070, 0.037]
calib_quat = [-0.120, 0.285, 0.885, -0.349]

def get_matrix(trans, quat):
    """Translation 리스트와 Quaternion 리스트를 4x4 행렬로 변환"""
    T = tf.transformations.quaternion_matrix(quat)
    T[0:3, 3] = trans
    return T

def main():
    rospy.init_node('camera_to_base_publisher')

    # 1. Base -> Camera 행렬 생성 (T_bc)
    T_hc = get_matrix(calib_trans, calib_quat)

    # 2. 역행렬 계산: Camera -> Base (T_cb)
    # 우리가 송출할 TF의 부모는 카메라이고 자식은 베이스이므로 역변환이 필요합니다.
    T_ch = np.linalg.inv(T_hc)

    # 3. 역변환된 행렬에서 다시 Translation, Quaternion 추출
    trans = tf.transformations.translation_from_matrix(T_hc)
    quat = tf.transformations.quaternion_from_matrix(T_hc)

    # 4. TF Broadcaster 설정
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_msg = geometry_msgs.msg.TransformStamped()

    static_msg.header.stamp = rospy.Time.now()
    static_msg.header.frame_id = "panda_hand" # 부모
    static_msg.child_frame_id = "camera_color_optical_frame" # 자식

    static_msg.transform.translation.x = trans[0]
    static_msg.transform.translation.y = trans[1]
    static_msg.transform.translation.z = trans[2]

    static_msg.transform.rotation.x = quat[0]
    static_msg.transform.rotation.y = quat[1]
    static_msg.transform.rotation.z = quat[2]
    static_msg.transform.rotation.w = quat[3]

    # 5. TF 송출
    broadcaster.sendTransform(static_msg)
    
    rospy.loginfo("Publishing TF: camera_color_optical_frame -> panda_hand")
    rospy.loginfo(f"Base Position relative to Camera: \n{trans}")
    
    rospy.spin()

if __name__ == '__main__':
    main()