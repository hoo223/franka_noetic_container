#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import numpy as np
import cv2

# ==========================================
# 1. PnP 결과 입력 (단위: meter)
# ==========================================
# 앞서 구한 rvec, tvec 값을 여기에 입력하세요.
# (예시 값입니다. 본인의 값으로 교체 필수!)
pnp_rvec = np.array([[ 2.34688697], [ 1.04782877], [-0.33317723]]) # Rotation Vector
pnp_tvec = np.array([[-0.40537013], [-0.11817677], [ 0.90783591]]) # Translation Vector (m 단위)
# position: [-0.8520174145635915, 0.10309158303282767, -0.21146669106354837, -1.207251787623118, -0.003209620167232779, 1.3212008865576081, 2.023816270708023, 0.04012594372034073, 0.04012594372034073]
# === 결과 확인 ===
# Rotation Vector (rvec):
# [[ 2.34688697]
#  [ 1.04782877]
#  [-0.33317723]]
# Translation Vector (tvec):
# [[-0.40537013]
#  [-0.11817677]
#  [ 0.90783591]]
# Rotation Matrix (R):
# [[ 0.66655823  0.74544015 -0.00437157]
#  [ 0.61107097 -0.54974789 -0.56953448]
#  [-0.42695713  0.37695656 -0.82195582]]


def get_matrix_from_pnp(rvec, tvec):
    """
    OpenCV PnP 결과(rvec, tvec)를 4x4 변환 행렬로 변환
    이는 T_camera_world (World 점을 Camera 좌표계로 옮기는 행렬)입니다.
    """
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = tvec.reshape(3)
    return T

def main():
    rospy.init_node('camera_pose_publisher')

    # 1. PnP 결과를 4x4 행렬로 변환 (T_cw)
    T_cw = get_matrix_from_pnp(pnp_rvec, pnp_tvec)

    # 2. TF 메시지 생성
    trans = tf.transformations.translation_from_matrix(T_cw)
    quat = tf.transformations.quaternion_from_matrix(T_cw)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_msg = geometry_msgs.msg.TransformStamped()

    static_msg.header.stamp = rospy.Time.now()
    #static_msg.header.frame_id = "world" # 기준(부모): World
    #static_msg.child_frame_id = "camera_color_optical_frame" # 대상(자식): Camera
    static_msg.header.frame_id = "camera_color_optical_frame" # 기준(부모): World
    static_msg.child_frame_id = "world" # 대상(자식): Camera


    static_msg.transform.translation.x = trans[0]
    static_msg.transform.translation.y = trans[1]
    static_msg.transform.translation.z = trans[2]

    static_msg.transform.rotation.x = quat[0]
    static_msg.transform.rotation.y = quat[1]
    static_msg.transform.rotation.z = quat[2]
    static_msg.transform.rotation.w = quat[3]

    # 3. TF 발행
    broadcaster.sendTransform(static_msg)
    
    rospy.loginfo("Publishing TF: camera_color_optical_frame -> world")
    rospy.loginfo(f"Camera Position in World: {trans}")
    
    rospy.spin()

if __name__ == '__main__':
    main()