#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
from tf.transformations import quaternion_from_matrix

def publish_static_tf():
    rospy.init_node('depth_to_color_tf_publisher')

    # ==========================================
    # 1. 사용자 입력 데이터
    # ==========================================
    
    # 3x3 회전 행렬 (Row-major order)
    rotation_data = [
        0.9999971985816956, -0.0009309045854024589, -0.002173537854105234,
        0.0009337298688478768, 0.9999987483024597, 0.0012991802068427205,
        0.002172325737774372, -0.0013012060662731528, 0.9999967813491821
    ]
    
    # 이동 벡터 (x, y, z)
    translation_data = [-2.06662698474247e-05, 2.4687380573595874e-05, 8.946547313826159e-05]

    # TF 프레임 이름 설정 (환경에 맞게 수정하세요)
    # 일반적으로: Depth 센서가 기준(Parent) -> Color 센서가 타겟(Child)
    parent_frame_id = "camera_depth_optical_frame"
    child_frame_id = "camera_color_optical_frame" 

    # ==========================================
    # 2. 데이터 변환 (Matrix -> Quaternion)
    # ==========================================
    
    # 리스트를 3x3 numpy 행렬로 변환
    R = np.array(rotation_data).reshape(3, 3)
    
    # 4x4 동차 변환 행렬(Homogeneous Matrix) 생성 (변환 함수 입력을 위해)
    T = np.eye(4)
    T[:3, :3] = R
    
    # 회전 행렬을 쿼터니언으로 변환 (x, y, z, w)
    quat = quaternion_from_matrix(T)
    
    print(f"=== Calculated Quaternion ===")
    print(f"x: {quat[0]}, y: {quat[1]}, z: {quat[2]}, w: {quat[3]}")

    # ==========================================
    # 3. Static TF 발행
    # ==========================================
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    # 헤더 설정
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parent_frame_id
    static_transformStamped.child_frame_id = child_frame_id

    # 이동(Translation) 설정
    static_transformStamped.transform.translation.x = translation_data[0]
    static_transformStamped.transform.translation.y = translation_data[1]
    static_transformStamped.transform.translation.z = translation_data[2]

    # 회전(Rotation) 설정
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    # 발행 (Static TF는 한 번 발행하면 됨, 하지만 래치되어 지속됨)
    broadcaster.sendTransform(static_transformStamped)
    
    print(f"Successfully published static TF from '{parent_frame_id}' to '{child_frame_id}'")
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_static_tf()
    except rospy.ROSInterruptException:
        pass