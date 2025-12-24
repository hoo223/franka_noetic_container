#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import math

def main():
    rospy.init_node('world_to_base_broadcaster')

    # ==========================================
    # 1. 실측 데이터 입력 (단위: mm, degree)
    # ==========================================
    # measured_x_mm = 128.39
    # measured_y_mm = 116.023 + 2
    # measured_z_mm = 0.0      # (주의) 어댑터 플레이트 두께가 있다면 여기에 입력하세요!
    # measured_yaw_deg = 43.377 + 1

    measured_x_mm = 128.084
    measured_y_mm = 114.584
    measured_z_mm = 0.0      # (주의) 어댑터 플레이트 두께가 있다면 여기에 입력하세요!
    measured_yaw_deg = 43.404 + 0.9

    # ==========================================
    # 2. 단위 변환 (ROS 표준: meter, radian)
    # ==========================================
    x_m = measured_x_mm / 1000.0
    y_m = measured_y_mm / 1000.0
    z_m = measured_z_mm / 1000.0
    
    yaw_rad = math.radians(measured_yaw_deg) # degree -> radian 변환

    # ==========================================
    # 3. TF 메시지 생성
    # ==========================================
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"       # 부모 프레임
    static_transformStamped.child_frame_id = "panda_link0"  # 자식 프레임 (로봇 베이스)

    # Translation 적용
    static_transformStamped.transform.translation.x = x_m
    static_transformStamped.transform.translation.y = y_m
    static_transformStamped.transform.translation.z = z_m

    # Rotation (Euler -> Quaternion) 적용
    # 평면 위에 놓여 있으므로 Roll, Pitch는 0으로 가정
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw_rad)

    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    # ==========================================
    # 4. TF 송출
    # ==========================================
    broadcaster.sendTransform(static_transformStamped)
    
    rospy.loginfo("=== World -> Panda Base TF Published ===")
    rospy.loginfo(f"Translation: x={x_m:.4f}m, y={y_m:.4f}m, z={z_m:.4f}m")
    rospy.loginfo(f"Rotation: {measured_yaw_deg} deg ({yaw_rad:.4f} rad)")

    rospy.spin()

if __name__ == '__main__':
    main()