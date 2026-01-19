#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
import os

# ==========================================
# [사용자 설정 구역]
# ==========================================

# 1. CAD 파일 경로 (반드시 절대 경로 또는 package:// 경로 사용)
# 예: "file:///home/user/catkin_ws/src/my_pkg/meshes/optical_table.stl"
MESH_PATH = "file:///root/share/assets/mesh_optical_breadboard/optical_breadboard.obj" 

# 2. 파일 단위 스케일 (중요!)
# CAD가 mm 단위로 그려졌으면 0.001, m 단위면 1.0
MESH_SCALE = 1 

# 3. 설치 위치 (x, y, z) [미터]
# 브레드보드 중심(또는 원점)이 놓일 월드 좌표
BOARD_POS = [0.0, 0.0, 0.0] 

# 4. 회전 (roll, pitch, yaw) [라디안]
# 필요 시 수정 (보통 0,0,0)
BOARD_RPY = [0.0, 0.0, 0.0]

# 5. 색상 적용 여부
# True: CAD 파일 본연의 색상/텍스처 사용 (OBJ/DAE 등)
# False: 아래 설정한 단색으로 덮어쓰기 (STL 등)
USE_EMBEDDED_COLOR = False

# (False일 경우 적용할 색상: R, G, B, Alpha) - 검은색 무광 느낌
CUSTOM_COLOR = [0.15, 0.15, 0.15, 1.0] 
# ==========================================

import tf.transformations


#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
import tf.transformations

# ... [사용자 설정 구역 동일] ...

def main():
    rospy.init_node('optical_breadboard_visualizer')
    # latched=True 설정이 핵심입니다!
    pub = rospy.Publisher('optical_breadboard_marker', Marker, queue_size=1, latch=True)
    
    marker = Marker()
    marker.header.frame_id = "world"
    marker.ns = "breadboard_mesh"
    marker.id = 0
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    
    # [설정 부분 동일]
    marker.mesh_resource = MESH_PATH
    marker.mesh_use_embedded_materials = USE_EMBEDDED_COLOR
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = BOARD_POS
    q = tf.transformations.quaternion_from_euler(BOARD_RPY[0], BOARD_RPY[1], BOARD_RPY[2])
    marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w = q
    marker.scale.x = marker.scale.y = marker.scale.z = MESH_SCALE
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = CUSTOM_COLOR

    # 중요: lifetime을 0으로 설정 (0은 영구 유지를 의미)
    marker.lifetime = rospy.Duration(0)

    # 딱 한 번 발행
    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)
    
    rospy.loginfo(f"Mesh Marker published once. Node idling...")
    
    # 노드 유지 (CPU 점유율 0%)
    rospy.spin()

if __name__ == '__main__':
    main()

# def main():
#     rospy.init_node('optical_breadboard_visualizer')
#     pub = rospy.Publisher('optical_breadboard_marker', Marker, queue_size=10)
    
#     # 마커 설정
#     marker = Marker()
#     marker.header.frame_id = "world"
#     marker.ns = "breadboard_mesh"
#     marker.id = 0
#     marker.type = Marker.MESH_RESOURCE
#     marker.action = Marker.ADD
    
#     # 1. 파일 경로 입력
#     marker.mesh_resource = MESH_PATH
#     marker.mesh_use_embedded_materials = USE_EMBEDDED_COLOR

#     # 2. 위치 설정
#     marker.pose.position.x = BOARD_POS[0]
#     marker.pose.position.y = BOARD_POS[1]
#     marker.pose.position.z = BOARD_POS[2]

#     # 3. 회전 설정 (Euler -> Quaternion 변환)
#     q = tf.transformations.quaternion_from_euler(BOARD_RPY[0], BOARD_RPY[1], BOARD_RPY[2])
#     marker.pose.orientation.x = q[0]
#     marker.pose.orientation.y = q[1]
#     marker.pose.orientation.z = q[2]
#     marker.pose.orientation.w = q[3]

#     # 4. 크기(Scale) 설정
#     marker.scale.x = MESH_SCALE
#     marker.scale.y = MESH_SCALE
#     marker.scale.z = MESH_SCALE

#     # 5. 색상 설정 (Embedded Color가 False일 때만 적용됨)
#     marker.color.r = CUSTOM_COLOR[0]
#     marker.color.g = CUSTOM_COLOR[1]
#     marker.color.b = CUSTOM_COLOR[2]
#     marker.color.a = CUSTOM_COLOR[3]

#     # 주기적으로 Publish
#     rate = rospy.Rate(1.0) # 1Hz
#     rospy.loginfo(f"Publishing CAD Mesh: {MESH_PATH}")
    
#     while not rospy.is_shutdown():
#         marker.header.stamp = rospy.Time.now()
#         pub.publish(marker)
#         rate.sleep()

# if __name__ == '__main__':
#     try:
#         main()
#     except rospy.ROSInterruptException:
#         pass

    