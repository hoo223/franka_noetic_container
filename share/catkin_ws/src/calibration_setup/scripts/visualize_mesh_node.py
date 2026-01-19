#!/usr/bin/env python3
import rospy
import os
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

class MeshVisualizer:
    def __init__(self, obj_name):
        self.obj_name = obj_name
        rospy.init_node(f'mesh_visualizer_{self.obj_name}', anonymous=True)

        # 1. 파라미터 서버에서 설정 가져오기
        # launch 파일에서 <param>으로 넣어준 값을 우선적으로 가져옵니다.
        self.mesh_path = rospy.get_param(f"~mesh_path", f"file:///root/share/assets/mesh_{self.obj_name}/{self.obj_name}.obj")
        self.mesh_scale = rospy.get_param("~mesh_scale", 1.0)
        self.frame_id = f"object_{self.obj_name}" # fp_node.py에서 발행하는 TF 이름

        # 2. Marker Publisher 생성
        self.marker_pub = rospy.Publisher(f'/visualization_marker_{self.obj_name}', Marker, queue_size=10)

        rospy.loginfo(f"[{self.obj_name}] Visualizer started. Target TF: {self.frame_id}")
        rospy.loginfo(f"[{self.obj_name}] Mesh Path: {self.mesh_path}")

    def publish_marker(self):
        """
        주기적으로 마커를 발행하여 RViz에 표시
        이미 TF(object_name)가 존재하므로, 마커는 그 좌표계의 원점(0,0,0)에 고정합니다.
        """
        marker = Marker()
        
        # [핵심] fp_node.py가 발행하는 TF를 부모 프레임으로 잡습니다.
        marker.header.frame_id = self.frame_id 
        marker.header.stamp = rospy.Time.now()
        
        marker.ns = f"mesh_{self.obj_name}"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        
        # 부모(TF) 좌표계 원점에 마커를 위치시킴
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 스케일 설정
        marker.scale.x = self.mesh_scale
        marker.scale.y = self.mesh_scale
        marker.scale.z = self.mesh_scale

        # 색상 설정 (구분을 위해 물체별로 다르게 설정 가능)
        if "part11" in self.obj_name:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = [0.1, 0.8, 0.1, 0.8] # 녹색
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = [0.8, 0.1, 0.1, 0.8] # 빨간색

        marker.mesh_resource = self.mesh_path
        marker.mesh_use_embedded_materials = True # 텍스처(UV)가 있다면 True

        # 마커의 수명 (0은 영구)
        marker.lifetime = rospy.Duration(0.5)

        self.marker_pub.publish(marker)

    def run(self):
        rate = rospy.Rate(10) # 10Hz로 마커 갱신
        while not rospy.is_shutdown():
            self.publish_marker()
            rate.sleep()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: rosrun your_pkg visualize_mesh_node.py [object_name]")
        sys.exit(1)
        
    obj_arg = sys.argv[1]
    try:
        node = MeshVisualizer(obj_arg)
        node.run()
    except rospy.ROSInterruptException:
        pass