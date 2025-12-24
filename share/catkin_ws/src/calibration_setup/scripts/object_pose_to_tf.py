#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker  # Marker 메시지 추가

# ==========================================
# [사용자 설정]
# ==========================================
# 1. Mesh 파일 경로 (file:// 필수)
#MESH_PATH = "file:///root/share/assets/mesh_part11/part11_with_uv.obj"
MESH_PATH = "file:///root/share/assets/mesh_part1/part1_with_uv.obj"

# 2. 스케일 (mm 단위 파일이면 0.001, m 단위면 1.0)
MESH_SCALE = 1 

# 3. 색상 (R, G, B, A)
MESH_COLOR = [0.3, 0.3, 0.3, 1.0] # 회색
# ==========================================

class ObjectPoseToTF:
    def __init__(self):
        # 1. 노드 초기화
        rospy.init_node('object_pose_to_tf_broadcaster')
        
        # 2. TF Broadcaster 생성
        self.br = tf2_ros.TransformBroadcaster()

        # 3. Marker Publisher 생성 (추가됨)
        self.marker_pub = rospy.Publisher('part11_marker', Marker, queue_size=10)
        
        # 4. Subscriber 생성
        self.sub = rospy.Subscriber('/object_pose', PoseStamped, self.pose_callback)
        
        # 새로 만들 물체의 좌표계 이름 (Child Frame)
        self.child_frame_id = "detected_object"
        
        rospy.loginfo("Started object pose to TF & Marker node.")
        rospy.loginfo("Subscribing to: /object_pose")

    def pose_callback(self, msg):
        """
        PoseStamped 메시지를 받을 때마다 -> TF 발행 + Marker 발행
        """
        try:
            # -------------------------------------------------
            # [A] TF Broadcast (기존 로직)
            # -------------------------------------------------
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = msg.header.frame_id # Parent (Camera etc.)
            t.child_frame_id = self.child_frame_id  # Child (detected_object)

            # Translation & Rotation 복사
            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z
            t.transform.rotation.x = msg.pose.orientation.x
            t.transform.rotation.y = msg.pose.orientation.y
            t.transform.rotation.z = msg.pose.orientation.z
            t.transform.rotation.w = msg.pose.orientation.w

            self.br.sendTransform(t)

            # -------------------------------------------------
            # [B] Marker Publish (추가된 로직)
            # -------------------------------------------------
            marker = Marker()
            
            # 중요: 마커를 'detected_object' 프레임에 매달면, 
            # 마커 자체의 좌표는 (0,0,0)으로 둬도 TF를 따라다닙니다.
            marker.header.frame_id = self.child_frame_id 
            marker.header.stamp = msg.header.stamp
            
            marker.ns = "detected_mesh"
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            
            # 위치/회전: 부모 프레임(detected_object) 자체가 이미 이동했으므로, 
            # 여기서는 0으로 설정해야 물체 원점에 딱 붙습니다.
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            # 스케일
            marker.scale.x = MESH_SCALE
            marker.scale.y = MESH_SCALE
            marker.scale.z = MESH_SCALE

            # 색상
            marker.color.r = MESH_COLOR[0]
            marker.color.g = MESH_COLOR[1]
            marker.color.b = MESH_COLOR[2]
            marker.color.a = MESH_COLOR[3]

            # Mesh 경로
            marker.mesh_resource = MESH_PATH
            
            # (옵션) OBJ 파일의 텍스처를 쓰고 싶으면 True, 아니면 False
            marker.mesh_use_embedded_materials = False 

            self.marker_pub.publish(marker)

        except Exception as e:
            rospy.logwarn("Failed to broadcast TF or Marker: %s", str(e))

if __name__ == '__main__':
    try:
        node = ObjectPoseToTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python3
# import rospy
# import tf2_ros
# from geometry_msgs.msg import PoseStamped, TransformStamped

# class ObjectPoseToTF:
#     def __init__(self):
#         # 1. 노드 초기화
#         rospy.init_node('object_pose_to_tf_broadcaster')
        
#         # 2. TF Broadcaster 생성
#         self.br = tf2_ros.TransformBroadcaster()
        
#         # 3. Subscriber 생성
#         # 토픽 이름: /object_pose, 타입: PoseStamped
#         self.sub = rospy.Subscriber('/object_pose', PoseStamped, self.pose_callback)
        
#         # 새로 만들 물체의 좌표계 이름 (Child Frame)
#         self.child_frame_id = "detected_object"
        
#         rospy.loginfo("Started object pose to TF broadcaster node.")
#         rospy.loginfo("Subscribing to: /object_pose")
#         rospy.loginfo("Broadcasting TF child frame: %s", self.child_frame_id)

#     def pose_callback(self, msg):
#         """
#         PoseStamped 메시지를 받을 때마다 호출되는 함수
#         """
#         try:
#             # 4. TransformStamped 메시지 생성
#             t = TransformStamped()

#             # (A) 헤더 설정
#             # TF의 시간은 메시지가 생성된 시간(stamp)을 따라가는 것이 동기화에 유리합니다.
#             t.header.stamp = msg.header.stamp
            
#             # Parent Frame: PoseStamped 메시지 안에 있는 기준 좌표계 (예: camera_color_optical_frame)
#             t.header.frame_id = msg.header.frame_id
            
#             # Child Frame: 우리가 띄울 물체의 좌표계 이름
#             t.child_frame_id = self.child_frame_id

#             # (B) Translation (위치) 복사
#             t.transform.translation.x = msg.pose.position.x
#             t.transform.translation.y = msg.pose.position.y
#             t.transform.translation.z = msg.pose.position.z

#             # (C) Rotation (회전) 복사
#             t.transform.rotation.x = msg.pose.orientation.x
#             t.transform.rotation.y = msg.pose.orientation.y
#             t.transform.rotation.z = msg.pose.orientation.z
#             t.transform.rotation.w = msg.pose.orientation.w

#             # 5. TF 송출
#             self.br.sendTransform(t)

#         except Exception as e:
#             rospy.logwarn("Failed to broadcast TF: %s", str(e))

# if __name__ == '__main__':
#     try:
#         node = ObjectPoseToTF()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass