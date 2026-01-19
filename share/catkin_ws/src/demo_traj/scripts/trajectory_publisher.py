#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

def talker():
    # 노드 초기화
    rospy.init_node('trajectory_publisher', anonymous=True)
    
    # 퍼블리셔 선언: /object_path 토픽으로 Path 메시지 전송
    path_pub = rospy.Publisher('/object_path', Path, queue_size=10)
    
    # 루프 주기 설정 (10Hz)
    rate = rospy.Rate(10)
    
    # Path 메시지 객체 생성
    path = Path()
    path.header.frame_id = "map" # RViz의 Fixed Frame과 일치해야 함

    count = 0
    while not rospy.is_shutdown():
        # 현재 시간 업데이트
        path.header.stamp = rospy.Time.now()
        
        # 새로운 Pose 생성 (이 부분에 나중에 파일 데이터가 들어갑니다)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        # 예시 데이터: 나선형 궤적
        pose.pose.position.x = 1.0 * math.cos(count * 0.1)
        pose.pose.position.y = 1.0 * math.sin(count * 0.1)
        pose.pose.position.z = count * 0.01
        pose.pose.orientation.w = 1.0
        
        # Path에 Pose 추가
        path.poses.append(pose)
        
        # 메시지 발행
        path_pub.publish(path)
        
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass