#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import sys
from collections import deque

def listen_moving_average():
    rospy.init_node('tcp_moving_avg_listener', anonymous=True)
    listener = tf.TransformListener()

    # 프레임 설정
    source_frame = '/panda_link0'
    target_frame = '/panda_hand_tcp'
    
    # 50개 샘플을 담을 큐 (Window Size = 50)
    window_size = 50
    x_queue = deque(maxlen=window_size)
    y_queue = deque(maxlen=window_size)
    z_queue = deque(maxlen=window_size)

    rospy.loginfo(f"Real-time Moving Average (Window: {window_size})")
    
    try:
        listener.waitForTransform(source_frame, target_frame, rospy.Time(0), rospy.Duration(10.0))
    except:
        rospy.logerr("Frame not found!")
        return

    rate = rospy.Rate(50) # 50Hz로 데이터 수집
    
    print("\n" + "="*60)
    print(f" {'AVERAGED X':^15} | {'AVERAGED Y':^15} | {'AVERAGED Z':^15}")
    print("="*60)

    while not rospy.is_shutdown():
        try:
            now = rospy.Time(0)
            (trans, rot) = listener.lookupTransform(source_frame, target_frame, now)
            
            # 큐에 새로운 샘플 추가
            x_queue.append(trans[0])
            y_queue.append(trans[1])
            z_queue.append(trans[2])

            # 큐가 꽉 찼을 때만 평균 계산 및 출력
            if len(x_queue) == window_size:
                avg_x = sum(x_queue) / window_size
                avg_y = sum(y_queue) / window_size
                avg_z = sum(z_queue) / window_size
                
                # \r를 사용하여 한 줄에서 계속 업데이트
                sys.stdout.write(f"\r {avg_x:15.6f} , {avg_y:15.6f} , {avg_z:15.6f}")
                sys.stdout.flush()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        listen_moving_average()
    except rospy.ROSInterruptException:
        pass