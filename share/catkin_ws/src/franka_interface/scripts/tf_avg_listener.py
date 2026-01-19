#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
import sys
import signal

class TfAverageListener:
    def __init__(self):
        rospy.init_node('tf_average_listener', anonymous=True)

        # 설정: 기준 프레임과 타겟 프레임
        self.target_frame = "world"           # 기준 (Parent)
        self.source_frame = "detected_object" # 타겟 (Child)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.positions = []
        self.quaternions = []
        
        # 종료 시그널 처리 (Ctrl+C 눌렀을 때 계산하도록)
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.loginfo(f"Listening for transform: {self.source_frame} -> {self.target_frame}")
        rospy.loginfo("Please PLAY the rosbag now...")

    def collect_data(self):
        rate = rospy.Rate(100.0) # 100Hz
        while not rospy.is_shutdown():
            try:
                # Time(0)은 버퍼에 있는 '가장 최신' 데이터를 가져옴
                # Bag 파일을 재생 중일 때는 이게 가장 끊김 없이 잘 작동함
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame, 
                    self.source_frame, 
                    rospy.Time(0), 
                    rospy.Duration(0.1)
                )

                # 데이터 저장
                self.positions.append([
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                ])

                self.quaternions.append([
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w
                ])
                
                # 진행상황 표시 (매 100개 샘플마다)
                if len(self.positions) % 100 == 0:
                    sys.stdout.write(f"\rCollecting samples... Current count: {len(self.positions)}")
                    sys.stdout.flush()

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # 아직 TF 트리가 연결되지 않았거나 데이터가 안 들어올 때
                continue
            
            rate.sleep()

    def signal_handler(self, sig, frame):
        self.calculate_and_print()
        sys.exit(0)

    def calculate_and_print(self):
        if not self.positions:
            print("\n\n[Error] 수집된 데이터가 없습니다. Bag 파일에 해당 TF 연결고리가 있는지 확인하세요.")
            return

        # 1. 위치 평균
        avg_pos = np.mean(np.array(self.positions), axis=0)

        # 2. 쿼터니언 평균 및 정규화
        avg_quat = np.mean(np.array(self.quaternions), axis=0)
        norm = np.linalg.norm(avg_quat)
        if norm == 0:
            avg_quat = [0, 0, 0, 1]
        else:
            avg_quat = avg_quat / norm

        print("\n" + "="*50)
        print(f" FINAL RESULT (World Frame Average)")
        print(f" Samples: {len(self.positions)}")
        print("-" * 50)
        print(" Average Position (x, y, z):")
        print(f" [{avg_pos[0]:.7f}, {avg_pos[1]:.7f}, {avg_pos[2]:.7f}]")
        print("-" * 50)
        print(" Average Orientation (x, y, z, w):")
        print(f" [{avg_quat[0]:.7f}, {avg_quat[1]:.7f}, {avg_quat[2]:.7f}, {avg_quat[3]:.7f}]")
        print("="*50 + "\n")

if __name__ == '__main__':
    listener = TfAverageListener()
    try:
        listener.collect_data()
    except rospy.ROSInterruptException:
        pass