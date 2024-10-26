import rclpy
from rclpy.node import Node
import rosbag2_py
from geometry_msgs.msg import PoseStamped
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import csv
import threading
import sys
import os
import time
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import matplotlib.pyplot as plt
import pandas as pd

# ROS2 Python 패키지 경로 추가
ros_python_path = '/opt/ros/humble/lib/python3.10/site-packages'  # Python 버전에 따라 경로 수정 필요
sys.path.append(ros_python_path)

class GlobalPathPublisher(Node):
    def __init__(self, bag_file):
        super().__init__('global_path_publisher')
        
        # 선택한 bag 파일 이름을 기반으로 CSV 파일 이름 설정
        base_name = os.path.splitext(os.path.basename(bag_file))[0]
        self.csv_file_path = f'/home/wondoyeon/ros2_ws/dataset/{base_name}_utm_data_log.csv'

        # CSV 파일 초기화 및 헤더 작성
        with open(self.csv_file_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['sec', 'nanosec', 'position_x', 'position_y', 'position_z'])

        self.bag_file = bag_file  # bag 파일 경로 저장
        self.data_acquisition_complete = False  # 데이터 수집 완료 플래그
        self.initialize_reader()

        # 데이터 취득 스레드 시작
        self.data_thread = threading.Thread(target=self.read_bag_data)
        self.data_thread.start()

    def initialize_reader(self):
        # SequentialReader를 초기화하고 열기
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_file, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)

        topic_types = self.reader.get_all_topics_and_types()
        self.type_map = {topic.name: topic.type for topic in topic_types}

        self.msg_type = None
        for topic_name, topic_type in self.type_map.items():
            if topic_name == '/utm':
                self.msg_type = get_message(topic_type)
                break

        if self.msg_type is None:
            self.get_logger().error('Could not find /utm topic in bag file.')
            rclpy.shutdown()

    def get_total_messages(self):
        # 새로운 SequentialReader 인스턴스로 총 메시지 개수 확인
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_file, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader.open(storage_options, converter_options)
        
        count = 0
        while reader.has_next():
            reader.read_next()
            count += 1
        
        return count

    def read_bag_data(self):
        total_messages = self.get_total_messages()
        start_time = time.time()
        processed_messages = 0

        while self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            if topic == '/utm':
                msg = deserialize_message(data, self.msg_type)
                # self.get_logger().info('Retrieved a PoseStamped message from bag.')

                # 메시지 데이터에서 시간과 위치 좌표 추출하여 CSV 파일에 기록
                with open(self.csv_file_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    sec = msg.header.stamp.sec
                    nanosec = msg.header.stamp.nanosec
                    position_x = msg.pose.position.x
                    position_y = msg.pose.position.y
                    position_z = msg.pose.position.z
                    writer.writerow([sec, nanosec, position_x, position_y, position_z])

                # 남은 시간 계산
                processed_messages += 1
                elapsed_time = time.time() - start_time
                avg_time_per_msg = elapsed_time / processed_messages
                remaining_time = avg_time_per_msg * (total_messages - processed_messages)
                
                if processed_messages % 10 == 0:  # 10개의 메시지마다 남은 시간 출력
                    print(f"진행도: {processed_messages}/{total_messages}, 남은 예상 시간: {remaining_time:.2f}초")

        self.data_acquisition_complete = True
        self.get_logger().info('Finished reading all data from bag file.')

def plot_path(csv_file_path):
    # CSV 파일에서 데이터를 읽어와 경로를 플로팅
    data = pd.read_csv(csv_file_path)
    plt.figure(figsize=(10, 6))
    plt.plot(data['position_x'], data['position_y'], marker='o', linestyle='-', color='b')
    plt.title('Path Plot from UTM Data')
    plt.xlabel('Position X')
    plt.ylabel('Position Y')
    plt.grid()
    plt.show()

def main(args=None):
    # Tk 인터페이스 숨기기
    Tk().withdraw()

    # 파일 선택 창 열기
    bag_file = askopenfilename(title="Select Bag File", filetypes=[("ROS2 Bag Files", "*.db3")])
    
    if not bag_file:
        print("No bag file selected. Exiting.")
        return

    rclpy.init(args=args)
    node = GlobalPathPublisher(bag_file)

    # ROS 실행 (데이터 수집 완료 시 종료)
    while not node.data_acquisition_complete:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()  # 노드 삭제
    rclpy.shutdown()

    # 데이터 수집이 완료된 후 플로팅 함수 호출
    plot_path(node.csv_file_path)

if __name__ == '__main__':
    main()
