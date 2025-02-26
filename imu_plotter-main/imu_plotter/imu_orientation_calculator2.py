import rclpy
import math
from sensor_msgs.msg import Imu
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np

class IMUOrientationCalculator2(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator2')

        # ✅ IMU 데이터 구독
        self.imu_sub = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)

        # ✅ Roll, Pitch 발행하는 토픽 생성
        self.orientation_pub = self.create_publisher(Vector3, '/imu/orientation2', 10)   

        self.get_logger().info("✅ IMU Orientation Calculator 2 Node Started!")
    
    def imu_callback(self, msg):
        """ 가속도계를 사용하여 Roll, Pitch 계산 (가속도 기반) """
        
        self.get_logger().info("📥 IMU 데이터 수신됨!")  # ✅ 콜백 함수 실행 확인
        
        # ✅ 가속도 데이터 추출
        accel_x = msg.linear_acceleration.x
        accel_y = msg.linear_acceleration.y
        accel_z = msg.linear_acceleration.z

        # ✅ 데이터가 너무 작으면 오류 방지
        if abs(accel_x) < 1e-6 and abs(accel_y) < 1e-6 and abs(accel_z) < 1e-6:
            self.get_logger().warn("⚠️ IMU acceleration 값이 너무 작아서 계산을 스킵합니다.")
            return

        # ✅ Roll 및 Pitch 계산 (라디안)
        roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))
        # ✅ 메시지 생성 및 발행
        orientation2_msg = Vector3()
        orientation2_msg.x = math.degrees(roll)   # ✅ 라디안을 도(degree)로 변환
        orientation2_msg.y = math.degrees(pitch)
        orientation2_msg.z = 0.0  # ✅ Yaw는 계산하지 않음

        self.orientation_pub.publish(orientation2_msg)  # ✅ 올바른 변수명 사용

        self.get_logger().info(f"🚀 /imu/orientation2 발행 완료! Roll={orientation2_msg.x:.2f}°, Pitch={orientation2_msg.y:.2f}°")

def main():
    rclpy.init()
    node = IMUOrientationCalculator2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
import rclpy
import math
from sensor_msgs.msg import Imu
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np

class IMUOrientationCalculator2(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator2')

        # ✅ IMU 데이터 구독
        self.imu_sub = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)

        # ✅ Roll, Pitch 발행
        self.orientation_pub = self.create_publisher(Vector3, '/imu/orientation2', 10) 

        self.get_logger().info("IMU Orientation Calculator 2 Node Started!")
    
    def imu_callback(self, msg): 

        # 가속도 데이터 추출
        accel_x, accel_y, accel_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        # 데이터가 너무 작으면 오류 방지
        if ax**2 + ay**2 + az**2 < 1e-12:
            self.get_logger().warn("⚠️ IMU acceleration 값이 작아 계산 스킵")
            return 

        # Roll 및 Pitch 계산 (라디안)
        roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2))
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2))

        # 메시지 생성 및 발행
        orientation2_msg = Vector3()
        orientation2_msg.x = math.degrees(roll) # 라디안 -> 도
        orientation2_msg.y = math.degrees(pitch)
        orientation2_msg.z = 0.0  # Yaw 계산 X 

        self.orientation_pub.publish(orientation2_msg)  # 올바른 변수명 사용

        self.get_logger().info(f"""
🚀 가속도 발행 완료! 
Roll={orientation2_msg.x:.2f}°, 
Pitch={orientation2_msg.y:.2f}°
""")

def main():
    rclpy.init()
    node = IMUOrientationCalculator2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
