'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class IMUOrientationCalculator(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator')

        # ✅ 자이로 데이터 구독
        self.imu_sub = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)

        # ✅ 계산된 roll, pitch 발행
        self.orientation_pub = self.create_publisher(Vector3, '/imu/orientation', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.roll = 0.0  # Roll (x축 회전)
        self.pitch = 0.0  # Pitch (y축 회전)

        self.get_logger().info("IMU Orientation Calculator Node Started!")

    def imu_callback(self, msg):
        """ 자이로스코프 데이터를 적분하여 roll, pitch 계산 """
        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return

        # ✅ 1. 자이로스코프 데이터 가져오기 (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y

        # ✅ 2. 시간 변화량 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # ✅ 3. Roll, Pitch 적분 (단순 오일러 적분)
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt

        # ✅ 4. Roll, Pitch 값 발행
        orientation_msg = Vector3()
        orientation_msg.x = np.degrees(self.roll)  # 라디안을 도(degree)로 변환
        orientation_msg.y = np.degrees(self.pitch)
        orientation_msg.z = 0.0  # Yaw는 계산하지 않음

        self.orientation_pub.publish(orientation_msg)

        # ✅ 5. 디버깅 로그 출력
        self.get_logger().info(f"""
📌 Gyro Data (rad/s): x={gyro_x:.3f}, y={gyro_y:.3f}
🕒 Time Step: {dt:.3f}s
🎯 Computed Orientation (deg): Roll={orientation_msg.x:.3f}, Pitch={orientation_msg.y:.3f}
        """)

def main():
    rclpy.init()
    node = IMUOrientationCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
# yaw 값 계산, 발행 추가 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class IMUOrientationCalculator(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator')

        # ✅ 자이로 데이터 구독
        self.imu_sub = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)

        # ✅ 계산된 roll, pitch, yaw 발행
        self.orientation_pub = self.create_publisher(Vector3, '/imu/orientation', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.roll = 0.0  # Roll (x축 회전)
        self.pitch = 0.0  # Pitch (y축 회전)
        self.yaw = 0.0 # yaw (z축 회전)

        self.get_logger().info("IMU Orientation Calculator Node Started!")

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return

        # 자이로스코프 데이터 가져오기 (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # 시간 변화량 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Roll, Pitch 적분 (단순 오일러 적분)
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt 

        # roll, Pitch, yaw 값 발행 
        orientation_msg = Vector3()
        orientation_msg.x = np.degrees(self.roll)  # 라디안을 도(degree)로 변환
        orientation_msg.y = np.degrees(self.pitch)
        orientation_msg.z = np.degrees(self.yaw)

        self.orientation_pub.publish(orientation_msg)

        # ✅ 5. 디버깅 로그 출력
        self.get_logger().info(f"""
📌Gyro Data (rad/s): 
    x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}
📌Computed Orientation (deg): 
    Roll={orientation_msg.x:.3f}, Pitch={orientation_msg.y:.3f}, Yaw={orientation_msg.z:.3f}
📌Time Step: {dt:.3f}s
        """)

def main():
    rclpy.init()
    node = IMUOrientationCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
# dt 값 보정 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np

class IMUOrientationCalculator(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator')

        # ✅ 자이로 데이터 구독
        self.imu_sub = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)

        # ✅ 계산된 roll, pitch, yaw 발행
        self.orientation_pub = self.create_publisher(Vector3, '/imu/orientation', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.roll = 0.0  # Roll (x축 회전)
        self.pitch = 0.0  # Pitch (y축 회전)
        self.yaw = 0.0 # yaw (z축 회전)

        self.get_logger().info("IMU Orientation Calculator Node Started!")

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return

        # 자이로스코프 데이터 가져오기 (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # 시간 변화량 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0 or dt > 0.1:
            dt = 0.02 # 50Hz 설정 (사실 100Hz로 들어오고 있음)

        # Roll, Pitch 적분 (단순 오일러 적분)
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt 

        # roll, Pitch, yaw 값 발행 
        orientation_msg = Vector3()
        orientation_msg.x = np.degrees(self.roll)  # 라디안을 도(degree)로 변환
        orientation_msg.y = np.degrees(self.pitch)
        orientation_msg.z = np.degrees(self.yaw)

        self.orientation_pub.publish(orientation_msg)

        # ✅ 5. 디버깅 로그 출력
        self.get_logger().info(f"""
📌 Gyro Data (rad/s): x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}
📌 Computed Orientation (deg): Roll={orientation_msg.x:.3f}, Pitch={orientation_msg.y:.3f}, Yaw={orientation_msg.z:.3f}
📌 Time Step: {dt:.3f}s
        """)

def main():
    rclpy.init()
    node = IMUOrientationCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R  # ✅ 쿼터니언 변환을 위한 파이썬 라이브러리

class IMUOrientationCalculator(Node):
    def __init__(self):
        super().__init__('imu_orientation_calculator')

        # ✅ 자이로 데이터 구독
        self.imu_sub = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)

        # ✅ Roll, Pitch, Yaw 발행
        self.orientation_pub = self.create_publisher(Vector3, '/imu/orientation', 10)

        # ✅ 쿼터니언 발행 (EKF 입력용)
        self.quaternion_pub = self.create_publisher(Quaternion, '/imu/quaternion', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.roll = 0.0  # Roll (x축 회전)
        self.pitch = 0.0  # Pitch (y축 회전)
        self.yaw = 0.0  # Yaw (z축 회전)

        self.get_logger().info("✅ IMU Orientation Calculator Node Started!")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """ Roll, Pitch, Yaw 값을 쿼터니언으로 변환하는 함수 """
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=False)  # ✅ RPY -> Quaternion
        qx, qy, qz, qw = r.as_quat()  # ✅ [qx, qy, qz, qw] 순서로 변환
        return qx, qy, qz, qw

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9  # ✅ 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return

        # ✅ 자이로스코프 데이터 가져오기 (rad/s)
        gyro_x = msg.angular_velocity.x
        gyro_y = msg.angular_velocity.y
        gyro_z = msg.angular_velocity.z

        # ✅ 시간 변화량 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0 or dt > 0.1:
            dt = 0.02  # ✅ 50Hz 설정 (사실 100Hz로 들어오고 있음)

        # ✅ Roll, Pitch, Yaw 적분 (단순 오일러 적분)
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt

        # ✅ roll, Pitch, yaw 값 발행
        orientation_msg = Vector3()
        orientation_msg.x = np.degrees(self.roll)  # ✅ 라디안을 도(degree)로 변환
        orientation_msg.y = np.degrees(self.pitch)
        orientation_msg.z = np.degrees(self.yaw)

        self.orientation_pub.publish(orientation_msg)

        # ✅ 쿼터니언 변환 및 발행 (EKF용)
        qx, qy, qz, qw = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)
        quaternion_msg = Quaternion()
        quaternion_msg.x = qx
        quaternion_msg.y = qy
        quaternion_msg.z = qz
        quaternion_msg.w = qw

        self.quaternion_pub.publish(quaternion_msg)

        # ✅ 디버깅 로그 출력
        self.get_logger().info(f"""
📌 Gyro Data (rad/s): x={gyro_x:.3f}, y={gyro_y:.3f}, z={gyro_z:.3f}
📌 Computed Orientation (deg): Roll={orientation_msg.x:.3f}, Pitch={orientation_msg.y:.3f}, Yaw={orientation_msg.z:.3f}
📌 Quaternion: x={qx:.3f}, y={qy:.3f}, z={qz:.3f}, w={qw:.3f}
📌 Time Step: {dt:.3f}s """)

def main():
    rclpy.init()
    node = IMUOrientationCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




