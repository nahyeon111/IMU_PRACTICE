'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from tf_transformations import quaternion_matrix  # ✅ 올바른 임포트 방식

class IMUGravityProcessor(Node):
    def __init__(self):
        super().__init__('imu_gravity_processor')

        # ✅ 기존 IMU 데이터 구독
        self.imu_sub = self.create_subscription(
            Imu, '/android/imu', self.imu_callback, 10)

        # ✅ 기존 중력 가속도 유지하는 토픽 발행
        self.gravity_pub = self.create_publisher(
            Vector3, '/imu/with_gravity', 10)

        # ✅ 비중력 가속도 (중력 제거 후) 발행
        self.non_gravity_pub = self.create_publisher(
            Vector3, '/imu/non_gravity_acceleration', 10)

        self.get_logger().info("IMU Gravity Processor Node Started!")

    def imu_callback(self, msg):
        """ IMU 데이터를 받아 기존 중력 가속도를 유지하면서 비중력 가속도 계산 """
        # ✅ 1. 측정된 가속도 데이터 가져오기
        measured_accel = np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])

        # ✅ 2. 쿼터니언을 회전 행렬로 변환
        quaternion = np.array([msg.orientation.x,
                               msg.orientation.y,
                               msg.orientation.z,
                               msg.orientation.w])

        # ✅ 3. 회전 행렬 변환
        rotation_matrix = quaternion_matrix(quaternion)[:3, :3]

        # ✅ 4. 중력 벡터 변환 (센서 기준으로 변환)
        # 휴대폰이 뒤집혔을 때 자동으로 중력 방향 조정
        gravity_earth_frame = np.array([0, 0, 9.81])  # 일반적으로 ENU 기준 (필요하면 -9.81로 변경)
        
        # 변환 행렬 적용하여 중력 벡터를 센서 좌표계로 변환
        gravity_in_sensor_frame = rotation_matrix @ gravity_earth_frame

        # ✅ 5. 비중력 가속도 계산 (센서에서 측정된 값 - 중력)
        non_gravity_accel = measured_accel - gravity_in_sensor_frame

        # ✅ 6. 기존 중력 가속도 유지하여 발행
        gravity_msg = Vector3()
        gravity_msg.x = float(gravity_in_sensor_frame[0])
        gravity_msg.y = float(gravity_in_sensor_frame[1])
        gravity_msg.z = float(gravity_in_sensor_frame[2])
        self.gravity_pub.publish(gravity_msg)

        # ✅ 7. 비중력 가속도 발행
        non_grav_msg = Vector3()
        non_grav_msg.x = float(non_gravity_accel[0])
        non_grav_msg.y = float(non_gravity_accel[1])
        non_grav_msg.z = float(non_gravity_accel[2])
        self.non_gravity_pub.publish(non_grav_msg)

        # ✅ 8. 디버깅 로그 추가 (센서 방향 확인)
        self.get_logger().info(f"""
📌 Original Acceleration (with gravity): x={measured_accel[0]:.3f}, y={measured_accel[1]:.3f}, z={measured_accel[2]:.3f}
🌀 Rotation Matrix:
{rotation_matrix}
🌍 Transformed Gravity (sensor frame): x={gravity_msg.x:.3f}, y={gravity_msg.y:.3f}, z={gravity_msg.z:.3f}
🚀 Non-Gravity Acceleration: x={non_grav_msg.x:.3f}, y={non_grav_msg.y:.3f}, z={non_grav_msg.z:.3f}
        """)

def main():
    rclpy.init()
    node = IMUGravityProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
# orientation 값을 받아와야 하는 문제가 있는데,좀 귀찮다
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np
from transforms3d.quaternions import quat2mat 
# from tf_transformations import quaternion_matrix -> ros1의 tf 패키지에 포함?
# pip install transforms3d

class IMUGravityProcessor(Node):
    def __init__(self):
        super().__init__('imu_gravity_processor')

        # ✅ 기존 IMU 데이터 구독
        self.imu_sub = self.create_subscription(
            Imu, '/android/imu', self.imu_callback, 10)

        # ✅ 기존 중력 가속도 유지하는 토픽 발행
        self.gravity_pub = self.create_publisher(
            Vector3, '/imu/with_gravity', 10)

        # ✅ 비중력 가속도 (중력 제거 후) 발행
        self.non_gravity_pub = self.create_publisher(
            Vector3, '/imu/non_gravity_acceleration', 10)
        
        # ✅ 속도 발행 (비중력 가속도 적분)
        self.velocity_pub = self.create_publisher(Vector3, '/imu/velocity', 10)

        # ✅ 상대 위치 발행 (속도 적분)
        self.position_pub = self.create_publisher(Vector3, '/imu/position', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.velocity = np.array([0.0, 0.0, 0.0])  # 속도 초기화
        self.position = np.array([0.0, 0.0, 0.0])  # 위치 초기화

        self.get_logger().info("IMU Gravity Processor Node Started!")

    def imu_callback(self, msg):
        """ IMU 데이터를 받아 비중력 계산 + 속도, 위치 """

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        # ✅ dt (시간 변화량) 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # ✅ dt 값 보정 (비정상적으로 크거나 0 이하일 경우)
        if dt <= 0 or dt > 0.1:
            self.get_logger().warn(f"⚠️ 비정상적인 dt 감지: {dt:.3f}s → 기본값 0.02s로 보정")
            dt = 0.02  # 50Hz 기준 (현재 100Hz)

        # ✅ 측정된 가속도 데이터 가져오기
        measured_accel = np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])

        # ✅ 쿼터니언을 회전 행렬로 변환 (수정함)
        quaternion = np.array([msg.orientation.w,  
                               msg.orientation.x,
                               msg.orientation.y,
                               msg.orientation.z]) # transforms3d는 (w, x, y, z) 순서 사용
        
        quaternion = quaternion / np.linalg.norm(quaternion)  # 정규화
        rotation_matrix = quat2mat(quaternion) # 3x3 회전 행렬 생성


        # ✅ 중력 벡터 변환 (센서 기준으로 변환)
        gravity_earth_frame = np.array([0, 0, 9.81])  # ENU 기준 (필요하면 -9.81로 변경)
        gravity_in_sensor_frame = rotation_matrix @ gravity_earth_frame # 변환 행렬 적용하여 중력 벡터를 센서 좌표계로 변환

        # ✅ 비중력 가속도 계산
        non_gravity_accel = measured_accel - gravity_in_sensor_frame

        # ✅ 속도 계산 (비중력 가속도를 적분)
        self.velocity += non_gravity_accel * dt

        # ✅ 위치 계산 (속도를 적분)
        self.position += self.velocity * dt

        # ✅ 중력 가속도 유지 발행
        gravity_msg = Vector3()
        gravity_msg.x = float(gravity_in_sensor_frame[0])
        gravity_msg.y = float(gravity_in_sensor_frame[1])
        gravity_msg.z = float(gravity_in_sensor_frame[2])
        self.gravity_pub.publish(gravity_msg)

        # ✅ 비중력 가속도 발행
        non_grav_msg = Vector3()
        non_grav_msg.x = float(non_gravity_accel[0])
        non_grav_msg.y = float(non_gravity_accel[1])
        non_grav_msg.z = float(non_gravity_accel[2])
        self.non_gravity_pub.publish(non_grav_msg)

        # ✅ 속도 발행
        velocity_msg = Vector3()
        velocity_msg.x = float(self.velocity[0])
        velocity_msg.y = float(self.velocity[1])
        velocity_msg.z = float(self.velocity[2])
        self.velocity_pub.publish(velocity_msg)

        # ✅ 위치 발행
        position_msg = Vector3()
        position_msg.x = float(self.position[0])
        position_msg.y = float(self.position[1])
        position_msg.z = float(self.position[2])
        self.position_pub.publish(position_msg)

        # ✅ 8. 디버깅 로그 추가 (센서 방향 확인)
        self.get_logger().info(f"""
🌍 Original Acceleration (with gravity): x={measured_accel[0]:.3f}, y={measured_accel[1]:.3f}, z={measured_accel[2]:.3f}
🌍 Rotation Matrix:
{rotation_matrix}
🌍 Transformed Gravity (sensor frame): x={gravity_msg.x:.3f}, y={gravity_msg.y:.3f}, z={gravity_msg.z:.3f}
🌍 Non-Gravity Acceleration: x={non_grav_msg.x:.3f}, y={non_grav_msg.y:.3f}, z={non_grav_msg.z:.3f}
🌍 Velocity: x={velocity_msg.x:.3f}, y={velocity_msg.y:.3f}, z={velocity_msg.z:.3f}
🌍 Position: x={position_msg.x:.3f}, y={position_msg.y:.3f}, z={position_msg.z:.3f}
🕒 Time Step: {dt:.3f}s
🌀 Quaternion: w={quaternion[0]:.3f}, x={quaternion[1]:.3f}, y={quaternion[2]:.3f}, z={quaternion[3]:.3f}
        """)

def main():
    rclpy.init()
    node = IMUGravityProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''
'''
# 넘파이만 이용
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import numpy as np


# 쿼터니언을 회전 행렬로 변환하는 함수 (넘파이 사용)
def quaternion_to_rotation_matrix(q):
    """ 쿼터니언을 회전 행렬로 변환 """
    w, x, y, z = q
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])

class IMUGravityProcessor(Node):
    def __init__(self):
        super().__init__('imu_gravity_processor')

        # ✅ 기존 IMU 데이터 구독
        self.imu_sub = self.create_subscription(
            Imu, '/android/imu', self.imu_callback, 10)

        # ✅ 기존 중력 가속도 유지하는 토픽 발행
        self.gravity_pub = self.create_publisher(
            Vector3, '/imu/with_gravity', 10)

        # ✅ 비중력 가속도 (중력 제거 후) 발행
        self.non_gravity_pub = self.create_publisher(
            Vector3, '/imu/non_gravity_acceleration', 10)
        
        # ✅ 속도 발행 (비중력 가속도 적분)
        self.velocity_pub = self.create_publisher(Vector3, '/imu/velocity', 10)

        # ✅ 상대 위치 발행 (속도 적분)
        self.position_pub = self.create_publisher(Vector3, '/imu/position', 10)

        # ✅ 초기값 설정
        self.prev_time = None
        self.velocity = np.array([0.0, 0.0, 0.0])  # 속도 초기화
        self.position = np.array([0.0, 0.0, 0.0])  # 위치 초기화

        self.get_logger().info("IMU Gravity Processor Node Started!")

    def imu_callback(self, msg):
        """ IMU 데이터를 받아 비중력 계산 + 속도, 위치 """

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        # 첫 데이터 수신 시 시간 초기화
        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        # ✅ dt (시간 변화량) 계산
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # ✅ dt 값 보정 (비정상적으로 크거나 0 이하일 경우)
        if dt <= 0 or dt > 0.1:
            self.get_logger().warn(f"⚠️ 비정상적인 dt 감지: {dt:.3f}s → 기본값 0.02s로 보정")
            dt = 0.02  # 50Hz 기준 (현재 100Hz)

        # ✅ 측정된 가속도 데이터 가져오기
        measured_accel = np.array([msg.linear_acceleration.x,
                                   msg.linear_acceleration.y,
                                   msg.linear_acceleration.z])

        # ✅ 쿼터니언을 회전 행렬로 변환 (수정함)
        quaternion = np.array([msg.orientation.w,  
                               msg.orientation.x,
                               msg.orientation.y,
                               msg.orientation.z])
        
        quaternion = quaternion / np.linalg.norm(quaternion)  # 정규화
        rotation_matrix = quaternion_to_rotation_matrix(quaternion)


        # ✅ 중력 벡터 변환 (센서 기준으로 변환)
        gravity_earth_frame = np.array([0, 0, 9.81])  # ENU 기준 (필요하면 -9.81로 변경)
        gravity_in_sensor_frame = rotation_matrix @ gravity_earth_frame # 변환 행렬 적용하여 중력 벡터를 센서 좌표계로 변환

        # ✅ 비중력 가속도 계산
        non_gravity_accel = measured_accel - gravity_in_sensor_frame

        # ✅ 속도 계산 (비중력 가속도를 적분)
        self.velocity += non_gravity_accel * dt

        # ✅ 위치 계산 (속도를 적분)
        self.position += self.velocity * dt

        # ✅ 중력 가속도 유지 발행
        gravity_msg = Vector3()
        gravity_msg.x = float(gravity_in_sensor_frame[0])
        gravity_msg.y = float(gravity_in_sensor_frame[1])
        gravity_msg.z = float(gravity_in_sensor_frame[2])
        self.gravity_pub.publish(gravity_msg)

        # ✅ 비중력 가속도 발행
        non_grav_msg = Vector3()
        non_grav_msg.x = float(non_gravity_accel[0])
        non_grav_msg.y = float(non_gravity_accel[1])
        non_grav_msg.z = float(non_gravity_accel[2])
        self.non_gravity_pub.publish(non_grav_msg)

        # ✅ 속도 발행
        velocity_msg = Vector3()
        velocity_msg.x = float(self.velocity[0])
        velocity_msg.y = float(self.velocity[1])
        velocity_msg.z = float(self.velocity[2])
        self.velocity_pub.publish(velocity_msg)

        # ✅ 위치 발행
        position_msg = Vector3()
        position_msg.x = float(self.position[0])
        position_msg.y = float(self.position[1])
        position_msg.z = float(self.position[2])
        self.position_pub.publish(position_msg)

        # ✅ 8. 디버깅 로그 추가 (센서 방향 확인)
        self.get_logger().info(f"""
🌍 Original Acceleration (with gravity): x={measured_accel[0]:.3f}, y={measured_accel[1]:.3f}, z={measured_accel[2]:.3f}
🌍 Rotation Matrix:
{rotation_matrix}
🌍 Transformed Gravity (sensor frame): x={gravity_msg.x:.3f}, y={gravity_msg.y:.3f}, z={gravity_msg.z:.3f}
🌍 Non-Gravity Acceleration: x={non_grav_msg.x:.3f}, y={non_grav_msg.y:.3f}, z={non_grav_msg.z:.3f}
🌍 Velocity: x={velocity_msg.x:.3f}, y={velocity_msg.y:.3f}, z={velocity_msg.z:.3f}
🌍 Position: x={position_msg.x:.3f}, y={position_msg.y:.3f}, z={position_msg.z:.3f}
🕒 Time Step: {dt:.3f}s
🌀 Quaternion: w={quaternion[0]:.3f}, x={quaternion[1]:.3f}, y={quaternion[2]:.3f}, z={quaternion[3]:.3f}
        """)

def main():
    rclpy.init()
    node = IMUGravityProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''


# orientation 데이터 받아오지 않고 VER.
# roll, pitch, yaw 데이터 필요
# 가속도, 자이로, 지자기계에서 뽑은 상태값에서 칼만필터 적용 후 보정된 roll, pitch, yaw 사용할 것
# 계산 처리 과정 간소화, 오차 줄이기 위함 
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import numpy as np

# 중력제거 클래스
class GravityRemover:
    def __init__(self):
        self.orientation = np.zeros(3)  # Roll, Pitch, Yaw (초기값)

    def update_orientation(self, gyro, dt, mag_x, mag_y):
        """자이로 및 지자기 데이터를 이용하여 Roll, Pitch, Yaw 업데이트"""
        self.orientation[0] += gyro[0] * dt  # Roll
        self.orientation[1] += gyro[1] * dt  # Pitch
        self.orientation[2] = np.arctan2(mag_y, mag_x)  # Yaw 보정

    def remove_gravity(self, accel):
        """가속도 데이터에서 중력 성분을 제거"""
        roll, pitch, _ = self.orientation
        g = 9.81  # 중력 가속도 (m/s²)

        # 중력 벡터 계산 (센서 기준 좌표계)
        gravity = np.array([
            -g * np.sin(pitch),
            g * np.sin(roll) * np.cos(pitch),
            g * np.cos(roll) * np.cos(pitch)
        ])

        return accel - gravity  # 중력 제거된 가속도 반환


class IMUPositionEstimator(Node):
    """ROS2 노드 - IMU 데이터 처리 및 위치/속도 계산"""

    def __init__(self):
        super().__init__('imu_position_estimator')

        # ✅ 센서 데이터 구독
        self.imu_sub = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(MagneticField, '/android/mag', self.mag_callback, 10)

        # ✅ 데이터 발행
        self.non_gravity_pub = self.create_publisher(Vector3, '/imu/non_gravity_acceleration', 10)
        self.velocity_pub = self.create_publisher(Vector3, '/imu/velocity', 10)
        self.position_pub = self.create_publisher(Vector3, '/imu/position', 10)

        # ✅ 초기값 설정
        self.gravity_remover = GravityRemover()  # 중력 제거 객체 생성
        self.prev_time = None
        self.velocity = np.zeros(3)  # 초기 속도
        self.position = np.zeros(3)  # 초기 위치
        self.mag_x = self.mag_y = 0.0  # 지자기 데이터 초기화

        self.get_logger().info("IMU Position Estimator Node Started!")

    def mag_callback(self, msg):
        """지자기 센서 데이터를 받아 저장"""
        self.mag_x, self.mag_y = msg.magnetic_field.x, msg.magnetic_field.y

    def imu_callback(self, msg):
        """IMU 데이터를 받아 중력 제거 및 속도/위치 계산"""

        current_time = self.get_clock().now().nanoseconds / 1e9  # 초 단위 변환

        if self.prev_time is None:
            self.prev_time = current_time
            return
        
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0 or dt > 0.1:
            self.get_logger().warn(f"⚠️ 비정상적인 dt 감지: {dt:.3f}s → 기본값 0.02s로 보정")
            dt = 0.02  # 50Hz 기준

        # ✅ 가속도 및 자이로 데이터 추출
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])

        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])

        # ✅ 중력 제거
        self.gravity_remover.update_orientation(gyro, dt, self.mag_x, self.mag_y)
        non_gravity_accel = self.gravity_remover.remove_gravity(accel)

        # ✅ 속도 및 위치 적분
        self.velocity += non_gravity_accel * dt
        self.position += self.velocity * dt

        # ✅ 중력 제거된 가속도 발행
        self.publish_vector(self.non_gravity_pub, non_gravity_accel)

        # ✅ 속도 발행
        self.publish_vector(self.velocity_pub, self.velocity)

        # ✅ 위치 발행
        self.publish_vector(self.position_pub, self.position)

        # ✅ 디버깅 로그 출력
        self.get_logger().info(f"""
📌 Original Acceleration: x={accel[0]:.3f}, y={accel[1]:.3f}, z={accel[2]:.3f}
🔄 Roll: {np.degrees(self.gravity_remover.orientation[0]):.2f}°, 
    Pitch: {np.degrees(self.gravity_remover.orientation[1]):.2f}°, 
    Yaw: {np.degrees(self.gravity_remover.orientation[2]):.2f}°
🌍 Gravity (sensor frame): x={accel[0] - non_gravity_accel[0]:.3f}, 
    y={accel[1] - non_gravity_accel[1]:.3f}, 
    z={accel[2] - non_gravity_accel[2]:.3f}
🚀 Non-Gravity Acceleration: x={non_gravity_accel[0]:.3f}, y={non_gravity_accel[1]:.3f}, z={non_gravity_accel[2]:.3f}
🏃 Velocity: x={self.velocity[0]:.3f}, y={self.velocity[1]:.3f}, z={self.velocity[2]:.3f}
📍 Position: x={self.position[0]:.3f}, y={self.position[1]:.3f}, z={self.position[2]:.3f}
🕒 Time Step: {dt:.3f}s
        """)

    def publish_vector(self, publisher, vector):
        """Vector3 메시지로 변환하여 발행"""
        msg = Vector3()
        msg.x, msg.y, msg.z = float(vector[0]), float(vector[1]), float(vector[2])
        publisher.publish(msg)


def main():
    rclpy.init()
    node = IMUPositionEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

