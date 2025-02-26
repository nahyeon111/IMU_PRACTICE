import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import math
import numpy as np

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        self.subscription_imu = self.create_subscription(Imu, '/android/imu', self.imu_callback, 10)
        self.subscription_mag = self.create_subscription(MagneticField, '/android/magnetic_field', self.mag_callback, 10)
        self.publisher_raw_data = self.create_publisher(Vector3, '/vehicle/raw_orientation', 10)
        self.publisher_position = self.create_publisher(Vector3, '/vehicle/position', 10)
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.dt = 0.02  # Assume 50Hz update rate
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])

    def imu_callback(self, msg):
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # Roll and Pitch from accelerometer
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))
        
        # Gyroscope integration
        self.roll += gx * self.dt
        self.pitch += gy * self.dt
        self.yaw += gz * self.dt  
        
        # Update position by integrating acceleration twice + 수정 필요 
        acc_vector = np.array([ax, ay, az])
        self.velocity += acc_vector * self.dt
        self.position += self.velocity * self.dt

        raw_msg = Vector3()
        raw_msg.x, raw_msg.y, raw_msg.z = self.roll, self.pitch, self.yaw
        self.publisher_raw_data.publish(raw_msg)

        pos_msg = Vector3()
        pos_msg.x, pos_msg.y, pos_msg.z = self.position
        self.publisher_position.publish(pos_msg)
        
        # 터미널에 위치 데이터 출력 + 수정 필요 
        self.get_logger().info(f'Position: x={self.position[0]:.2f}, y={self.position[1]:.2f}, z={self.position[2]:.2f}')

    def mag_callback(self, msg):
        mx, my, mz = msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z
        self.yaw = math.atan2(my, mx)

def main():
    rclpy.init()
    node = IMUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
