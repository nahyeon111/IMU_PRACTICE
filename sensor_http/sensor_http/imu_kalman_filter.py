import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import numpy as np

class KalmanFilter:
    def __init__(self):
        self.x = np.zeros(3)  # State: [roll, pitch, yaw]
        self.P = np.eye(3) * 0.1  # Covariance matrix
        self.Q = np.eye(3) * 0.01  # Process noise
        self.R = np.eye(3) * 0.1  # Measurement noise

    def predict(self, u):
        self.x += u  # Assuming simple integration model
        self.P += self.Q

    def update(self, z):
        K = self.P @ np.linalg.inv(self.P + self.R)
        self.x += K @ (z - self.x)
        self.P = (np.eye(3) - K) @ self.P
        return self.x

class IMUKalmanFilter(Node):
    def __init__(self):
        super().__init__('imu_kalman_filter')
        self.subscription = self.create_subscription(Vector3, '/vehicle/raw_orientation', self.kalman_callback, 10)
        self.publisher = self.create_publisher(Vector3, '/vehicle/filtered_orientation', 10)
        self.kf = KalmanFilter()
    
    def kalman_callback(self, msg):
        raw_data = np.array([msg.x, msg.y, msg.z])
        filtered_data = self.kf.update(raw_data)
        
        filtered_msg = Vector3()
        filtered_msg.x, filtered_msg.y, filtered_msg.z = filtered_data
        self.publisher.publish(filtered_msg)
        
        self.get_logger().info(f'Filtered Orientation - Roll: {filtered_msg.x:.2f}, Pitch: {filtered_msg.y:.2f}, Yaw: {filtered_msg.z:.2f}')

def main():
    rclpy.init()
    node = IMUKalmanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
