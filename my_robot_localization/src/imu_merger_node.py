import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

from message_filters import Subscriber, ApproximateTimeSynchronizer

#import numpy as np
#from tf2_transformations import quaternion_multiply, quaternion_from_euler



class ImuMerger(Node):
    def __init__(self):
        super().__init__('imu_merger')

        # Declare parameters with default values
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("orientation_covariance", [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05])
        self.declare_parameter("angular_velocity_covariance", [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02])
        self.declare_parameter("linear_acceleration_covariance", [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01])

        # Publisher for the complete IMU data
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)

        # Create message_filters subscribers for each topic
        self.gyro_sub = Subscriber(self, Vector3, 'gyro')
        self.accel_sub = Subscriber(self, Vector3, 'acceleration')
        self.quat_sub = Subscriber(self, Quaternion, 'quaternion')
        # Synchronize headerless topics (with allow_headerless=True)
        self.ts = ApproximateTimeSynchronizer(
            [self.gyro_sub, self.accel_sub, self.quat_sub],
            queue_size=10,
            slop=0.1,
            allow_headerless=True
        )
        self.ts.registerCallback(self.sync_callback)

        self.get_logger().info('IMU Merger node has started.')
        
    #def convert_to_enu(self, imu_msg):
    #    # --- Linear Acceleration ---
    #    accel_ned = np.array([
    #        imu_msg.linear_acceleration.x,
    #        imu_msg.linear_acceleration.y,
    #        imu_msg.linear_acceleration.z
    #    ])
    #    accel_enu = np.array([accel_ned[1], accel_ned[0], -accel_ned[2]])
    #    imu_msg.linear_acceleration.x = accel_enu[0]
    #    imu_msg.linear_acceleration.y = accel_enu[1]
    #    imu_msg.linear_acceleration.z = accel_enu[2]

        # --- Angular Velocity ---
    #    ang_vel_ned = np.array([
    #        imu_msg.angular_velocity.x,
    #        imu_msg.angular_velocity.y,
    #        imu_msg.angular_velocity.z
    #    ])
    #    ang_vel_enu = np.array([ang_vel_ned[1], ang_vel_ned[0], -ang_vel_ned[2]])
    #    imu_msg.angular_velocity.x = ang_vel_enu[0]
    #    imu_msg.angular_velocity.y = ang_vel_enu[1]
    #    imu_msg.angular_velocity.z = ang_vel_enu[2]

        # --- Orientation (Quaternion) ---
    #    q_ned = [
    #        imu_msg.orientation.x,
    #        imu_msg.orientation.y,
    #        imu_msg.orientation.z,
    #        imu_msg.orientation.w
    #    ]
    #    # Rotazione di 180° attorno all'asse X
    #    R_flip = tf_transformations.quaternion_from_euler(np.pi, 0, 0)
    #    q_enu = tf_transformations.quaternion_multiply(R_flip, q_ned)

    #    imu_msg.orientation.x = q_enu[0]
    #    imu_msg.orientation.y = q_enu[1]
    #    imu_msg.orientation.z = q_enu[2]
     #   imu_msg.orientation.w = q_enu[3]

     #   return imu_msg
    
    
    def sync_callback(self, gyro: Vector3, acceleration: Vector3, quaternion: Quaternion):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.get_parameter("frame_id").value

        imu_msg.orientation = quaternion
        imu_msg.angular_velocity = gyro
        imu_msg.linear_acceleration = acceleration

        # Set covariance values from parameters (each should be a list of 9 elements)
        imu_msg.orientation_covariance = self.get_parameter("orientation_covariance").value
        imu_msg.angular_velocity_covariance = self.get_parameter("angular_velocity_covariance").value
        imu_msg.linear_acceleration_covariance = self.get_parameter("linear_acceleration_covariance").value
        
     #   imu_msg = self.convert_to_enu(imu_msg)
        

        self.imu_pub.publish(imu_msg)
        self.get_logger().debug('Published IMU message.')
        

    
    
def main(args=None):
    rclpy.init(args=args)
    node = ImuMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU Merger node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
