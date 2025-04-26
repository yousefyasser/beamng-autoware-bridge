import numpy as np

from rclpy.node import Node

import tf2_ros

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField, Imu
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from beamng_msgs.msg import StateSensor
from autoware_auto_vehicle_msgs.msg import SteeringReport, VelocityReport, ControlModeReport

import math


class AutowareBeamngBridge(Node):
    def __init__(self):
        super().__init__('beamng_scenario')

        self.vehicle_velocity = 0.0
        self.initial_pose = {
            'x': 3739.25,
            'y': 73729.0,
            'z': 0.0
        }

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.create_publishers()
        self.create_subscriptions()
        
        self._publish_static_transforms()

    def create_publishers(self):
        self.lidar_publisher = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            10
        )

        self.initial_pose_publisher = self.create_publisher(
            PoseStamped,
            '/initialpose',
            10
        )

        self._pose_with_cov_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/sensing/gnss/pose_with_covariance', 
            1
        )

        self._steering_status_publisher = self.create_publisher(
            SteeringReport, 
            '/vehicle/status/steering_status', 
            1
        )
        
        self._vehicle_control_mode_publisher = self.create_publisher(
            ControlModeReport, 
            '/vehicle/status/control_mode', 
            1
        )
        
        self._velocity_report_publisher = self.create_publisher(
            VelocityReport, 
            '/vehicle/status/velocity_status', 
            1
        )
    
    def create_subscriptions(self):
        self.state_subscriber = self.create_subscription(
            StateSensor,
            '/vehicles/ego/sensors/state',
            self._state_callback,
            1
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/vehicles/ego/sensors/position_imu0',
            self._imu_callback,
            1
        )

    def _state_callback(self, state_msg):
        # Publish static control mode
        control_mode_msg = ControlModeReport()
        control_mode_msg.stamp = self.get_clock().now().to_msg()
        control_mode_msg.mode = 1
        self._vehicle_control_mode_publisher.publish(control_mode_msg)


        self.vehicle_velocity = np.sqrt(state_msg.velocity.x ** 2 + state_msg.velocity.y ** 2 + state_msg.velocity.z ** 2)

        # pose_with_cov msg
        out_pose_with_cov = PoseWithCovarianceStamped()
        out_pose_with_cov.header.frame_id = 'map'
        out_pose_with_cov.header.stamp = state_msg.header.stamp
        out_pose_with_cov.pose.pose.position = state_msg.position
        out_pose_with_cov.pose.pose.orientation.x = 0.0
        out_pose_with_cov.pose.pose.orientation.y = 0.0
        out_pose_with_cov.pose.pose.orientation.z = state_msg.dir.y
        out_pose_with_cov.pose.pose.orientation.w = state_msg.dir.x

        out_pose_with_cov.pose.covariance = [
            0.1,0.0,0.0,0.0,0.0,0.0,
            0.0,0.1,0.0,0.0,0.0,0.0,
            0.0,0.0,0.1,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0
            ]
        self._pose_with_cov_publisher.publish(out_pose_with_cov)
        self._publish_dynamic_transforms(state_msg)

    def _imu_callback(self, imu_msg):
        output_velocity_report = VelocityReport()
        output_velocity_report.heading_rate = -imu_msg.angular_velocity.z
        output_velocity_report.longitudinal_velocity = self.vehicle_velocity
        output_velocity_report.lateral_velocity = -imu_msg.linear_acceleration.y 

        header = Header()
        header.frame_id = 'base_link'
        header.stamp = imu_msg.header.stamp
        output_velocity_report.header = header
        self._velocity_report_publisher.publish(output_velocity_report)

    def _publish_static_transforms(self):
        # Create transform from base_link to lidar
        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = self.get_clock().now().to_msg()
        base_to_lidar.header.frame_id = 'base_link'
        base_to_lidar.child_frame_id = 'lidar_top'
        base_to_lidar.transform.translation.x = 0.0
        base_to_lidar.transform.translation.y = 0.0
        base_to_lidar.transform.translation.z = 3.0  # Lidar height above vehicle
        base_to_lidar.transform.rotation.w = 1.0

        # Publish both transforms
        self.static_tf_broadcaster.sendTransform([base_to_lidar])

    def _publish_dynamic_transforms(self, state_msg):
        # Create transform from map to base_link
        map_to_base = TransformStamped()
        map_to_base.header.stamp = self.get_clock().now().to_msg()
        map_to_base.header.frame_id = 'map'
        map_to_base.child_frame_id = 'base_link'
        # Set vehicle's initial position in the map
        map_to_base.transform.translation.x = self.initial_pose['x'] + state_msg.position.x
        map_to_base.transform.translation.y = self.initial_pose['y'] + state_msg.position.y
        map_to_base.transform.translation.z = self.initial_pose['z'] 
        
        # Convert to quaternion (w, x, y, z)
        
        map_to_base.transform.rotation.x = 0.0
        map_to_base.transform.rotation.y = 0.0 
        map_to_base.transform.rotation.z = state_msg.dir.y
        map_to_base.transform.rotation.w = state_msg.dir.x

        self.tf_broadcaster.sendTransform(map_to_base)

    def publish_lidar_data(self, point_cloud):
        # Publish LIDAR data
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_top'
        
        # Transform point cloud to match Autoware's coordinate system if needed
        # BeamNG uses a different coordinate system than ROS/Autoware
        transformed_points = np.copy(point_cloud)
        # Swap axes if needed - this may need adjustment based on testing
        # transformed_points[:, [0, 1, 2]] = point_cloud[:, [0, 2, 1]]
        # filter points with z coordinate <= 0
        
        msg.height = 1
        msg.width = len(transformed_points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(transformed_points)
        msg.data = transformed_points.astype(np.float32).tobytes()
        msg.is_dense = True
        
        # Add more detailed logging
        # self.get_logger().info(f"Publishing LIDAR data: {len(point_cloud)} points to topic: /sensing/lidar/top/pointcloud_raw")
        
        # Calculate some statistics for logging
        if len(point_cloud) > 0:
            distances = np.sqrt(np.sum(point_cloud**2, axis=1))
            valid_distances = distances[distances > 0]
            if len(valid_distances) > 0:
                min_dist = np.min(valid_distances)
                max_dist = np.max(valid_distances)
                # self.get_logger().info(f"LIDAR stats - Min distance: {min_dist:.2f}m, Max distance: {max_dist:.2f}m")
        
        self.lidar_publisher.publish(msg)
