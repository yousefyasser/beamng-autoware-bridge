import numpy as np

from rclpy.node import Node

import tf2_ros

import datetime

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2, PointField, Imu, Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped

from beamng_msgs.msg import StateSensor
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, ControlModeReport, GearReport
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationStatusStamped

import math


class AutowareBeamngBridge(Node):
    def __init__(self):
        super().__init__('beamng_scenario')

        self.timestamp = 0.0
        self.clock = Time(sec=0)
        self.vehicle_velocity = 0.0
        self.initial_pose = {
            'x': 3739.25,
            'y': 73729.0,
            'z': 0.0
        }

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self._create_publishers()
        self._create_subscriptions()
        
        self._publish_static_transforms()
        # self._publish_initial_pose()

        self.create_timer(1/60, self.publish_sim_time)

    def _create_publishers(self):
        self._clock_publisher = self.create_publisher(
            Clock, '/clock', 10
        )

        # self.initial_pose_publisher = self.create_publisher(
        #     PoseStamped, '/initialpose', 1
        # )


        # ============ Sensors Publishers =================

        self.pub_lidar = self.create_publisher(
            PointCloud2, '/sensing/lidar/top/pointcloud_before_sync', 10
        )

        self.pub_pose_with_cov = self.create_publisher(
            PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', 1
        )

        self.pub_imu = self.create_publisher(
            Imu, "/sensing/imu/tamagawa/imu_raw", 1
        )


        # ========= Vehicle States Publishers ==============
        self.pub_steering_state = self.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', 1
        )
        
        self.pub_ctrl_mode = self.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode', 1
        )
        
        self.pub_vel_state = self.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', 1
        )

        self.pub_gear_state = self.create_publisher(
            GearReport, "/vehicle/status/gear_status", 1
        )
        
        self.pub_actuation_status = self.create_publisher(
            ActuationStatusStamped, "/vehicle/status/actuation_status", 1
        )
    
    def _create_subscriptions(self):
        self.state_subscriber = self.create_subscription(
            StateSensor, '/vehicles/ego/sensors/state', self._state_callback, 1
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu, '/vehicles/ego/sensors/position_imu0', self._imu_callback, 1
        )

        # self.sub_control = self.create_subscription(
        #     ActuationCommandStamped, "/control/command/actuation_cmd", self.control_callback, 1
        # )

    # def _publish_initial_pose(self):
    #     # Create PoseStamped message
    #     pose_msg = PoseStamped()
    #     pose_msg.header.stamp = self.get_clock().now().to_msg()     
    #     pose_msg.header.frame_id = 'map'

    #     # Set position
    #     pose_msg.pose.position.x = self.initial_pose['x']
    #     pose_msg.pose.position.y = self.initial_pose['y']
    #     pose_msg.pose.position.z = self.initial_pose['z']

    #     pose_msg.pose.orientation.x = 0.0
    #     pose_msg.pose.orientation.y = 0.0
    #     pose_msg.pose.orientation.z = -0.980555
    #     pose_msg.pose.orientation.w = 0.196243

    #     # Publish the message
    #     self.initial_pose_publisher.publish(pose_msg)
    #     self.get_logger().info("Published initial pose to /initialpose")

    def publish_sim_time(self):
        self.timestamp += 1/60

        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        
        simulation_time = Clock()
        simulation_time.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock = simulation_time.clock

        self._clock_publisher.publish(simulation_time)

    def _state_callback(self, state_msg):
        self.vehicle_velocity = np.sqrt(state_msg.velocity.x ** 2 + state_msg.velocity.y ** 2 + state_msg.velocity.z ** 2) / 40

        # pose_with_cov msg
        out_pose_with_cov = PoseWithCovarianceStamped()
        out_pose_with_cov.header.frame_id = 'map'
        out_pose_with_cov.header.stamp = self.clock
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
        self.pub_pose_with_cov.publish(out_pose_with_cov)
        self._publish_dynamic_transforms(state_msg)

    def _imu_callback(self, imu_msg):
        # publish imu
        out_imu_msg = imu_msg
        out_imu_msg.header.frame_id = 'tamagawa/imu_link_changed'
        out_imu_msg.header.stamp = self.clock
        self.pub_imu.publish(out_imu_msg)

        # publish velocity report
        header = Header()
        header.frame_id = 'base_link'
        header.stamp = self.clock

        output_velocity_report = VelocityReport()
        output_velocity_report.heading_rate = -imu_msg.angular_velocity.z
        output_velocity_report.longitudinal_velocity = self.vehicle_velocity
        output_velocity_report.lateral_velocity = -imu_msg.linear_acceleration.y 

        output_velocity_report.header = header
        self.pub_vel_state.publish(output_velocity_report)

        # Publish static control mode
        control_mode_msg = ControlModeReport()
        control_mode_msg.stamp = header.stamp
        control_mode_msg.mode = ControlModeReport.AUTONOMOUS
        self.pub_ctrl_mode.publish(control_mode_msg)

        # publish gear status
        out_gear_state = GearReport()
        out_gear_state.stamp = header.stamp
        out_gear_state.report = GearReport.DRIVE
        self.pub_gear_state.publish(out_gear_state)

        # publish actuation status
        out_actuation_status = ActuationStatusStamped()
        out_actuation_status.header = header
        out_actuation_status.status.accel_status = 0.0
        out_actuation_status.status.brake_status = 0.0
        out_actuation_status.status.steer_status = 0.0
        self.pub_actuation_status.publish(out_actuation_status)

        # publish steering status
        out_steering_state = SteeringReport()
        out_steering_state.stamp = header.stamp
        out_steering_state.steering_tire_angle = 0.0
        self.pub_steering_state.publish(out_steering_state)

    def _publish_static_transforms(self):
        # Create transform from base_link to lidar
        base_to_lidar = TransformStamped()
        base_to_lidar.header.stamp = self.get_clock().now().to_msg()
        base_to_lidar.header.frame_id = 'velodyne_top'
        base_to_lidar.child_frame_id = 'velodyne_top_changed'
        base_to_lidar.transform.translation.x = 0.0
        base_to_lidar.transform.translation.y = 0.0
        base_to_lidar.transform.translation.z = 3.0  # Lidar height above vehicle
        base_to_lidar.transform.rotation.w = 1.0
        
        # Create transform from base_link to lidar
        base_to_imu = TransformStamped()
        base_to_imu.header.stamp = self.get_clock().now().to_msg()
        base_to_imu.header.frame_id = 'tamagawa/imu_link'
        base_to_imu.child_frame_id = 'tamagawa/imu_link_changed'
        base_to_imu.transform.translation.x = 0.0
        base_to_imu.transform.translation.y = 0.0
        base_to_imu.transform.translation.z = 3.0  # Lidar height above vehicle
        base_to_imu.transform.rotation.w = 1.0

        # base_to_gnss = TransformStamped()
        # base_to_gnss.header.stamp = self.get_clock().now().to_msg()
        # base_to_gnss.header.frame_id = 'base_link'
        # base_to_gnss.child_frame_id = 'gnss'
        # base_to_gnss.transform.translation.z = 0.0
        # base_to_gnss.transform.rotation.w = 1.0

        # Publish both transforms
        self.static_tf_broadcaster.sendTransform([base_to_lidar, base_to_imu])

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
        map_to_base.transform.rotation.z = -state_msg.dir.y
        map_to_base.transform.rotation.w = state_msg.dir.x

        self.tf_broadcaster.sendTransform(map_to_base)

    def publish_lidar_data(self, point_cloud):
        # Publish LIDAR data
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.clock
        msg.header.frame_id = 'velodyne_top_changed'
        
        # Get number of points
        num_points = len(point_cloud)
        
        # Create a byte array to hold all point data
        # 32 bytes per point as specified in the error message
        buffer = bytearray(num_points * 32)
        
        # For each point, manually pack the data into the buffer
        for i, point in enumerate(point_cloud):
            # Calculate buffer offset for this point
            offset = i * 32
            
            # Pack XYZ as float32 (4 bytes each)
            x_bytes = np.float32(point[0]).tobytes()
            y_bytes = np.float32(point[1]).tobytes()
            z_bytes = np.float32(point[2]).tobytes()
            
            # Copy XYZ bytes to buffer
            buffer[offset:offset+4] = x_bytes
            buffer[offset+4:offset+8] = y_bytes
            buffer[offset+8:offset+12] = z_bytes
            
            # Set intensity (uint8, 1 byte)
            buffer[offset+12] = 100  # Default intensity
            
            # Set return_type (uint8, 1 byte)
            buffer[offset+13] = 0
            
            # Set channel (uint16, 2 bytes)
            channel_bytes = np.uint16(0).tobytes()
            buffer[offset+14:offset+16] = channel_bytes
            
            # Calculate and set azimuth (float32, 4 bytes)
            azimuth = np.float32(np.arctan2(point[1], point[0])).tobytes()
            buffer[offset+16:offset+20] = azimuth
            
            # Calculate and set elevation (float32, 4 bytes)
            xy_dist = np.sqrt(point[0]**2 + point[1]**2)
            elevation = np.float32(np.arctan2(point[2], xy_dist)).tobytes()
            buffer[offset+20:offset+24] = elevation
            
            # Calculate and set distance (float32, 4 bytes)
            distance = np.float32(np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)).tobytes()
            buffer[offset+24:offset+28] = distance
            
            # Set timestamp (float32, 4 bytes)
            timestamp = np.float32(self.get_clock().now().nanoseconds / 1e9).tobytes()
            buffer[offset+28:offset+32] = timestamp
        
        # Set up the PointCloud2 message
        msg.height = 1
        msg.width = num_points
        
        # Define fields with correct offsets
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
            PointField(name='azimuth', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='elevation', offset=20, datatype=PointField.FLOAT32, count=1),
            PointField(name='distance', offset=24, datatype=PointField.FLOAT32, count=1),
            PointField(name='time_stamp', offset=28, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 32  # Total size of one point in bytes
        msg.row_step = msg.point_step * num_points
        msg.data = bytes(buffer)  # Convert bytearray to bytes
        msg.is_dense = True
        
        self.pub_lidar.publish(msg)
