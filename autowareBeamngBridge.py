import numpy as np

from rclpy.node import Node

import tf2_ros

import datetime
import time
import math

from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header

from sensor_msgs.msg import PointCloud2, PointField, Imu, Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped

from beamng_msgs.msg import StateSensor

from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, ControlModeReport, GearReport
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationStatusStamped

from vehicleStatusPublisher import VehicleStatusPublisher

from scipy.spatial.transform import Rotation as R


class AutowareBeamngBridge(Node):
    def __init__(self, vehicle, imu):
        super().__init__('beamng_scenario')

        self.vehicle = vehicle
        self.imu = imu
        self.vehicle.sensors.poll()

        self.timestamp = 0.0
        self.clock = Time(sec=0)
        self.initial_pose = {
            'x': 3739.25,
            'y': 73729.0,
            'z': 0.0
        }

        self.initial_orientation = {
            'x': 0.0,
            'y': 0.0,
            'z': -0.966043,
            'w': 0.258382,
        }

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self._create_publishers()
        self._create_subscriptions()
        
        self._publish_static_transforms()
        self._publish_initial_pose()

        publishers = {
            'steering_state': self.pub_steering_state,
            'control_mode': self.pub_ctrl_mode,
            'velocity_state': self.pub_vel_state,
            'gear_state': self.pub_gear_state,
            'actuation_status': self.pub_actuation_status
        }

        self.status_publisher = VehicleStatusPublisher(
            self.clock, self.vehicle.sensors['electrics2'], self.vehicle.sensors['state'], publishers
        )

        self.create_timer(0.01, self.publish_sim_time)
        self.create_timer(0.01, self.publish_vehicle_status)
        self.create_timer(0.01, self.publish_imu)
        self.create_timer(0.01, self.publish_state)

    def _create_publishers(self):
        self._clock_publisher = self.create_publisher(
            Clock, '/clock', 10
        )

        self.initial_pose_publisher = self.create_publisher(
            PoseStamped, '/initialpose', 1
        )


        # ============ Sensors Publishers =================

        self.pub_lidar = self.create_publisher(
            PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10
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
        pass
        # self.sub_control = self.create_subscription(
        #     ActuationCommandStamped, "/control/command/actuation_cmd", self.control_callback, 1
        # )

    def _publish_initial_pose(self):
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()     
        pose_msg.header.frame_id = 'map'

        # Set position
        pose_msg.pose.position.x = self.initial_pose['x']
        pose_msg.pose.position.y = self.initial_pose['y']
        pose_msg.pose.position.z = self.initial_pose['z']

        pose_msg.pose.orientation.x = self.initial_orientation['x']
        pose_msg.pose.orientation.y = self.initial_orientation['y']
        pose_msg.pose.orientation.z = self.initial_orientation['z']
        pose_msg.pose.orientation.w = self.initial_orientation['w']

        # Publish the message
        self.initial_pose_publisher.publish(pose_msg)
        self.get_logger().info("Published initial pose to /initialpose")

    def publish_sim_time(self):
        # self.timestamp += 1/60
        self.timestamp = time.time()

        seconds = int(self.timestamp)
        nanoseconds = int((self.timestamp - int(self.timestamp)) * 1000000000.0)
        
        simulation_time = Clock()
        simulation_time.clock = Time(sec=seconds, nanosec=nanoseconds)
        self.clock = simulation_time.clock

        self._clock_publisher.publish(simulation_time)

    def publish_vehicle_status(self):
        self.vehicle.sensors.poll()
        self.status_publisher.publish(
            self.clock, self.vehicle.sensors['electrics2'], self.vehicle.sensors['state']
        )

    def publish_state(self):
        state_msg = self.vehicle.sensors['state']

        # pose_with_cov msg
        out_pose_with_cov = PoseWithCovarianceStamped()
        out_pose_with_cov.header.frame_id = 'map'
        out_pose_with_cov.header.stamp = self.clock
        out_pose_with_cov.pose.pose.position.x = state_msg['pos'][0]
        out_pose_with_cov.pose.pose.position.y = state_msg['pos'][1]
        out_pose_with_cov.pose.pose.position.z = state_msg['pos'][2]

        quat = self.direction_to_quaternion(state_msg['dir'])

        out_pose_with_cov.pose.pose.orientation.x = quat[0]
        out_pose_with_cov.pose.pose.orientation.y = quat[1]
        out_pose_with_cov.pose.pose.orientation.z = quat[2]
        out_pose_with_cov.pose.pose.orientation.w = quat[3]

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

    def publish_imu(self):
        imu_msg = self.imu.poll()
        
        out_imu_msg = Imu()
        out_imu_msg.header.frame_id = 'tamagawa/imu_link_changed'
        out_imu_msg.header.stamp = self.clock

        out_imu_msg.orientation.x = 0.0
        out_imu_msg.orientation.y = 0.0
        out_imu_msg.orientation.z = 0.0
        out_imu_msg.orientation.w = 1.0

        out_imu_msg.angular_velocity.x = imu_msg['angVelSmooth'][0]
        out_imu_msg.angular_velocity.y = imu_msg['angVelSmooth'][1]
        out_imu_msg.angular_velocity.z = imu_msg['angVelSmooth'][2]

        out_imu_msg.linear_acceleration.x = imu_msg['accSmooth'][0]
        out_imu_msg.linear_acceleration.y = imu_msg['accSmooth'][1]
        out_imu_msg.linear_acceleration.z = imu_msg['accSmooth'][2]

        out_imu_msg.orientation_covariance = [-1.0,] * 9
        out_imu_msg.angular_velocity_covariance = [-1.0,] * 9
        out_imu_msg.linear_acceleration_covariance = [-1.0,] * 9

        self.pub_imu.publish(out_imu_msg)
        

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

        map_to_base.transform.translation.x = self.initial_pose['x'] + state_msg['pos'][0]
        map_to_base.transform.translation.y = self.initial_pose['y'] + state_msg['pos'][1]
        map_to_base.transform.translation.z = self.initial_pose['z'] 
        
        quat = self.direction_to_quaternion(state_msg['dir'])

        map_to_base.transform.rotation.x = quat[0]
        map_to_base.transform.rotation.y = quat[1]
        map_to_base.transform.rotation.z = quat[2]
        map_to_base.transform.rotation.w = quat[3]

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

    def direction_to_quaternion(self, direction, forward_reference=np.array([1, 0, 0])):
        """
        Converts a direction vector into a quaternion that represents a rotation from
        a reference forward vector to the given direction.

        :param direction: List or np.array of shape (3,) representing [x, y, z]
        :param forward_reference: The default 'forward' vector of the vehicle
        :return: Quaternion as [x, y, z, w]
        """
        direction = np.array(direction, dtype=float)
        norm = np.linalg.norm(direction)
        if norm == 0:
            raise ValueError("Direction vector cannot be zero.")
        direction = direction / norm

        forward_reference = forward_reference / np.linalg.norm(forward_reference)

        # If vectors are the same, no rotation needed
        if np.allclose(direction, forward_reference):
            return [0, 0, 0, 1]

        # If vectors are opposite, rotate 180 degrees around an arbitrary perpendicular axis
        if np.allclose(direction, -forward_reference):
            # Pick an arbitrary perpendicular vector
            axis = np.cross(forward_reference, [0, 0, 1])
            if np.linalg.norm(axis) < 1e-6:
                axis = np.cross(forward_reference, [0, 1, 0])
            axis = axis / np.linalg.norm(axis)
            rot = R.from_rotvec(np.pi * axis)
            return rot.as_quat().tolist()

        # Compute rotation between vectors
        rot = R.from_rotvec(np.arccos(np.clip(np.dot(forward_reference, direction), -1.0, 1.0)) *
                            np.cross(forward_reference, direction) / np.linalg.norm(np.cross(forward_reference, direction)))
        return rot.as_quat().tolist()
