
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
import datetime, time, math, tf2_ros, numpy as np

from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock
from beamng_msgs.msg import StateSensor
from sensor_msgs.msg import Imu, Image, CameraInfo
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationStatusStamped
from geometry_msgs.msg import TransformStamped, PoseStamped, PoseWithCovarianceStamped
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, ControlModeReport, GearReport

from vehicleStatusPublisher import VehicleStatusPublisher
from utils.get_header import get_header


class AutowareBeamngBridge(Node):
    def __init__(self, vehicle, imu):
        super().__init__('beamng_scenario')

        self.vehicle = vehicle
        self.imu = imu
        self.vehicle.sensors.poll()

        self.initial_pose = {
            'x': 237.90,
            'y': -894.42,
            'z': 246.10
        }
        self.initial_orientation = {
            'x': 0.0173,
            'y': -0.0019,
            'z': -0.6354,
            'w': 0.7720,
        }
        
        # self.initial_pose = {
        #     'x': 0.0,
        #     'y': 0.0,
        #     'z': 0.0
        # }
        # self.initial_orientation = {
        #     'x': 0.0,
        #     'y': 0.0,
        #     'z': 0.0,
        #     'w': 1.0,
        # }

        # self.initial_pose = {
        #     'x': 3739.25,
        #     'y': 73729.0,
        #     'z': 0.0
        # }

        # self.initial_orientation = {
        #     'x': 0.0,
        #     'y': 0.0,
        #     'z': -0.966043,
        #     'w': 0.258382,
        # }

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self._create_publishers()
        self._create_subscriptions()
        
        self._publish_static_transforms()
        # self._publish_initial_pose()

        publishers = {
            'steering_state': self.pub_steering_state,
            'control_mode': self.pub_ctrl_mode,
            'velocity_state': self.pub_vel_state,
            'gear_state': self.pub_gear_state,
            'actuation_status': self.pub_actuation_status
        }

        self.status_publisher = VehicleStatusPublisher(
            self.vehicle.sensors['electrics2'], self.vehicle.sensors['state'], publishers
        )


    def _create_publishers(self):
        self._clock_publisher = self.create_publisher(
            Clock, '/clock', 10
        )

        # self.initial_pose_publisher = self.create_publisher(
        #     PoseStamped, '/initialpose', 1
        # )


        # ============ Sensors Publishers =================

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

    def start_publishers(self, stop_event):
        self.get_logger().info('\033[92m' + "All publishers started except lidar\n")

        while not stop_event.is_set():
            self.publish_sim_time()
            self.publish_vehicle_status()
            self.publish_state()
            self.publish_imu()
            time.sleep(0.1)
        
    # def _publish_initial_pose(self):
    #     # Create PoseStamped message
    #     pose_msg = PoseStamped()
    #     pose_msg.header.stamp = self.clock    
    #     pose_msg.header.frame_id = 'map'

    #     # Set position
    #     pose_msg.pose.position.x = self.initial_pose['x']
    #     pose_msg.pose.position.y = self.initial_pose['y']
    #     pose_msg.pose.position.z = self.initial_pose['z']

    #     pose_msg.pose.orientation.x = self.initial_orientation['x']
    #     pose_msg.pose.orientation.y = self.initial_orientation['y']
    #     pose_msg.pose.orientation.z = self.initial_orientation['z']
    #     pose_msg.pose.orientation.w = self.initial_orientation['w']

    #     # Publish the message
    #     self.initial_pose_publisher.publish(pose_msg)
    #     self.get_logger().info("Published initial pose to /initialpose")

    def publish_sim_time(self):
        simulation_time = Clock()
        simulation_time.clock = get_header(frame_id='map').stamp

        self._clock_publisher.publish(simulation_time)

    def publish_vehicle_status(self):
        self.vehicle.sensors.poll()
        self.status_publisher.publish(
            self.vehicle.sensors['electrics2'], self.vehicle.sensors['state']
        )

    def publish_state(self):
        state_msg = self.vehicle.sensors['state']

        # pose_with_cov msg
        out_pose_with_cov = PoseWithCovarianceStamped()
        out_pose_with_cov.header = get_header('map')
        out_pose_with_cov.pose.pose.position.x = state_msg['pos'][0]
        out_pose_with_cov.pose.pose.position.y = state_msg['pos'][1]
        out_pose_with_cov.pose.pose.position.z = state_msg['pos'][2]

        quat = self.direction_to_quaternion(state_msg['dir'])

        out_pose_with_cov.pose.pose.orientation.x = quat[0]
        out_pose_with_cov.pose.pose.orientation.y = quat[1]
        out_pose_with_cov.pose.pose.orientation.z = quat[2]
        out_pose_with_cov.pose.pose.orientation.w = quat[3]

        out_pose_with_cov.pose.covariance = [
            0.01,0.0,0.0,0.0,0.0,0.0,
            0.0,0.01,0.0,0.0,0.0,0.0,
            0.0,0.0,0.01,0.0,0.0,0.0,
            0.0,0.0,0.0,0.01,0.0,0.0,
            0.0,0.0,0.0,0.0,0.01,0.0,
            0.0,0.0,0.0,0.0,0.0,0.01
            ]

        self.pub_pose_with_cov.publish(out_pose_with_cov)
        self._publish_dynamic_transforms(state_msg)

    def publish_imu(self):
        imu_msg = self.imu.poll()
        
        out_imu_msg = Imu()
        out_imu_msg.header = get_header('tamagawa/imu_link_changed')

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
        base_to_lidar_top = TransformStamped()
        base_to_lidar_top.header = get_header('velodyne_top')
        base_to_lidar_top.child_frame_id = 'velodyne_top_changed'
        base_to_lidar_top.transform.translation.x = -0.1
        base_to_lidar_top.transform.translation.y = 0.65
        base_to_lidar_top.transform.translation.z = 2.0
        base_to_lidar_top.transform.rotation.w = 1.0
        
        # base_to_lidar_right = TransformStamped()
        # base_to_lidar_right.header = get_header('velodyne_right')
        # base_to_lidar_right.child_frame_id = 'velodyne_right_changed'
        # base_to_lidar_right.transform.translation.x = 0.0
        # base_to_lidar_right.transform.translation.y = 0.65
        # base_to_lidar_right.transform.translation.z = 2.0
        # base_to_lidar_right.transform.rotation.w = 1.0
        
        # base_to_lidar_left = TransformStamped()
        # base_to_lidar_left.header = get_header('velodyne_left')
        # base_to_lidar_left.child_frame_id = 'velodyne_left_changed'
        # base_to_lidar_left.transform.translation.x = 0.1
        # base_to_lidar_left.transform.translation.y = 0.65
        # base_to_lidar_left.transform.translation.z = 2.0
        # base_to_lidar_left.transform.rotation.w = 1.0
        
        # Create transform from base_link to lidar
        base_to_imu = TransformStamped()
        base_to_imu.header = get_header('tamagawa/imu_link')
        base_to_imu.child_frame_id = 'tamagawa/imu_link_changed'
        base_to_imu.transform.translation.x = 0.0
        base_to_imu.transform.translation.y = 0.0
        base_to_imu.transform.translation.z = 1.7
        base_to_imu.transform.rotation.w = 1.0

        base_to_gnss = TransformStamped()
        base_to_gnss.header.stamp = self.get_clock().now().to_msg()
        base_to_gnss.header.frame_id = 'base_link'
        base_to_gnss.child_frame_id = 'gnss'
        base_to_gnss.transform.translation.z = 0.0
        base_to_gnss.transform.rotation.w = 1.0

        # Publish both transforms
        self.static_tf_broadcaster.sendTransform([base_to_lidar_top, base_to_imu, base_to_gnss])

    def _publish_dynamic_transforms(self, state_msg):
        # Create transform from map to base_link
        map_to_base = TransformStamped()
        map_to_base.header = get_header('map')
        map_to_base.child_frame_id = 'base_link'

        map_to_base.transform.translation.x = state_msg['pos'][0]
        map_to_base.transform.translation.y = state_msg['pos'][1]
        map_to_base.transform.translation.z = self.initial_pose['z'] 
        
        quat = self.direction_to_quaternion(state_msg['dir'])

        map_to_base.transform.rotation.x = quat[0]
        map_to_base.transform.rotation.y = quat[1]
        map_to_base.transform.rotation.z = quat[2]
        map_to_base.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(map_to_base)

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
