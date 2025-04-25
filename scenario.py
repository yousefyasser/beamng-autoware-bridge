from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Radar
import numpy as np
import os
import sys
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

import math

def get_windows_ip():
    try:
        result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'default via' in line:
                return line.split()[2]
    except:
        pass
    return '172.24.64.1'

class BeamNGScenario(Node):
    def __init__(self):
        super().__init__('beamng_scenario')
        self.lidar_publisher = self.create_publisher(
            PointCloud2,
            '/sensing/lidar/concatenated/pointcloud',
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Add publisher for initial pose
        self.initial_pose_publisher = self.create_publisher(
            PoseStamped,
            '/initialpose',
            10
        )

        self._pose_with_cov_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', 1)
        
        # Publish initial static transforms
        self.publish_static_transforms()
        
        # Publish initial pose
        self.publish_initial_pose()
    
    def publish_initial_pose(self):
        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Set position
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        
        # Set orientation (-90 degrees yaw)
        angle = math.radians(90)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(angle/2)
        pose_msg.pose.orientation.w = math.cos(angle/2)
        
        # Publish the message
        self.initial_pose_publisher.publish(pose_msg)
        self.get_logger().info("Published initial pose to /initialpose")

    def publish_car_state(self, state_msg):
        # velocity_report_msg = state_msg

        header = Header()
        header.frame_id = 'base_link'
        header.stamp = state_msg.header.stamp
        # velocity_report_msg.header = header
        # self._velocity_report_publisher.publish(velocity_report_msg)

        # pose_with_cov msg
        out_pose_with_cov = PoseWithCovarianceStamped()
        out_pose_with_cov.header.frame_id = 'map'
        out_pose_with_cov.header.stamp = state_msg.header.stamp
        out_pose_with_cov.pose.pose.position = state_msg.position
        out_pose_with_cov.pose.pose.orientation.x = state_msg.dir.x
        out_pose_with_cov.pose.pose.orientation.y = state_msg.dir.y
        out_pose_with_cov.pose.pose.orientation.z = state_msg.dir.z
        out_pose_with_cov.pose.pose.orientation.w = 1.0

        out_pose_with_cov.pose.covariance = [
            0.1,0.0,0.0,0.0,0.0,0.0,
            0.0,0.1,0.0,0.0,0.0,0.0,
            0.0,0.0,0.1,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0
            ]
        self._pose_with_cov_publisher.publish(out_pose_with_cov)

    def publish_static_transforms(self):
        # Create transform from map to base_link
        map_to_base = TransformStamped()
        map_to_base.header.stamp = self.get_clock().now().to_msg()
        map_to_base.header.frame_id = 'map'
        map_to_base.child_frame_id = 'base_link'
        # Set vehicle's initial position in the map
        map_to_base.transform.translation.x = 0.0
        map_to_base.transform.translation.y = 0.0
        map_to_base.transform.translation.z = 0.0
        
        # Set rotation to -90 degrees around Z axis (yaw)
        # Convert to quaternion (w, x, y, z)
        
        angle = math.radians(90)
        map_to_base.transform.rotation.x = 0.0
        map_to_base.transform.rotation.y = 0.0
        map_to_base.transform.rotation.z = math.sin(angle/2)
        map_to_base.transform.rotation.w = math.cos(angle/2)
        
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
        self.static_tf_broadcaster.sendTransform([map_to_base, base_to_lidar])

    def publish_lidar_data(self, point_cloud):
        # Get current vehicle position from BeamNG
        # Get rotation as well
        
        # Update transform with current vehicle position
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'lidar_top'
        tf.transform.translation.x = 0.0  # Adjust these values based on your LIDAR position
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 3.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)

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

def main():
    rclpy.init()
    scenario_node = BeamNGScenario()
    beamng_home = os.getenv('BNG_HOME')
    if not beamng_home:
        print("Error: BNG_HOME environment variable not set!")
        sys.exit(1)

    windows_ip = get_windows_ip()
    input("Press Enter to start")
    print(f"Connecting to BeamNG.tech on {windows_ip}:25252...")

    try:
        bng = BeamNGpy(windows_ip, 25252, home=beamng_home)
        bng.open(launch=False)
        
        scenario = bng.scenario.get_current()
        active_vehicles = bng.vehicles.get_current()
        car = active_vehicles["ego"]
        
        # Setup camera
        # camera = Camera(
        #     'camera1',
        #     bng,
        #     car,
        #     requested_update_time=0.1,
        #     is_using_shared_memory=False,
        #     pos=(-0.3, 1, 2),
        #     dir=(0, -1, 0),
        #     field_of_view_y=70,
        #     near_far_planes=(0.1, 1000),
        #     resolution=(640, 480),
        #     is_streaming=True,
        #     is_render_colours=True
        # )

        # Setup LIDAR
        lidar = Lidar(
            "lidar1",
            bng,
            car,
            requested_update_time=0.1,
            is_using_shared_memory=False,
            is_360_mode=True,
            dir=(0,-1,0),
            pos=(0,0,3.0),
        )

        car.connect(bng)

    #     RANGE_MIN = 0.1
    #     RANGE_MAX = 100.0
    #     RESOLUTION = (200, 200)
    #     FOV = 70
    #     radar = Radar(
    #     "radar1",
    #     bng,
    #     car,
    #     requested_update_time=0.01,
    #     pos=(0, 0, 1.7),
    #     dir=(0, -1, 0),
    #     up=(0, 0, 1),
    #     resolution=RESOLUTION,
    #     field_of_view_y=FOV,
    #     near_far_planes=(RANGE_MIN, RANGE_MAX),
    #     range_roundness=-2.0,
    #     range_cutoff_sensitivity=0.0,
    #     range_shape=0.23,
    #     range_focus=0.12,
    #     range_min_cutoff=0.5,
    #     range_direct_max_cutoff=RANGE_MAX,
    # )

        time.sleep(2)

        car.ai_set_mode('manual')

        print("Scenario loaded. Starting sensor polling...")
        time.sleep(2)

        try:
            index = 0
            while True:
                # scenario_node.publish_car_state(car.state)

                # Poll LIDAR data
                lidar_data = lidar.poll()
                if lidar_data and 'pointCloud' in lidar_data:
                    point_cloud = lidar_data['pointCloud']
                    scenario_node.publish_lidar_data(point_cloud)  # Pass car object
                    distances = np.sqrt(np.sum(point_cloud**2, axis=1))
                    valid_distances = distances[distances > 0]
                    
                    # if len(valid_distances) > 0:
                    #     print("\nLIDAR Data:")
                    #     print(f"- Valid points: {len(valid_distances)}")
                    #     print(f"- Min distance: {np.min(valid_distances):.2f}m")
                    #     print(f"- Max distance: {np.max(valid_distances):.2f}m")

                # Process ROS callbacks
                rclpy.spin_once(scenario_node, timeout_sec=0.01)
                
                time.sleep(0.1)
                # images = camera.poll()
                # if images and "colour" in images and images["colour"]:
                #     output_path = "vehicle_image_%03d.png" % index
                #     index += 1
                #     images["colour"].save(output_path)
                #     print(f"Saved image to: {output_path}")
                # else:
                #     print("Failed to capture image")
                
                # radar_data = radar.poll()

                # print(radar_data)

                time.sleep(1)
        except KeyboardInterrupt:
            lidar.remove()
            bng.disconnect()
            print("\nStopping sensor polling...")
        
        input("Press Enter to Stop")

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        print("\nDebug information:")
        print(f"- BNG_HOME: {beamng_home}")
        print(f"- Windows Host IP: {windows_ip}")
        print(f"- Home directory exists: {os.path.exists(beamng_home)}")
        print(f"- Binary directory exists: {os.path.exists(os.path.join(beamng_home, 'BinLinux'))}")
        print(f"- BeamNG settings directory exists: {os.path.exists(os.path.expanduser('~/.local/share/BeamNG.drive/0.34'))}")
        print("\nTroubleshooting:")
        print("1. Make sure BeamNG.tech is running on Windows")
        print("2. Check if Windows Defender Firewall is blocking the connection")
        print("3. Try adding an exception for BeamNG.tech in Windows Firewall")
        print("4. Try installing an older version of BeamNGpy: pip install beamngpy==1.24")
    finally:
        if 'bng' in locals() and bng:
            try:
                pass
            except:
                pass

if __name__ == "__main__":
    main()
