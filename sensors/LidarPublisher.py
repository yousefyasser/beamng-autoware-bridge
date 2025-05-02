from sensor_msgs.msg import PointCloud2, PointField
from beamngpy.sensors import Lidar
import time
import numpy as np
from rclpy.node import Node

from utils.get_header import get_header

class LidarPublisher(Node):
    def __init__(self, bng, car):
        super().__init__('lidar_publisher')

        self.lidar_top = Lidar(
            "top",
            bng,
            car,
            requested_update_time=0.02,
            is_using_shared_memory=False,
            is_360_mode=True,
            # vertical_resolution=32,
            # horizontal_angle=360,
            # is_rotate_mode=False,
            # is_visualised=False,
            # is_streaming=True,
            # is_dir_world_space=False,
            dir=(0, -1, 0),
            pos=(-0.1, 0.65, 2.0),
        )
        # self.lidar_right = Lidar(
        #     "right",
        #     bng,
        #     car,
        #     requested_update_time=0.02,
        #     is_using_shared_memory=True,
        #     is_360_mode=True,
        #     vertical_resolution=32,
        #     horizontal_angle=360,
        #     is_rotate_mode=False,
        #     is_visualised=False,
        #     is_streaming=True,
        #     is_dir_world_space=False,
        #     dir=(0, -1, 0),
        #     pos=(0, 0.65, 2.0),
        # )

        # self.lidar_left = lidar = Lidar(
        #     "left",
        #     bng,
        #     car,
        #     requested_update_time=0.02,
        #     is_using_shared_memory=True,
        #     is_360_mode=True,
        #     vertical_resolution=32,
        #     horizontal_angle=360,
        #     is_rotate_mode=False,
        #     is_visualised=False,
        #     is_streaming=True,
        #     is_dir_world_space=False,
        #     dir=(0, -1, 0),
        #     pos=(0, 0.65, 2.0),
        # )

        self.pub_lidar_top = self.create_publisher(
            PointCloud2, '/localization/util/downsample/pointcloud', 10
        )
        # self.pub_lidar_right = self.create_publisher(
        #     PointCloud2, '/sensing/lidar/points', 10
        # )
        # self.pub_lidar_left = self.create_publisher(
        #     PointCloud2, '/sensing/lidar/left/aw_points', 10
        # )

    def collect_pcd(self, stop_event):
        self.get_logger().info('\033[92m' + "Lidar Publisher started\n")

        while not stop_event.is_set() :
            lidar_data_top = self.lidar_top.poll()
            # lidar_data_right = self.lidar_right.poll()
            # lidar_data_left = self.lidar_left.poll()

            if lidar_data_top and 'pointCloud' in lidar_data_top:
                point_cloud = lidar_data_top['pointCloud']
                self.publish(point_cloud, self.pub_lidar_top, 'velodyne_top_changed')
                # self.publish(point_cloud, self.pub_lidar_right, 'velodyne_right_changed')
                # self.publish(point_cloud, self.pub_lidar_left, 'velodyne_left_changed')

            # if lidar_data_right and 'pointCloud' in lidar_data_right:
            #     point_cloud = lidar_data_right['pointCloud']
            #     self.publish(point_cloud, self.pub_lidar_right, 'velodyne_right_changed')
            
            # if lidar_data_left and 'pointCloud' in lidar_data_left:
            #     point_cloud = lidar_data_left['pointCloud']
            #     self.publish(point_cloud, self.pub_lidar_left, 'velodyne_left_changed')

            time.sleep(0.1)

    def publish(self, point_cloud, publisher, frame_id):
        msg = PointCloud2()
        msg.header = get_header(frame_id)

        num_points = len(point_cloud)
        
        buffer = bytearray(num_points * 32)
        
        for i, point in enumerate(point_cloud):
            offset = i * 32
            
            x_bytes = np.float32(point[0]).tobytes()
            y_bytes = np.float32(point[1]).tobytes()
            z_bytes = np.float32(point[2]).tobytes()
            
            buffer[offset:offset+4] = x_bytes
            buffer[offset+4:offset+8] = y_bytes
            buffer[offset+8:offset+12] = z_bytes
            
            buffer[offset+12] = 100
            
            buffer[offset+13] = 0
            
            channel_bytes = np.uint16(0).tobytes()
            buffer[offset+14:offset+16] = channel_bytes
            
            azimuth = np.float32(np.arctan2(point[1], point[0])).tobytes()
            buffer[offset+16:offset+20] = azimuth
            
            xy_dist = np.sqrt(point[0]**2 + point[1]**2)
            elevation = np.float32(np.arctan2(point[2], xy_dist)).tobytes()
            buffer[offset+20:offset+24] = elevation
            
            distance = np.float32(np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)).tobytes()
            buffer[offset+24:offset+28] = distance
            
            timestamp = np.float32(self.get_clock().now().nanoseconds / 1e9).tobytes()
            buffer[offset+28:offset+32] = timestamp
        
        msg.height = 1
        msg.width = num_points
        
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
        msg.point_step = 32 
        msg.row_step = msg.point_step * num_points
        msg.data = bytes(buffer)
        msg.is_dense = True
        
        publisher.publish(msg)