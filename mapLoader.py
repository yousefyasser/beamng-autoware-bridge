import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import os
import struct
import time
from std_msgs.msg import Header

class MapLoader(Node):
    def __init__(self):
        super().__init__('map_loader')
        
        # Create QoS profile with TRANSIENT_LOCAL durability
        # This is typically what Autoware expects for map data
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create publisher for the map with the QoS profile
        self.map_publisher = self.create_publisher(
            PointCloud2, '/map/pointcloud_map', qos
        )
        
        # Path to the PCD map file - adjust this to your actual map location
        # This should be the same map used by Autoware
        self.map_path = os.environ.get('AUTOWARE_MAP_PATH', 
                                      os.path.expanduser('~/autoware_map/sample-map-planning/sample-map-planning/pointcloud_map.pcd'))
        
        self.get_logger().info(f"Loading map from: {self.map_path}")
        
        # Store the loaded points in memory
        self.map_points = None
        
        # Load the map once at startup
        self.load_map()
        
        # Create a timer to periodically republish the map (without reloading)
        self.create_timer(10.0, self.publish_map)
        
    def load_map(self):
        """Load PCD file and store points in memory"""
        try:
            # Check if file exists
            if not os.path.exists(self.map_path):
                self.get_logger().error(f"Map file not found: {self.map_path}")
                return
                
            # Load the PCD file
            self.map_points = self.load_pcd(self.map_path)
            if self.map_points is None or len(self.map_points) == 0:
                self.get_logger().error("Failed to load points from PCD file")
                return
                
            self.get_logger().info(f"Successfully loaded map with {len(self.map_points)} points")
            
            # Publish the map immediately after loading
            self.publish_map()
            
        except Exception as e:
            self.get_logger().error(f"Error loading map: {str(e)}")
    
    def publish_map(self):
        """Publish the stored map points as PointCloud2"""
        if self.map_points is None:
            self.get_logger().warn("No map points available to publish")
            return
            
        try:
            # Create and publish the PointCloud2 message
            msg = self.create_cloud_message(self.map_points)
            self.map_publisher.publish(msg)
            self.get_logger().info(f"Published map with {len(self.map_points)} points")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing map: {str(e)}")
    
    def load_pcd(self, file_path):
        """Load a PCD file and return the points as numpy array"""
        try:
            with open(file_path, 'rb') as f:
                # Initialize variables with default values
                width = 0
                fields = []
                sizes = []
                data_format = 'binary'  # Default to binary format
                
                # Skip header until DATA line
                line = f.readline().decode().strip()
                while line and not line.startswith('DATA'):
                    # Check for WIDTH field to get point count
                    if line.startswith('WIDTH'):
                        width = int(line.split()[1])
                    # Check for FIELDS to understand data format
                    elif line.startswith('FIELDS'):
                        fields = line.split()[1:]
                    # Check for SIZE to understand data size
                    elif line.startswith('SIZE'):
                        sizes = [int(s) for s in line.split()[1:]]
                    # Check for data format (ascii or binary)
                    elif line.startswith('DATA'):
                        data_format = line.split()[1]
                    
                    # Read next line
                    line = f.readline().decode().strip()
                    if not line:  # End of file reached without finding DATA line
                        self.get_logger().error("PCD file format error: No DATA line found")
                        return None
                
                # If we found a DATA line, extract the format
                if line.startswith('DATA'):
                    data_format = line.split()[1]
                
                # For binary data
                if data_format == 'binary':
                    # Read all binary data
                    data = f.read()
                    
                    # Calculate point size in bytes
                    if not sizes:
                        self.get_logger().error("PCD file format error: No SIZE information found")
                        return None
                    
                    point_size = sum(sizes)
                    
                    # Parse points
                    points = []
                    for i in range(0, len(data), point_size):
                        if i + 12 <= len(data):  # Ensure we have at least x,y,z (3 floats)
                            x = struct.unpack('f', data[i:i+4])[0]
                            y = struct.unpack('f', data[i+4:i+8])[0]
                            z = struct.unpack('f', data[i+8:i+12])[0]
                            points.append([x, y, z])
                    
                    return np.array(points)
                
                # For ASCII data
                elif data_format == 'ascii':
                    points = []
                    for line in f:
                        values = line.decode().strip().split()
                        if len(values) >= 3:  # Ensure we have at least x,y,z
                            points.append([float(values[0]), float(values[1]), float(values[2])])
                    
                    return np.array(points)
                
                else:
                    self.get_logger().error(f"Unsupported PCD format: {data_format}")
                    return None
                    
        except Exception as e:
            self.get_logger().error(f"Error loading PCD file: {str(e)}")
            return None
    
    def create_cloud_message(self, points):
        """Create a PointCloud2 message from numpy array of points"""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Get number of points
        num_points = len(points)
        
        # Create a byte array to hold all point data
        # 16 bytes per point (x,y,z,intensity as float32)
        buffer = bytearray(num_points * 16)
        
        # For each point, manually pack the data into the buffer
        for i, point in enumerate(points):
            # Calculate buffer offset for this point
            offset = i * 16
            
            # Pack XYZ as float32 (4 bytes each)
            x_bytes = np.float32(point[0]).tobytes()
            y_bytes = np.float32(point[1]).tobytes()
            z_bytes = np.float32(point[2]).tobytes()
            intensity_bytes = np.float32(255).tobytes()  # Default intensity
            
            # Copy bytes to buffer
            buffer[offset:offset+4] = x_bytes
            buffer[offset+4:offset+8] = y_bytes
            buffer[offset+8:offset+12] = z_bytes
            buffer[offset+12:offset+16] = intensity_bytes
        
        # Set up the PointCloud2 message
        msg.height = 1
        msg.width = num_points
        
        # Define fields with correct offsets
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16  # Total size of one point in bytes
        msg.row_step = msg.point_step * num_points
        msg.data = bytes(buffer)  # Convert bytearray to bytes
        msg.is_dense = True
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    map_loader = MapLoader()
    
    try:
        rclpy.spin(map_loader)
    except KeyboardInterrupt:
        pass
    finally:
        map_loader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()