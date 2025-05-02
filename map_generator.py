from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Lidar
import numpy as np
import os
import subprocess
import time
import threading
import struct

class MapGenerator:
    def __init__(self):
        beamng_home = os.getenv('BNG_HOME')

        if not beamng_home:
            print("Error: BNG_HOME environment variable not set!")
            exit(1)

        windows_ip = self._get_windows_ip()

        input("Press Enter to start map generation")
        print(f"Connecting to BeamNG.tech on {windows_ip}:25252...")

        try:
            self.bng = BeamNGpy(windows_ip, 25252, home=beamng_home)
            self.bng.open(launch=False)
            
            # Create a scenario in the Italy map
            self.scenario = Scenario(
                "west_coast_usa",  # Use Italy map
                "LiDAR_map_generator",
                description="Generating a point cloud map with LiDAR",
            )

            self.car = Vehicle(
                'ego',
                model='etk800',
                licence='MAP',
                color='Blue'
            )
            
            # Starting position in Italy map - adjust as needed
            self.scenario.add_vehicle(self.car, pos=(-170.09, 517.71, 74.99),
            rot_quat=(-0.0046, 0.0095, 0.0074, 0.99))
            
            self.scenario.make(self.bng)
            self.bng.load_scenario(self.scenario)
            self.bng.start_scenario()
            
            # Setup LiDAR sensor
            self._setup_lidar()
            self.car.connect(self.bng)
            
            # Set to AI mode for automated driving
            self.car.ai_set_mode('manual')  # 'span' mode will drive around the map
                # Set a reasonable speed for mapping
            
            print("Scenario loaded. Starting point cloud collection...")
            
            # Initialize storage for collected point clouds
            self.all_points = []
            self.point_count = 0
            self.is_collecting = True
            
            # Start collection in a separate thread
            collection_thread = threading.Thread(target=self._collect_point_clouds)
            collection_thread.start()
            
            # Wait for user to stop collection
            input("Press Enter to stop collection and generate map...")
            self.is_collecting = False
            collection_thread.join()
            
            # Generate the PCD file
            self._generate_pcd_file()
            
            self.bng.disconnect()
            print("\nMap generation complete!")

        except Exception as e:
            print(f"Error: {e}")
            print("\nDebug information:")
            print(f"- BNG_HOME: {beamng_home}")
            print(f"- Windows Host IP: {windows_ip}")
            print("\nTroubleshooting:")
            print("1. Make sure BeamNG.tech is running on Windows")
            print("2. Check if Windows Defender Firewall is blocking the connection")
            print("3. Try adding an exception for BeamNG.tech in Windows Firewall")

    def _get_windows_ip(self):
        try:
            result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if 'default via' in line:
                    return line.split()[2]
        except:
            pass
        return '172.24.64.1'

    def _setup_lidar(self):
        # Create a high-resolution LiDAR for mapping
        self.lidar = Lidar(
            "lidar_mapper",
            self.bng,
            self.car,
            requested_update_time=0.1,
            is_using_shared_memory=False,
            is_360_mode=True,
            dir=(0, -1, 0),
            pos=(0, 0, 2.0)
        )

    def _collect_point_clouds(self):
        """Collect point clouds while driving around"""
        last_position = None
        min_distance_moved = 0.5  # Minimum distance in meters to move before capturing a new scan
        
        while self.is_collecting:
            # Get current vehicle position
            self.scenario.update()
            vehicle_state = self.car.state
            # print(vehicle_state)
            current_position = np.array([vehicle_state['pos'][0], vehicle_state['pos'][1], vehicle_state['pos'][2]])
            
            # Only capture new point cloud if we've moved enough
            if last_position is None or np.linalg.norm(current_position - last_position) > min_distance_moved:
                # Poll LiDAR data
                lidar_data = self.lidar.poll()
                
                if lidar_data and 'pointCloud' in lidar_data:
                    point_cloud = lidar_data['pointCloud']
                    
                    # Transform points to world coordinates
                    transformed_points = self._transform_to_world(point_cloud, vehicle_state)
                    
                    # Add to our collection
                    self.all_points.append(transformed_points)
                    self.point_count += len(transformed_points)
                    
                    print(f"Captured scan at position {current_position}. Total points: {self.point_count}")
                    
                    # Update last position
                    last_position = current_position
            
            time.sleep(0.1)

    def _transform_to_world(self, points, vehicle_state):
        """Transform points from vehicle coordinates to world coordinates"""
        # Extract vehicle position and rotation
        pos = np.array(vehicle_state['pos'])
        # Use 'rotation' instead of 'rot'
        rotation = vehicle_state['rotation']
        
        # Convert quaternion to Euler angles
        # BeamNG uses quaternion format [x, y, z, w]
        qx, qy, qz, qw = rotation
        
        # Convert quaternion to rotation matrix directly
        # This is more accurate than using Euler angles
        R = np.array([
            [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]
        ])
        
        # Transform each point
        transformed_points = []
        for point in points:
            # Rotate point
            rotated = R @ point
            # Translate point
            world_point = rotated + pos
            transformed_points.append(world_point)
        
        return np.array(transformed_points)

    def _generate_pcd_file(self):
        """Generate a PCD file from all collected point clouds"""
        if not self.all_points or len(self.all_points) == 0:
            print("No point cloud data collected!")
            return
        
        # Combine all point clouds
        combined_points = np.vstack(self.all_points)
        
        # Remove duplicate points (optional)
        # This can be computationally expensive for large point clouds
        print("Removing duplicate points...")
        combined_points = np.unique(combined_points, axis=0)
        
        # Optionally downsample if the point cloud is too large
        # if len(combined_points) > 2500000:
        #     print(f"Downsampling from {len(combined_points)} points...")
        #     # Simple random downsampling
        #     indices = np.random.choice(len(combined_points), 2500000, replace=False)
        #     combined_points = combined_points[indices]
        
        # Write to PCD file
        output_path = os.path.expanduser('~/autoware_map/italy/pointcloud_map.pcd')
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        print(f"Writing {len(combined_points)} points to {output_path}...")
        self._write_pcd_file(combined_points, output_path)
        print(f"PCD file created successfully at {output_path}")

    def _write_pcd_file(self, points, file_path):
        """Write points to a PCD file"""
        with open(file_path, 'w') as f:
            # Write PCD header
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write(f"VERSION 0.7\n")
            f.write(f"FIELDS x y z\n")
            f.write(f"SIZE 4 4 4\n")
            f.write(f"TYPE F F F\n")
            f.write(f"COUNT 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write(f"HEIGHT 1\n")
            f.write(f"VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write(f"DATA ascii\n")
            
            # Write points
            for point in points:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")

if __name__ == "__main__":
    MapGenerator()