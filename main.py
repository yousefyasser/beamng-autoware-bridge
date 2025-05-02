import rclpy, sys, os, subprocess, time, threading, numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Radar, State, Electrics, AdvancedIMU

from autowareBeamngBridge import AutowareBeamngBridge


class ScenarioRunner:
    def __init__(self):
        rclpy.init()
        beamng_home = os.getenv('BNG_HOME')

        if not beamng_home:
            print("Error: BNG_HOME environment variable not set!")
            sys.exit(1)

        windows_ip = self._get_windows_ip()

        input("Press Enter to start")
        print(f"Connecting to BeamNG.tech on {windows_ip}:25252...")

        try:
            self.bng = BeamNGpy(windows_ip, 25252, home=beamng_home)
            self.bng.open(launch=False)
            
            scenario = Scenario(
                # "smallgrid",
                "italy",
                "LiDAR_demo",
                description="Spanning the map with a LiDAR sensor",
            )

            self.car = Vehicle(
                'ego',
                model='etk800',
                licence='RED',
                color='Red'
            )
        
            scenario.add_vehicle(self.car, pos=(237.90,-894.42,246.10), rot_quat=(0.0173,-0.0019,-0.6354,0.7720))
            # scenario.add_vehicle(self.car, pos=(0,0,0), rot_quat=(0.0, 0.0, 0.15, 0.258382))
            
            scenario.make(self.bng)
            self.bng.load_scenario(scenario)
            self.bng.start_scenario()
            
            self._setup_sensors()
            self.car.connect(self.bng)
            self.car.ai_set_mode('manual')

            scenario_node = AutowareBeamngBridge(self.car, self.sensors[1])
            self.spin_thread = threading.Thread(target=rclpy.spin, args=(scenario_node,))
            self.spin_thread.start()

            print("Scenario loaded. Starting sensor polling...")

            try:
                index = 0

                while True:
                    lidar_data = self.sensors[0].poll()

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
                    # rclpy.spin_once(scenario_node, timeout_sec=0.01)

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

                    time.sleep(0.1)
            except KeyboardInterrupt:
                self.bng.disconnect()
                self.spin_thread.join()
                print("\nStopping sensor polling...")

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

    def _get_windows_ip(self):
        try:
            result = subprocess.run(['ip', 'route'], capture_output=True, text=True)
            for line in result.stdout.split('\n'):
                if 'default via' in line:
                    return line.split()[2]
        except:
            pass
        return '172.24.64.1'

    def _setup_sensors(self):
        lidar = Lidar(
            "lidar1",
            self.bng,
            self.car,
            requested_update_time=0.1,
            is_using_shared_memory=False,
            is_360_mode=True,
            dir=(0,-1,0),
            pos=(0,0,3.0),
        )

        imu = AdvancedIMU(
            "imu1",
            self.bng,
            self.car,
            is_send_immediately=True,
        )

        self.car.sensors.attach("electrics2", Electrics())

        self.sensors = [lidar, imu]

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

        # RANGE_MIN = 0.1
        # RANGE_MAX = 100.0
        # RESOLUTION = (200, 200)
        # FOV = 70

        # radar = Radar(
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


if __name__ == "__main__":
    ScenarioRunner()
