import rclpy, sys, os, subprocess, time, threading, numpy as np

from beamngpy import BeamNGpy, Scenario, Vehicle
from beamngpy.sensors import Camera, Lidar, Radar, State, Electrics, AdvancedIMU

from autowareBeamngBridge import AutowareBeamngBridge
from sensors.LidarPublisher import LidarPublisher


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

            print('\033[95m' + "================= Scenario loaded ===================\n")

            stop_event = threading.Event()
            stop_thread = threading.Thread(target=lambda: (input("Press Enter to stop...\n"), stop_event.set()))
            stop_thread.start()

            scenario_node = AutowareBeamngBridge(self.car, self.sensors[0])
            lidar_node = LidarPublisher(self.bng, self.car)

            scenario_thread = threading.Thread(target=scenario_node.start_publishers, args=(stop_event,))
            scenario_thread.start()

            lidar_thread = threading.Thread(target=lidar_node.collect_pcd, args=(stop_event,))
            lidar_thread.start()

            stop_thread.join()
            scenario_thread.join()
            lidar_thread.join()

            print('\033[91m' + "================= Scenario stopped ===================\n")
            self.bng.disconnect()


        except Exception as e:
            print(f"Error: {e}", file=sys.stderr)

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
        imu = AdvancedIMU(
            "imu1",
            self.bng,
            self.car,
            is_send_immediately=True,
        )

        self.car.sensors.attach("electrics2", Electrics())

        self.sensors = [imu]

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
