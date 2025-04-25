# Investigating sensor errors in Autonomous vehicles in collaboration with Siemens Innexis VSI and BeamNG

## Steps to run

  - go to ros2 workspace dir `cd ros2_ws`
  - Replace the file in `ros2_ws/src/beamng-ros2-integration/beamng_ros2/config/scenarios/example_tech_ground.json` with `example_tech_ground.json` from this repo
  - Re-build the ros package `colcon build --packages-select beamng_ros2`

  - Open BeamNG simulator
  - Launch Autoware e2e_simulator
      ```bash
      ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/test-map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
      ```

  - Launch Beamng-ros2-bridge
      ```bash
      ros2 run beamng_ros2 beamng_bridge
      ros2 param set /beamng_bridge host <WSL_IP>
      ros2 run beamng_ros2 example_client
      ```

  - Execute scenario.py
      ```bash
      python3 /path_to_cloned_repo/scenario.py
      ```
