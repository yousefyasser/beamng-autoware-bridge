import numpy as np

from std_msgs.msg import Header
from tier4_vehicle_msgs.msg import ActuationCommandStamped, ActuationStatusStamped
from autoware_vehicle_msgs.msg import SteeringReport, VelocityReport, ControlModeReport, GearReport

class VehicleStatusPublisher:
    def __init__(self, clock, electrics, state, publishers):
        self.clock = clock
        self.electrics = electrics
        self.state = state

        self.pub_steering_state = publishers['steering_state']
        self.pub_ctrl_mode = publishers['control_mode']
        self.pub_vel_state = publishers['velocity_state']
        self.pub_gear_state = publishers['gear_state']
        self.pub_actuation_status = publishers['actuation_status']

    def publish(self, clock, electrics, state):
        self.clock = clock
        self.state = state
        self.electrics = electrics
    
        self.publish_velocity_status()
        self.publish_control_mode()
        self.publish_gear_status()
        self.publish_actuation_status()
        self.publish_steering_status()

    def publish_velocity_status(self):
        header = Header()
        header.frame_id = 'base_link'
        header.stamp = self.clock

        dir_vector = self.state['dir']
        vel_vector = self.state['vel']
        dir_norm = dir_vector / np.linalg.norm(dir_vector)

        longitudinal_vel = np.dot(vel_vector, dir_norm)

        lateral_dir = np.array([-dir_norm[1], dir_norm[0], 0])
        lateral_vel = np.dot(vel_vector, lateral_dir)


        output_velocity_report = VelocityReport()
        output_velocity_report.heading_rate = 0.0
        output_velocity_report.longitudinal_velocity = longitudinal_vel
        output_velocity_report.lateral_velocity = lateral_vel 

        output_velocity_report.header = header
        self.pub_vel_state.publish(output_velocity_report)

    def publish_control_mode(self):
        control_mode_msg = ControlModeReport()
        control_mode_msg.stamp = self.clock
        control_mode_msg.mode = ControlModeReport.AUTONOMOUS

        self.pub_ctrl_mode.publish(control_mode_msg)

    def publish_gear_status(self):
        gear_map = {'N': 1, 'R': 20, 'D': 2}

        out_gear_state = GearReport()
        out_gear_state.stamp = self.clock
        out_gear_state.report = gear_map.get(self.electrics['gear'], 0)

        self.pub_gear_state.publish(out_gear_state)

    def publish_actuation_status(self):
        out_actuation_status = ActuationStatusStamped()
        
        out_actuation_status.header.frame_id = 'base_link'
        out_actuation_status.header.stamp = self.clock
        out_actuation_status.status.accel_status = self.electrics['throttle_input']
        out_actuation_status.status.brake_status = self.electrics['brake_input']
        out_actuation_status.status.steer_status = self.electrics['steering_input']

        self.pub_actuation_status.publish(out_actuation_status)

    def publish_steering_status(self):
        out_steering_state = SteeringReport()
        out_steering_state.stamp = self.clock
        out_steering_state.steering_tire_angle = self.electrics['steering_input'] * 0.7 * -1

        self.pub_steering_state.publish(out_steering_state)