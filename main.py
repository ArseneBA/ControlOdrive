import odrive
from odrive.enums import *
from time import sleep


class Odrive:

    def __init__(self):
        print("Look for an odrive ...")
        self.odrv0 = odrive.find_any()
        print("Odrive found")

    def configuration(self):
        print("Configuration hoverboard style")

        # Test error DRV_FAULT
        self.odrv0.axis0.motor.config.motor_type = MotorType.HIGH_CURRENT

        # Configuration
        self.odrv0.config.enable_brake_resistor = True
        self.odrv0.config.brake_resistance = 3.5

        self.odrv0.axis0.motor.config.pole_pairs = 8

        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
        self.odrv0.axis0.motor.config.requested_current_range = 25  # Requires config save and reboot
        self.odrv0.axis0.motor.config.current_control_bandwidth = 100
        self.odrv0.axis0.motor.config.torque_constant = 0.21  # Not sure of this value

        self.odrv0.axis0.encoder.config.mode = EncoderMode.HALL
        self.odrv0.axis0.encoder.config.cpr = 6 * 8
        self.odrv0.axis0.encoder.config.calib_scan_distance = 150
        self.odrv0.config.gpio9_mode = GpioMode.DIGITAL
        self.odrv0.config.gpio10_mode = GpioMode.DIGITAL
        self.odrv0.config.gpio11_mode = GpioMode.DIGITAL

        self.odrv0.axis0.encoder.config.bandwidth = 100
        self.odrv0.axis0.controller.config.pos_gain = 1
        self.odrv0.axis0.controller.config.vel_gain = (0.02 * self.odrv0.axis0.motor.config.torque_constant
                                                      * self.odrv0.axis0.encoder.config.cpr)
        self.odrv0.axis0.controller.config.vel_integrator_gain = (0.1 * self.odrv0.axis0.motor.config.torque_constant *
                                                                 self.odrv0.axis0.encoder.config.cpr)
        self.odrv0.axis0.controller.config.vel_limit = 10

        """ 
        self.odrv0.axis0.encoder.config.pre_calibrated = True
        self.odrv0.axis0.motor.config.pre_calibrated = True

        self.odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.input_vel = 2
        """
        print("End configuration")

    def set_turn_s(self, turn_s):
        self.odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv0.axis0.controller.input_vel = turn_s
        self.odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL

    def get_values(self):
        print("pos_estimate", self.odrv0.axis0.encoder.pos_estimate)
        print("pos_estimate_counts", self.odrv0.axis0.encoder.pos_estimate_counts)
        print("pos_circular", self.odrv0.axis0.encoder.pos_circular)
        print("vel_estimate", self.odrv0.axis0.encoder.vel_estimate, "\n")


odrive_motor = Odrive()
# odrive_motor.configuration()
odrive_motor.set_turn_s(2)
n = 0
while n < 100:
    odrive_motor.get_values()
    sleep(0.1)
    n += 1
odrive_motor.odrv0.axis0.requested_state = AxisState.IDLE
