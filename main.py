import odrive
from odrive.enums import *
import math
from time import sleep

""" encoder_is_present = False

print("Look for an odrive ...")
odrv0 = odrive.find_any()
print("Odrive found")
# Current limit
odrv0.axis1.motor.config.current_lim = 10
# Velocity limit, for safety reason set to 2
odrv0.axis1.controller.config.vel_limit = 2
# Calibration current, small motor - need to reduce it
odrv0.axis1.motor.config.calibration_current = 10
# Enabling brake resistor, set to True if used
odrv0.config.enable_brake_resistor = True
# If True, give the value of the brake resistor in ohms
odrv0.config.brake_resistance = 3000
# Negative Current, if no brake resistor is used
# This is the amount of current [Amps] allowed to flow back into the power supply.
# The convention is that it is negative. By default, it is set to a conservative value of 10mA.
# If you are using a brake resistor and getting DC_BUS_OVER_REGEN_CURRENT errors, raise it slightly.
# If you are not using a brake resistor and you intend to send braking current back to the power supply,
# set this to a safe level for your power source.
# odrv0.config.dc_max_negative_current
# Pole Pairs, on en compte 16
# Number of magnet poles in the rotor divided by two.
odrv0.axis1.motor.config.pole_pairs = 8
# Torque constant, torque produced by the motor per Amp of current delivered
odrv0.axis1.motor.config.torque_constant = 0.21
# Motor type,
odrv0.axis1.motor.config.motor_type = MotorType.HIGH_CURRENT
# Encoder, Hall effect encoders
# cpr = 6* pole pairs
odrv0.axis1.encoder.config.mode = EncoderMode.HALL
odrv0.axis1.encoder.config.cpr = 6 * 8
 """



""" print("Hall polarity calibration")
odrv0.axis1.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION
sleep(10)
print("Hall polarity calibration done, error:")
print(odrv0.axis1.encoder.error)

print("Encoder offset calibration")
odrv0.axis1.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
sleep(10)
print("done, error:")
print(odrv0.axis1.encoder.error)

# Start the calibration
print("Start Calibration")
odrv0.axis1.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
 """
class Odrive:

    def __init__(self):
        print("Look for an odrive ...")
        self.odrv0 = odrive.find_any()
        print("Odrive found")

    def configuration(self):
        print("Configuration hoverboard style")

        # Test erreur DRV_FAULT
        self.odrv0.axis1.motor.config.motor_type = MotorType.HIGH_CURRENT


        # Configuration
        self.odrv0.config.enable_brake_resistor = True
        self.odrv0.config.brake_resistance = 3.5

        self.odrv0.axis1.motor.config.pole_pairs = 8 * 42 # Réduction de 41.8:1 x5 pour arriver sur un nombre entier

        self.odrv0.axis1.motor.config.calibration_current = 10
        self.odrv0.axis1.motor.config.resistance_calib_max_voltage = 20
        self.odrv0.axis1.motor.config.requested_current_range = 25 #Requires config save and reboot
        self.odrv0.axis1.motor.config.current_control_bandwidth = 100
        self.odrv0.axis1.motor.config.torque_constant = 0.21  # Not sure of this value

        """ self.odrv0.axis1.encoder.config.mode = EncoderMode.HALL
        self.odrv0.axis1.encoder.config.cpr = 6 * 8
        self.odrv0.axis1.encoder.config.calib_scan_distance = 150 """

        self.odrv0.axis1.encoder.config.mode = EncoderMode.INCREMENTAL
        self.odrv0.axis1.encoder.config.cpr = 24000 # Multiplier par 5 car on multiple par 5 pp pour avoir un nombre entier
        self.odrv0.axis1.encoder.config.calib_range = 0.05


        # self.odrv0.axis1.encoder.config.use_index = True

        self.odrv0.config.gpio9_mode = GpioMode.DIGITAL
        self.odrv0.config.gpio10_mode = GpioMode.DIGITAL
        self.odrv0.config.gpio11_mode = GpioMode.DIGITAL

        # self.odrv0.axis1.encoder.config.bandwidth = 100
        self.odrv0.axis1.controller.config.pos_gain = 1
        self.odrv0.axis1.controller.config.vel_gain = 0.02 * self.odrv0.axis1.motor.config.torque_constant * self.odrv0.axis1.encoder.config.cpr
        self.odrv0.axis1.controller.config.vel_integrator_gain = 0.1 * self.odrv0.axis1.motor.config.torque_constant * self.odrv0.axis1.encoder.config.cpr
        self.odrv0.axis1.controller.config.vel_limit = 0.1

        """ 
        self.odrv0.axis1.encoder.config.pre_calibrated = True
        self.odrv0.axis1.motor.config.pre_calibrated = True

        self.odrv0.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL
        self.odrv0.axis1.controller.input_vel = 2
        """
        print("End configuration")

    def set_turn_s(self, turn_s):
        self.odrv0.axis1.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.odrv0.axis1.controller.input_vel = -0.001
        self.odrv0.axis1.requested_state = AxisState.CLOSED_LOOP_CONTROL

    def get_values(self):
        print("pos_estimate", self.odrv0.axis1.encoder.pos_estimate)
        print("pos_estimate_counts", self.odrv0.axis1.encoder.pos_estimate_counts)
        print("pos_circular", self.odrv0.axis1.encoder.pos_circular)
        print("vel_estimate", self.odrv0.axis1.encoder.vel_estimate, "\n")


odrive_motor = Odrive()
odrive_motor.configuration()
""" odrive_motor.set_turn_s(2)
n = 0
while n < 100:
    odrive_motor.get_values()
    sleep(0.1)
    n += 1
odrive_motor.odrv0.axis1.requested_state = AxisState.IDLE """
