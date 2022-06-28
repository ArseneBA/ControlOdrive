import odrive
from odrive.enums import *
import math
from time import sleep

""" encoder_is_present = False

print("Look for an odrive ...")
odrv0 = odrive.find_any()
print("Odrive found")
# Current limit
odrv0.axis0.motor.config.current_lim = 10
# Velocity limit, for safety reason set to 2
odrv0.axis0.controller.config.vel_limit = 2
# Calibration current, small motor - need to reduce it
odrv0.axis0.motor.config.calibration_current = 10
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
odrv0.axis0.motor.config.pole_pairs = 8
# Torque constant, torque produced by the motor per Amp of current delivered
odrv0.axis0.motor.config.torque_constant = 0.21
# Motor type,
odrv0.axis0.motor.config.motor_type = MotorType.HIGH_CURRENT
# Encoder, Hall effect encoders
# cpr = 6* pole pairs
odrv0.axis0.encoder.config.mode = EncoderMode.HALL
odrv0.axis0.encoder.config.cpr = 6 * 8
# ERROR_ENCODER_CPR_POLEPAIRS_MISMATCH
odrv0.axis0.encoder.config.calib_scan_distance = 8 * 2 * 2 * math.pi
odrv0.axis0.encoder.config.calib_range = 0.05
 """
""" print("Hall polarity calibration")
odrv0.axis0.requested_state = AxisState.ENCODER_HALL_POLARITY_CALIBRATION
sleep(10)
print("Hall polarity calibration done, error:")
print(odrv0.axis0.encoder.error)

print("Encoder offset calibration")
odrv0.axis0.requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
sleep(10)
print("done, error:")
print(odrv0.axis0.encoder.error)

# Start the calibration
print("Start Calibration")
odrv0.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
"""


print("Look for an odrive ...")
odrv0 = odrive.find_any()
print("Odrive found")

odrv0.config.enable_brake_resistor = True
odrv0.config.brake_resistance = 3000

odrv0.axis0.motor.config.pole_pairs = 8
odrv0.axis0.motor.config.resistance_calib_max_voltage = 4
odrv0.axis0.motor.config.requested_current_range = 25 #Requires config save and reboot
odrv0.axis0.motor.config.current_control_bandwidth = 100
odrv0.axis0.motor.config.torque_constant = 0.21

odrv0.axis0.encoder.config.mode = EncoderMode.HALL
odrv0.axis0.encoder.config.cpr = 6 * 8
odrv0.axis0.encoder.config.calib_scan_distance = 150
odrv0.config.gpio9_mode = GpioMode.DIGITAL
odrv0.config.gpio10_mode = GpioMode.DIGITAL
odrv0.config.gpio11_mode = GpioMode.DIGITAL

odrv0.axis0.encoder.config.bandwidth = 100
odrv0.axis0.controller.config.pos_gain = 1
odrv0.axis0.controller.config.vel_gain = 0.02 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis0.controller.config.vel_integrator_gain = 0.1 * odrv0.axis0.motor.config.torque_constant * odrv0.axis0.encoder.config.cpr
odrv0.axis0.controller.config.vel_limit = 10
odrv0.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL

 