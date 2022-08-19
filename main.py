""" Example on how to use the Motor.py code."""

from Motor import *


motor = OdriveEncoderHall()
motor.erase_configuration()
motor.configuration()
motor.calibration()
