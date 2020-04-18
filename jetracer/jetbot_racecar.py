from .racecar import Racecar
import traitlets
import numpy as np
from math import degrees
from Adafruit_MotorHAT import Adafruit_MotorHAT
from .dc_motor import DcMotor


def throttle_angle_to_thrust(r, theta):
        """ Assumes theta in degrees and r = 0 to 100 %
            returns a tuple of percentages: (left_thrust, right_thrust)"""
        theta = ((theta + 180) % 360) - 180  # normalize value to [-180, 180)
        r = min(max(0, r), 100)              # normalize value to [0, 100]
        v_a = r * (45 - theta % 90) / 45          # falloff of main motor
        v_b = min(100, 2 * r + v_a, 2 * r - v_a)  # compensation of other motor
        if theta < -90: return -v_b, -v_a
        if theta < 0:   return -v_a, v_b
        if theta < 90:  return v_b, v_a
        return v_a, -v_b

class JetbotRacecar(Racecar):
    
    steering_gain = traitlets.Float(default_value=-0.65)
    steering_offset = traitlets.Float(default_value=0)
    throttle_gain = traitlets.Float(default_value=0.8)
    
    i2c_bus = traitlets.Integer(default_value=1).tag(config=True)
    left_motor_channel = traitlets.Integer(default_value=1).tag(config=True)
    right_motor_channel = traitlets.Integer(default_value=2).tag(config=True)
    
    def __init__(self, *args, **kwargs):
        super(JetbotRacecar, self).__init__(*args, **kwargs)
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=self.i2c_bus)
        self.left_motor = DcMotor(self.motor_driver, channel=self.left_motor_channel)
        self.right_motor = DcMotor(self.motor_driver, channel=self.right_motor_channel)
    
    def _move_car(self):
        """Moves car with 2 wheels (left, right)"""
        value_y = self.steering * self.steering_gain + self.steering_offset
        value_x = self.throttle * self.throttle_gain
        
        # convert steering and throttle to left/right thrust values
        theta = np.arctan2(value_y, value_x)
        radial_x = abs(value_x) * np.cos(theta)
        radial_y = abs(value_y) * np.sin(theta)
        radius = np.sqrt(pow(radial_x, 2) + pow(radial_y, 2))*100.
        theta_radial_degrees = degrees(np.arctan2(radial_y, radial_x))
        left_thrust, right_thrust = throttle_angle_to_thrust(radius, theta_radial_degrees)
        
        # set motors values
        self.left_motor.value = float(left_thrust/100.)
        self.right_motor.value = float(right_thrust/100.)
    
    @traitlets.observe('steering')
    def _on_steering(self, change):
        self._move_car()
    
    @traitlets.observe('throttle')
    def _on_throttle(self, change):
        self._move_car()
        
    @traitlets.observe('throttle_gain')
    def _on_throttle_gain(self, change):
        self._move_car()
        
    @traitlets.observe('steering_gain')
    def _on_steering_gain(self, change):
        self._move_car()   
    
    @traitlets.observe('steering_offset')
    def _on_steering_offset(self, change):
        self._move_car()   