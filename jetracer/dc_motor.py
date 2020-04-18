import atexit
from Adafruit_MotorHAT import Adafruit_MotorHAT
import traitlets
from traitlets.config.configurable import Configurable

class DcMotor(Configurable):
    value = traitlets.Float()
    def __init__(self, driver, channel, *args, **kwargs):
        super(DcMotor, self).__init__(*args, **kwargs)  # initializes traitlets
        self._driver = driver
        self._motor = self._driver.getMotor(channel)
        atexit.register(self._release)
        
    @traitlets.observe('value')
    def _observe_value(self, change):
        self._write_value(change['new'])

    def _write_value(self, value):
        """Sets motor value between [-1, 1]"""
        #mapped_value = int(255.0 * (self.alpha * value + self.beta))
        mapped_value = int(255.0 * value)
        speed = min(max(abs(mapped_value), 0), 255)
        self._motor.setSpeed(speed)
        if mapped_value < 0:
            self._motor.run(Adafruit_MotorHAT.FORWARD)
        else:
            self._motor.run(Adafruit_MotorHAT.BACKWARD)

    def _release(self):
        """Stops motor by releasing control"""
        self._motor.run(Adafruit_MotorHAT.RELEASE)