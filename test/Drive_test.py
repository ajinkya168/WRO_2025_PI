import os
import pigpio



os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')

class RotaryEncoder:
    def __init__(self, pi, gpioA, gpioB, callback=None):
        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.callback = callback
        self.last_gpio = None
        self.levA = 0
        self.levB = 0
        self.count = 0

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)
        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.last_gpio:
            self.last_gpio = gpio
            if gpio == self.gpioA and level == 1 and self.levB == 1:
                self._callback(1)
            elif gpio == self.gpioB and level == 1 and self.levA == 1:
                self._callback(-1)

    def _callback(self, direction):
        self.count += direction
        if self.callback is not None:
            self.callback(direction)

    def get_count(self):
        return self.count

    def reset_count(self):
        self.count = 0

    def cancel(self):
        self.cbA.cancel()
        self.cbB.cancel()

def rotary_callback(direction):
    print("Rotated", "Right" if direction > 0 else "Left")


pwm = pigpio.pi()


ENCODER_A = 17
ENCODER_B = 18
power = 30
prev_power = 0

encoder = RotaryEncoder(pwm, ENCODER_A, ENCODER_B, rotary_callback)

pwm.set_mode(12, pigpio.OUTPUT)  # Set pin 12 as an output
pwm.set_mode(20, pigpio.OUTPUT)  # Set pin 20 as an output
pwm.hardware_PWM(12, 100, 0)
pwm.set_PWM_dutycycle(12, 0)  # Set duty cycle to 50% (128/255)

total_power = 0
prev_power = 0
power = int(input("Enter power: "))

try:
	while True:
			total_power = (power * 0.1) + (prev_power * 0.9)
			prev_power = total_power
			pwm.set_PWM_dutycycle(12, 2.55 * total_power)  # Set duty cycle to 50% (128/255)
			print("Current count:", encoder.get_count())
except KeyboardInterrupt:
	total_power = 0
	prev_power = 0
	pwm.set_PWM_dutycycle(12, 2.55 * total_power)  # Set duty cycle to 50% (128/255)
