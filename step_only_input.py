## this is an stepper_motor control with keyboard input

from time import sleep
import RPi.GPIO as GPIO     # import GPIO library
import time

# Import the PCA9685 module.
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_mid = 350
servo_max = 530  # Max pulse length out of 4096

class stepper:
    def __init__(self, ena_pin, dir_pin, pul_pin, step_angle, dire='CW', speed=2):

        self.enable = ena_pin
        self.dire = dir_pin
        self.pulse = pul_pin
        self.step_angle = step_angle
        self.motor_stop = 26        # when this pin is low at time motor is pause/stop
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dire, GPIO.OUT)
        GPIO.setup(self.enable, GPIO.OUT)
        GPIO.setup(self.pulse, GPIO.OUT)
        GPIO.setup(self.motor_stop, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setwarnings(False)

        self.puls = (360.0 / step_angle) ## ==> 48
        self.rps = speed # ==> input as 10
        self.pulse_in = self.rps * self.puls # ==> 480
        self.delay = (1 / (2 * self.pulse_in)) # ==> 1/960  approximately 0.001
        GPIO.output(self.enable, GPIO.HIGH)

        if dire == 'CW':
            GPIO.output(self.dire, 1)
        else:
            GPIO.output(self.dire, 0)

    def execution(self):
        # while((GPIO.input(self.motor_stop))==1): # ==> 26
        cycle = int(input("Cycle : "))
        GPIO.output(self.enable, GPIO.HIGH)
        for x in range(400 * cycle): # ==> 960         2 * int(self.pulse_in)
            GPIO.output(self.pulse, GPIO.HIGH)
            sleep(self.delay)
            GPIO.output(self.pulse, GPIO.LOW)
            sleep(self.delay)

        ## just value to confirm
        print(self.puls, self.rps, self.pulse_in, self.delay)
        print(self.motor_stop, 2 * int(self.pulse_in))

class servo:
    def __init__(self, pin):
        print("Servo difinition")
        self.pin = pin
        pwm.set_pwm_freq(50)

    def execution(self, value):
        pwm.set_pwm(self.pin, 0, value) #min : 150 max : 530

# def step_only():
#     direction = input("Input cw | ccw : ")
#     if direction == "cw":
#         print("motor has been set as CW")
#         # sleep(4)
#         motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=7)
#         motor1.execution()
#     else:
#         print("motor has been set as CCW")
#         motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CCW', speed=7)
#         motor1.execution()

# Set frequency to 60hz, good for servos.

def execute_stepper():
    # servo definition
    servo1 = servo(pin = 0)
    servo2 = servo(pin = 1)
    servo3 = servo(pin = 3)

    question = input("[ 1 : CW | 2 : CCW ] : ")
    if int(question) == 1:
        print("set as CW")
        motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=7)
        motor1.execution()

        # servo3.execution(415)
        # time.sleep(0.5)
        # servo1.execution(175)
        # servo2.execution(270)

    elif int(question) == 2:
        print("set as CCW")
        motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CCW', speed=7)
        motor1.execution()

        # servo1.execution(450)
        # servo2.execution(340)
        # time.sleep(0.5)
        # servo3.execution(130)


# if __name__ == '__main__':
#     while 1:
#         motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=10)
#         # motor1.execution()
#         main()

if __name__ == '__main__':
    while True:
        execute_stepper()