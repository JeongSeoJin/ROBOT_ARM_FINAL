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

    def execution_with_cycle(self, cycle):
        # while((GPIO.input(self.motor_stop))==1): # ==> 26
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

servo1 = servo(pin = 0) ## middle
servo2 = servo(pin = 1) ## wrist
servo3 = servo(pin = 2) ## grapper

def run_motor_cw():
    motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=7)
    motor1.execution()

def run_motor_ccw():
    motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CCW', speed=7)
    motor1.execution()

# Set frequency to 60hz, good for servos.
def go_home():
    global servo1, servo2, servo3
    print("To Home")
    run_motor_cw()

    servo3.execution(415)
    time.sleep(0.5)
    servo1.execution(175)
    servo2.execution(180)

def go_forward():
    global servo1, servo2, servo3
    print("Go forward")
    run_motor_ccw()

    servo1.execution(280)
    time.sleep(1)
    servo2.execution(300)
    time.sleep(1)
    servo3.execution(200)
    time.sleep(0.5)

def lift():
    global servo1, servo2, servo3
    print("lift")
    motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=7)
    motor1.execution_with_cycle(cycle = 1)

    servo1.execution(220)
    time.sleep(1)


# def upward():
#     global servo1, servo2, servo3
#     print("upward")
#     servo1.execution(330)
#     time.sleep(0.5)
#     motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=7)
#     motor1.execution_with_cycle(cycle = 1)

def go_up():
    global servo1, servo2, servo3
    print("go up")
    servo1.execution(500)
    time.sleep(0.5)
    servo2.execution(350)
    time.sleep(0.5)
    
    motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CCW', speed=7)
    motor1.execution_with_cycle(cycle = 1)
    # time.sleep(1)

    servo3.execution(415)
    time.sleep(0.5)

def head_up():
    global servo1, servo2, servo3
    time.sleep(0.5)
    servo1.execution(550)
    time.sleep(0.5)
    
    motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=7)
    motor1.execution_with_cycle(cycle = 2)
    time.sleep(0.5)
    
    servo1.execution(175)
    servo2.execution(230)




def servo_test_mode():
    global servo1, servo2, servo3
    question = int(input("[ 1 : Middle | 2 : wrist | 3 : grapper ] : "))
    if question == 1:
        servo1.execution(175)
        time.sleep(1)
        servo1.execution(450)
        time.sleep(1)
        servo1.execution(175)
    elif question == 2:
        servo2.execution(200)
        time.sleep(1)
        servo2.execution(300)
        time.sleep(1)
        servo2.execution(200)
    else:
        servo3.execution(415)
        time.sleep(1)
        servo3.execution(200)
        time.sleep(1)
        servo3.execution(415)


def main_go_home():
    question = input("[ 1 : Home | 2 : Go forward ] : ")
    if int(question) == 1:
        go_home()

    elif int(question) == 2:
        go_forward()
        time.sleep(1)
        lift()
        time.sleep(1)
        # upward()
        # time.sleep(1)
        go_up()
        time.sleep(1)
        head_up()
        time.sleep(1)
        go_home()



if __name__ == '__main__':
    while True:
        main_or_test = int(input(" [ main mode : 1 | test mode : 2 ] : "))
        if main_or_test == 1:
            main_go_home()
        else:
            servo_test_mode()