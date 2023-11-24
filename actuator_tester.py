from time import sleep
import RPi.GPIO as GPIO    
import time
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()


################ stepper motor definition ####################

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

    def execution(self, cycle):
        # while((GPIO.input(self.motor_stop))==1): # ==> 26
        GPIO.output(self.enable, GPIO.HIGH)
        for x in range(100 * cycle): # ==> 960         2 * int(self.pulse_in)
            GPIO.output(self.pulse, GPIO.HIGH)
            sleep(self.delay)
            GPIO.output(self.pulse, GPIO.LOW)
            sleep(self.delay)

        ## just value to confirm
        print(self.puls, self.rps, self.pulse_in, self.delay)
        print(self.motor_stop, 2 * int(self.pulse_in))

def run_motor_cw(cycle_value):
    motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CW', speed=7)
    motor1.execution(cycle_value)

def run_motor_ccw(cycle_value):
    motor1 = stepper(ena_pin=16, dir_pin=20, pul_pin=21, step_angle=7.5, dire='CCW', speed=7)
    motor1.execution(cycle_value)

################# servo definition ##########################

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_mid = 350
servo_max = 530  # Max pulse length out of 4096

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


################### control function #####################

if __name__ == '__main__':
    while True:
    ##setting
        value1 = int(input("servo1(middle) : "))
        value2 = int(input("servo2(wrist) : "))
        value3 = int(input("servo3(grap) : "))
        value4 = input("stepper motor direction : ")
        value5 = int(input("cycle : "))

        servo1.execution(value1)
        servo2.execution(value2)
        servo3.execution(value3)

        if value4 == "cw":
            run_motor_cw(value5)
        elif value4 == "ccw":
            run_motor_ccw(value5)
        else:
            print("###### error ######")