from time import sleep
import RPi.GPIO as GPIO    
import time
import Adafruit_PCA9685
pwm = Adafruit_PCA9685.PCA9685()

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
		value1 = int(input("servo1(middle) : "))
		value2 = int(input("servo2(wrist) : "))
		value3 = int(input("servo3(grap) : "))

		servo1.execution(value1)
		servo2.execution(value2)
		servo3.execution(value3)