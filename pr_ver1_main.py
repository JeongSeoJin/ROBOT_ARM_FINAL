import pr_module
import time

def get_candy():
	pr_module.HOME()
	pr_module.motor_ccw(4)

	pr_module.STAND()
	pr_module.motor_ccw(4)

	pr_module.TARGET()
	pr_module.motor_cw(2)

	pr_module.DELIVERY()
	pr_module.motor_ccw(2)
	time.sleep(0.5)
	pr_module.COMPLETE()

	pr_module.motor_cw(8)
	pr_module.HOME()


if __name__ == "__main__":
	while True:
		control = raw_input("Do you want to delivery that semiconductor?? [ yes | no ] : ")
		if control == "yes":
			get_candy()

		else:
			pass