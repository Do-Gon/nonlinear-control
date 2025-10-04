from pymycobot.mycobot import MyCobot
import time

mc = MyCobot('/dev/ttyUSB0', 1000000)
time.sleep(0.6)
mc.set_gripper_value(100, 50); time.sleep(0.8)  # open
mc.set_gripper_value( 20, 50); time.sleep(0.8)  # close
