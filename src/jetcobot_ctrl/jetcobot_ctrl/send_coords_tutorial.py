#!/usr/bin/env python3
# coding=utf-8

import os
# import ipywidgets.widgets as widgets
# from IPython.display import display
# import time
# import threading
# import cv2 as cv
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot.genre import Coord
import time

def main():
    mc = MyCobot('/dev/ttyUSB0', 1000000)
    g_speed = 100
    speed = 100

    def reset_joints():
        mc.send_angles([0, 0, 0, 0, 0, -45], 50)
        mc.set_gripper_value(100, 50)

    reset_joints()

    # mc.release_all_servos()

    if True:
        id = Angle.J1.value
        degree = 0
        mc.send_angle(id, degree, speed)
        time.sleep(1)
        coords = [-50.0, -70.0, 60, -92.11, -45.07, -88.41] # [0.43, -0.52, 0.17, -1.14, -0.35, -45.08]
        mc.send_coords(coords, speed)
        time.sleep(1)
    if False:
        id = Coord.Z.value
        coord = 350
        mc.send_coord(id, coord, speed)
        time.sleep(1)
        coords = [49.6, -63.3, 419, -92.11, -45.07, -88.41]
        mc.send_coords(coords, speed)
        time.sleep(1)
    if False:
        id = Angle.J1.value
        encoder = 1500
        mc.set_encoder(id, encoder, speed)
        time.sleep(1)
    if False:
        encoders = [2048, 2048, 2048, 2048, 2048, 2048]
        mc.set_encoders(encoders, speed)
        time.sleep(1)
    if False:
        radians = [0.008, -0.009, 0.003, -0.02, -0.006, -0.787]
        mc.send_radians(radians, speed)
        time.sleep(1)
    if False:
        gripper = 100
        mc.set_gripper_value(gripper, speed)
        time.sleep(1)

    

if __name__ == "__main__":
    main()
