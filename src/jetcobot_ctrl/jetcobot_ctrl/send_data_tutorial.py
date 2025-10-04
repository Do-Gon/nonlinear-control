#!/usr/bin/env python3
# coding=utf-8

import os
import time
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot.genre import Coord

def main():
    mc = MyCobot('/dev/ttyUSB0', 1000000)
    speed = 100

    if True:
        id = Angle.J1.value
        degree = 0
        mc.send_angle(id, degree, speed)

        degrees = [0.43, -0.52, 0.17, -1.14, -0.35, -45.08]
        mc.send_angles(degrees, speed)

    # if True:
    #     id = Coord.Z.value
    #     coord = 350
    #     mc.send_coord(id, coord, speed)

    #     coords = [49.6, -63.3, 419, -92.11, -45.07, -88.41]
    #     mc.send_coords(coords, speed)

    # if True:
    #     id = Angle.J1.value
    #     encoder = 1500
    #     mc.set_encoder(id, encoder, speed)

    # if True:
    #     encoders = [2048, 2048, 2048, 2048, 2048, 2048]
    #     mc.set_encoders(encoders, speed)

    # if True:
    #     radians = [0.008, -0.009, 0.003, -0.02, -0.006, -0.787]
    #     mc.send_radians(radians, speed)

    # if True:
    #     gripper = 100
    #     mc.set_gripper_value(gripper, speed)

    

if __name__ == "__main__":
    main()
