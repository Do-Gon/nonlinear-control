#!/usr/bin/env python3
# coding=utf-8

import os
from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from pymycobot.genre import Coord
import time

def main():
    mc = MyCobot('/dev/ttyUSB0', 1000000)
    speed = 50

    def reset_joints():
        mc.send_angles([0, 0, 0, 0, 0, -45], 50)
        mc.set_gripper_value(100, 50)

    reset_joints()
    time.sleep(3)

    # Grasping object
    coords = [276.80, -86.20, 120.30, -178.60, -3.45, -45.51]
    mc.send_coords(coords, speed)
    time.sleep(1)

    mc.set_gripper_value(20, 100)
    time.sleep(1)

    # intermediate point
    coords = [227.60, 48.60, 242.30, -173.16, -5.54, -24.07]
    mc.send_coords(coords, speed)
    time.sleep(1)

    # Releasing object
    coords = [210.10, 195.70, 120.40, -179.68, -0.17, -18.98]
    mc.send_coords(coords, speed) 
    time.sleep(1)

    mc.set_gripper_value(100, 100)
    time.sleep(1)

    reset_joints()
    time.sleep(3)

    mc.release_all_servos()

    if False:
        id = Angle.J1.value
        degree = 0
        mc.send_angle(id, degree, speed)

        degrees = [0.43, -0.52, 0.17, -1.14, -0.35, -45.08]
        mc.send_angles(degrees, speed)

    if False:
        id = Coord.Z.value
        coord = 350
        mc.send_coord(id, coord, speed)

        coords = [49.6, -63.3, 419, -92.11, -45.07, -88.41]
        mc.send_coords(coords, speed)


    

if __name__ == "__main__":
    main()
