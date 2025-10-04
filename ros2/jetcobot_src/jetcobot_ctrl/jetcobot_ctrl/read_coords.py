#!/usr/bin/env python3
# coding=utf-8

from pymycobot.mycobot import MyCobot

def main():
    mc = MyCobot('/dev/ttyUSB0', 1000000)

    def reset_joints():
        mc.send_angles([0, 0, 0, 0, 0, -45], 50)
        mc.set_gripper_value(100, 50)

    reset_joints()

    print("Releasing all servos...")
    mc.release_all_servos()

    for i in range(3):
        input(f"\nPress Enter to read coordinates ({i+1}/3)...")
        coords = mc.get_coords()
        if coords:
            print(f"Read #{i+1}: {['{:.2f}'.format(c) for c in coords]}")
        else:
            print(f"Read #{i+1}: Failed to read coordinates.")

if __name__ == "__main__":
    main()
