#!/usr/bin/env python3
# coding=utf-8

from pymycobot.mycobot import MyCobot

def main():
    mc = MyCobot('/dev/ttyUSB0', 1000000)
    
    print("Releasing all servos...")
    mc.release_all_servos()

    for i in range(3):
        input(f"\nPress Enter to read joint angles ({i+1}/3)...")
        angles = mc.get_angles()
        if angles:
            print(f"Read #{i+1}: {['{:.2f}'.format(a) for a in angles]}")
        else:
            print(f"Read #{i+1}: Failed to read angles.")

if __name__ == "__main__":
    main()
