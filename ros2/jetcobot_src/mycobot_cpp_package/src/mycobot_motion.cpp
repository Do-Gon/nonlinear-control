#include <iostream>
#include <thread>
#include <chrono>
#include <QCoreApplication>
#include "mycobot/MyCobot.hpp"

using namespace mycobot;
using namespace std::chrono_literals;

static void wait_until_reached(const MyCobot &mc, const Coords &target) {
    for (int i = 0; i < 200; ++i) {
        if (mc.IsInPosition(target, true) || !mc.IsMoving()) break;
        std::this_thread::sleep_for(100ms);
    }
}

static void set_gripper(MyCobot &mc, bool open, int pause_ms = 300) {
    mc.SetGriper(open ? 1 : 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(pause_ms));
    mc.SetGriper(open ? 1 : 0);
}

int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);
    try {
        auto mc = MyCobot::I();
        int speed = 50;

        mc.PowerOn();
        mc.StopRobot();

        mc.SetBasicOut(2, 1);
        std::this_thread::sleep_for(1s);
        mc.SetBasicOut(5, 1);
        std::this_thread::sleep_for(1s);
        mc.SetBasicOut(26, 1);
        std::this_thread::sleep_for(1s);

        Angles home{0, 0, 0, 0, 0, -45};
        mc.WriteAngles(home, speed);
        set_gripper(mc, true);
        std::this_thread::sleep_for(1s);

        Coords grasp{276.80, -86.20, 120.30, -178.60, -3.45, -45.51};
        mc.WriteCoords(grasp, speed);
        wait_until_reached(mc, grasp);

        set_gripper(mc, false);
        std::this_thread::sleep_for(1s);

        Coords mid{227.60, 48.60, 242.30, -173.16, -5.54, -24.07};
        mc.WriteCoords(mid, speed);
        wait_until_reached(mc, mid);
        std::this_thread::sleep_for(1s);

        Coords place{210.10, 195.70, 120.40, -179.68, -0.17, -18.98};
        mc.WriteCoords(place, speed);
        wait_until_reached(mc, place);
        set_gripper(mc, true);
        std::this_thread::sleep_for(1s);

        mc.WriteAngles(home, speed);
        std::this_thread::sleep_for(3s);

        mc.PowerOff();                 // releases all servos
        std::this_thread::sleep_for(500ms);

        return EXIT_SUCCESS;
    } catch (const std::error_code &e) {
        std::cerr << "SDK error [" << e.value() << "]: " << e.message() << "\n";
        return EXIT_FAILURE;
    }
}
