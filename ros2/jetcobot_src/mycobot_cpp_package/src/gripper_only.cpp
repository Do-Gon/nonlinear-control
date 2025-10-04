#include <QCoreApplication>
#include <thread>
#include <chrono>
#include <iostream>
#include "mycobot/MyCobot.hpp"

using namespace mycobot;
using namespace std::chrono;

static void ms_sleep(int ms) { std::this_thread::sleep_for(milliseconds(ms)); }

// Send twice (docs: first call may be ignored on 280 adaptive gripper)
static void set_gripper_twice(MyCobot &mc, bool open, int gap_ms=300) {
    std::cout << "[gripper] " << (open ? "OPEN" : "CLOSE") << " #1\n";
    mc.SetGriper(open ? 1 : 0);  // 1=open, 0=close
    ms_sleep(gap_ms);
    std::cout << "[gripper] " << (open ? "OPEN" : "CLOSE") << " #2\n";
    mc.SetGriper(open ? 1 : 0);
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    try {
        auto mc = MyCobot::I();

        mc.PowerOn();
        mc.StopRobot();

        // === Wake end-effector IO (280 adaptive gripper): set M5 basic outs HIGH ===
        std::cout << "[wake] SetBasicOut(2,1)\n";  mc.SetBasicOut(2, 1);  ms_sleep(300);
        std::cout << "[wake] SetBasicOut(5,1)\n";  mc.SetBasicOut(5, 1);  ms_sleep(300);
        std::cout << "[wake] SetBasicOut(26,1)\n"; mc.SetBasicOut(26, 1); ms_sleep(300);

        // Some units also want Atom outputs HIGH
        std::cout << "[wake] SetDigitalOut(23,1)\n"; mc.SetDigitalOut(23, 1); ms_sleep(200);
        std::cout << "[wake] SetDigitalOut(33,1)\n"; mc.SetDigitalOut(33, 1); ms_sleep(200);

        // === Open → Close → Open (each sent twice) ===
        set_gripper_twice(mc, /*open=*/true, 300);
        ms_sleep(800);
        set_gripper_twice(mc, /*open=*/false, 300);
        ms_sleep(800);
        set_gripper_twice(mc, /*open=*/true, 300);
        ms_sleep(800);

        std::cout << "[done]\n";
        return 0;
    } catch (const std::error_code &e) {
        std::cerr << "[err] SDK error [" << e.value() << "]: " << e.message() << "\n";
        return 1;
    }
}
