#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <iostream>
#include "mycobot/MyCobot.hpp"  // wrapper
#include "MyCobot.hpp"           // low-level rc API

using namespace std::chrono_literals;

static void pump_events(int ms) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(10ms);
    }
}

static bool fetch_joint_limits(rc::Joint j, double &min_deg, double &max_deg, int timeout_ms=1200) {
    // Queue both requests once
    (void)rc::MyCobot::Instance().GetJointMinPosLimit(j);
    (void)rc::MyCobot::Instance().GetJointMaxPosLimit(j);

    // Wait until either value changes from its current cached value
    const auto t0 = std::chrono::steady_clock::now();
    double last_min = rc::MyCobot::Instance().GetJointMinPosLimit(j);
    double last_max = rc::MyCobot::Instance().GetJointMaxPosLimit(j);

    do {
        pump_events(50);
        min_deg = rc::MyCobot::Instance().GetJointMinPosLimit(j);
        max_deg = rc::MyCobot::Instance().GetJointMaxPosLimit(j);
        if (min_deg != last_min || max_deg != last_max) return true;
    } while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(timeout_ms));

    // Return whatever we have (may still be valid if cache was already filled)
    return (min_deg != 0.0 || max_deg != 0.0);
}

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);

    auto mc = mycobot::MyCobot::I();
    mc.PowerOn();
    std::this_thread::sleep_for(700ms);

    for (int idx = 1; idx <= 6; ++idx) {
        double mn=0, mx=0;
        bool ok = fetch_joint_limits(static_cast<rc::Joint>(idx), mn, mx, 1500);
        std::cout << "J" << (idx) << " limits: [" << mn << ", " << mx << "] deg"
                  << (ok ? "" : "  (timeout/no change)") << "\n";
    }

    mc.PowerOff();
    return 0;
}
