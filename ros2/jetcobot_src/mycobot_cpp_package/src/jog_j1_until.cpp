#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <iostream>
#include "mycobot/MyCobot.hpp"

using namespace mycobot;
using namespace std::chrono_literals;

#define NLCTL_VERBOSE 1 // default
// #define NLCTL_VERBOSE 2 

static double read_j1(const MyCobot &mc) {
    auto a = mc.GetAngles();
    return a[0];
}

static void pump_events_ms(int ms) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    using namespace std::chrono_literals;
    while (std::chrono::steady_clock::now() < end) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(10ms);
    }
}


int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);

    const double target_deg = 45.0;
    const int    jog_speed  = 10;
    const double tol_deg    = 1;

    try {
        auto mc = MyCobot::I();

        mc.PowerOn();
        pump_events_ms(1200); // wait for power-on

        double curr = read_j1(mc);

        if (std::abs(curr - target_deg) <= tol_deg) {
            std::cout << "Already within tolerance, no jog." << std::endl;
            return 0;
        }

        for (int i = 0; i < 20; ++i) {
            pump_events_ms(50);
            double v = read_j1(mc);
            if (std::abs(v - curr) > 1e-3) { curr = v; break; }
        }
        int dir = (target_deg > curr) ? +1 : -1;

        std::cout << "J1 current=" << curr << "°, target=" << target_deg
                  << "°, dir=" << (dir>0?"+":"-") << ", speed=" << jog_speed << std::endl;

        mc.JogAngle(J1, dir, jog_speed);

        for (int i = 0; i < 10000; ++i) {
            curr = read_j1(mc);
            bool reached = std::abs(curr - target_deg) <= tol_deg;

            #if NLCTL_VERBOSE > 1
            std::cout << "J1 current=" << curr << std::endl;
            #endif // NLCTL_VERBOSE

            #if NLCTL_VERBOSE > 1
            std::cout << "reached=" << std::boolalpha << reached << " angle=" << curr << std::endl;
            #endif // NLCTL_VERBOSE
            
            if (i == 399) {
                std::cout << "Jog timeout, stopping." << std::endl;
            }

            if (reached) break;
            pump_events_ms(20);
        }

        mc.StopRobot();                        // stop jog (same API layer)
        pump_events_ms(300);
        mc.PowerOff();
        pump_events_ms(300);

        std::cout << "Stopped at J1=" << curr << "°, torque off." << std::endl;
        return 0;

    } catch (const std::error_code &e) {
        std::cerr << "SDK error [" << e.value() << "]: " << e.message() << std::endl;
        return 1;
    }
}
