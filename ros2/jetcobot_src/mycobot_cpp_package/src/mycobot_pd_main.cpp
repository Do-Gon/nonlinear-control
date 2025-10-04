// ---- simplfied ----
// #include <QCoreApplication>
// #include <chrono>
// #include <thread>
// #include <iostream>
// #include "mycobot/MyCobot.hpp"
// #include "mycobot_cpp_package/pd_controller.hpp"

// using namespace std::chrono_literals;

// int main(int argc, char** argv) {
//     QCoreApplication app(argc, argv);

//     // Targets & gains
//     constexpr double TARGET_DEG = -70.0;
//     constexpr double TARGET_VEL = 0.0;

//     PDController pd{0.2, 0.1, -1.0, 1.0, 0.1};

//     PDRunnerConfig cfg;
//     cfg.pos_tol_deg = 1.0;
//     cfg.vel_tol_dps = 5.0;
//     cfg.vel_alpha   = 0.25;
//     cfg.vel_clip    = 60.0;
//     cfg.brake_enable_err = 20.0;
//     cfg.brake_vel_min    = 2.0;
//     cfg.reverse_hysteresis = 75ms;
//     cfg.min_cmd_period     = 15ms;
//     cfg.min_spd = 1;
//     cfg.max_spd = 30;
//     cfg.cap_e_lo = 2.0;
//     cfg.cap_e_hi = 15.0;
//     cfg.verbose  = 2;   // 0,1,2

//     try {
//         auto mc = mycobot::MyCobot::I();

//         mc.PowerOn();
//         std::this_thread::sleep_for(1200ms);
//         mc.StopRobot();
//         std::this_thread::sleep_for(200ms);

//         // (Optional) move to a home pose first
//         int speed = 50;
//         mycobot::Angles home{0, 0, 0, 0, 0, -45};
//         mc.WriteAngles(home, speed);
//         std::this_thread::sleep_for(3s);

//         // Run the closed-loop PD jog on J1
//         J1PDControllerRunner runner(pd, cfg);
//         runner.run_to_target(mc, TARGET_DEG, TARGET_VEL);

//         // Done
//         std::cout << "YAY! PD control complete.\n";
//         return 0;
//     }
//     catch (const std::error_code &e) {
//         std::cerr << "SDK error [" << e.value() << "]: " << e.message() << "\n";
//         return 1;
//     }
// }

// ----- original ------
// #include <QCoreApplication>
// #include <chrono>
// #include <thread>
// #include <iostream>
// #include <cmath>
// #include <algorithm>  // std::clamp
// #include "mycobot/MyCobot.hpp"
// #include "mycobot_cpp_package/pd_controller.hpp"

// using namespace mycobot;
// using namespace std::chrono_literals;

// //#define NLCTL_VERBOSE 1
// #define NLCTL_VERBOSE 2

// // helper functions --------------------------------------------------

// static double read_j1(const MyCobot &mc) {
//     auto a = mc.GetAngles();
//     return a[0]; // a[0]: J1, a[1]: J2, a[2]: J3, a[3]: J4, a[4]: J5, a[5]: J6 (deg)
// }

// // Pump Qt events to keep QSerialPort callbacks flowing
// static void pump_events_ms(int ms) {
//     auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
//     while (std::chrono::steady_clock::now() < end) {
//         QCoreApplication::processEvents();
//         std::this_thread::sleep_for(10ms);
//     }
// }

// // Map normalized |u| in [0,1] to speed percent [min_spd, max_spd]
// static int speed_from_u(double abs_u, int min_spd=1, int max_spd=30) {
//     abs_u = std::clamp(abs_u, 0.0, 1.0);
//     double s = min_spd + abs_u * (max_spd - min_spd);
//     return std::clamp<int>(static_cast<int>(std::round(s)), min_spd, max_spd);
// }

// // Smoothly choose a per-iteration speed cap (1..30) from absolute error.
// static int cap_from_error(double ae_deg, double e_lo=2.0, double e_hi=40.0) {
//     if (ae_deg <= 0.5) return 0; // tiny window -> explicit stop
//     double t = (ae_deg - e_lo) / (e_hi - e_lo);
//     t = std::clamp(t, 0.0, 1.0);
//     t = t*t*(3.0 - 2.0*t); // cubic smoothstep to avoid sharp transitions
//     double cap = 1.0 + t * (30.0 - 1.0);         
//     return static_cast<int>(std::round(std::clamp(cap, 1.0, 30.0)));
// }

// static int sgn(double x) { 
//     return (x > 0.0) - (x < 0.0); // Positive direction: True - False = 1, Negative direction: False - True = -1, Zero: 0
// }

// // main ------------------------------------------------------------

// int main(int argc, char** argv) {
//     QCoreApplication app(argc, argv);

//     constexpr double TARGET_DEG = 50.0; // 80, -50, 150
//     constexpr double TRAGET_VEL = 0.0;
//     constexpr double POS_TOL_DEG  = 1.0;
//     constexpr double VEL_TOL_DPS  = 5.0;

//     // PD gains
//     PDController pd{0.2, 0.1, -1.0, 1.0, 0.1};

//     // Derivative protection
//     const double vel_alpha = 0.25;   // EMA (0..1) higher = less smoothing
//     const double vel_clip  = 60.0;   // clamp D-term velocity (deg/s)

//     // Reverse/brake policy near target
//     const auto   reverse_hysteresis = 75ms;   // min time between direction flips
//     const double brake_enable_err   = 20.0;   // |e| under this → allow reverse
//     const double brake_vel_min      = 2.0;    // need some motion to justify braking

//     try {
//         auto mc = MyCobot::I();
//         mc.PowerOn();
//         pump_events_ms(1200);
//         mc.StopRobot();
//         pump_events_ms(200);

//         double curr = read_j1(mc);

//         // ---- loop state ----
//         int last_dir = 0;
//         int last_spd = 0;

//         double vel_ema = 0.0;

//         auto t_prev = std::chrono::steady_clock::now();
//         double prev_pos = curr;

//         auto last_dir_change = t_prev;
//         auto last_cmd_time   = t_prev;

//         int speed = 50;
//         Angles home{0, 0, 0, 0, 0, -45};
//         mc.WriteAngles(home, speed);
//         std::this_thread::sleep_for(3s);

//         // ---- control loop ----
//         while (true) {
//             auto t_now = std::chrono::steady_clock::now();
//             double dt_s = std::chrono::duration<double>(t_now - t_prev).count();
//             if (dt_s <= 0.0) dt_s = 1e-3; // guard
//             t_prev = t_now;

//             curr = read_j1(mc);

//             // Raw velocity (deg/s)
//             double vel_inst = (curr - prev_pos) / dt_s;
//             prev_pos = curr;

//             // Low-pass + clamp for D-term robustness: Previously there were unexpected spikes when computing vel_inst.
//             vel_ema = vel_alpha * vel_inst + (1.0 - vel_alpha) * vel_ema;
//             double vel_filtered = std::clamp(vel_ema, -vel_clip, +vel_clip);

//             // Position & Velocity error
//             double e  = TARGET_DEG - curr;
//             double ae = std::abs(e);                  
//             double v_e = TRAGET_VEL - vel_filtered;
//             double av_e = std::abs(v_e);

//             // stopping condition
//             bool pos_ok  = (ae <= POS_TOL_DEG);
//             bool vel_ok = (av_e <= VEL_TOL_DPS);
//             if (pos_ok && vel_ok) {
//                     mc.StopRobot();
//                     pump_events_ms(500);
//                     break;
//             } 

//             double u_raw = pd.Kp * e + pd.Kd * v_e;
//             double u = std::clamp(u_raw, -1.0, 1.0);
//             if (std::abs(u) < pd.deadband) u = 0.0;

//             // Decide desired direction from controller
//             int desired_dir = sgn(u);
//             if (desired_dir == 0) {
//                 // no demand → stop
//             } 
//             else if (desired_dir != sgn(e)) {
//                 // controller asks to reverse
//                 if (!(ae <= brake_enable_err && std::abs(vel_filtered) >= brake_vel_min)) {
//                     desired_dir = sgn(e);
//                 }
//             }

//             // to avoid direction flipping (to prevent spamming of reverse commands)
//             int dir = desired_dir;
//             if (dir != 0 && dir != last_dir && last_dir != 0) {
//                 if (t_now - last_dir_change < reverse_hysteresis) {
//                     dir = last_dir; // still within hysteresis window, hold direction
//                 }
//             }

//             // Continuous slowdown (cap 1..30) based on |error|
//             int spd_cap = cap_from_error(ae); // 0, or 1..30
//             int spd = (dir == 0 || spd_cap == 0) ? 0: speed_from_u(std::abs(u), 1, spd_cap);

//             // reverse logic
//             bool need_stop   = (dir == 0 || spd == 0);
//             bool dir_changed = (dir != last_dir && last_dir != 0);
//             bool spd_changed = (std::abs(spd - last_spd) >= 1);
//             bool time_ok     = (t_now - last_cmd_time) >= 15ms;

//             if (need_stop) {
//                 if (last_dir != 0 && time_ok) {
//                     mc.StopRobot();
//                     pump_events_ms(100);
//                     last_cmd_time = t_now;
//                     last_dir = 0;
//                     last_spd = 0;
//                 }
//             } 
//             else {
//                 if (dir_changed && time_ok) {
//                     // we’re about to reverse and we’re not spamming commands.
//                     mc.StopRobot();
//                     pump_events_ms(100);
//                     last_dir_change = t_now;
//                 }
//                 if ((dir_changed || spd_changed) && time_ok) {
//                     mc.JogAngle(J1, dir, spd);
//                     pump_events_ms(100);
//                     last_cmd_time = t_now;
//                     last_dir = dir;
//                     last_spd = spd;
//                 }
//             }

//         #if NLCTL_VERBOSE > 1
//             std::cout << "[J1] pos=" << curr
//                       << " deg, vel_inst=" << vel_inst
//                       << " dps, vel_filtered=" << vel_filtered
//                       << " dps, e=" << e
//                       << ", v_e=" << v_e
//                       << ", u_raw=" << u_raw
//                       << " -> dir=" << (dir>0?"+":(dir<0?"-":"0"))
//                       << ", cap=" << spd_cap
//                       << ", spd=" << spd << "\n";
//         #endif

//             // Service events to ~50 Hz
//             int sleep_ms = 20 - static_cast<int>(std::round(dt_s * 1000.0));
//             if (sleep_ms > 0) pump_events_ms(sleep_ms);
//         }

//         // Done
//         // mc.PowerOff(); // or mc.PowerOff();
//         pump_events_ms(300); // this should be greater than 100!!!!

//         std::cout << "YAY! PD control complete. Final J1 ~ " << curr << " deg.\n";
//         return 0;
//     }
//     // just in case...
//     catch (const std::error_code &e) {
//         std::cerr << "SDK error [" << e.value() << "]: " << e.message() << "\n";
//         return 1;
//     }
// }


// ------ new method for first joint control --------
// #include <QCoreApplication>
// #include <chrono>
// #include <thread>
// #include <iostream>
// #include <cmath>
// #include <algorithm>  // std::clamp
// #include "mycobot/MyCobot.hpp"
// #include "mycobot_cpp_package/pd_controller.hpp"

// using namespace mycobot;
// using namespace std::chrono_literals;

// //#define NLCTL_VERBOSE 1
// #define NLCTL_VERBOSE 2

// // --- helpers ---------------------------------------------------------------

// static double read_j1(const MyCobot &mc) {
//     auto a = mc.GetAngles();
//     return a[0]; // a[0]: J1, a[1]: J2, a[2]: J3, a[3]: J4, a[4]: J5, a[5]: J6 (deg)
// }

// // Pump Qt events to keep QSerialPort callbacks flowing
// static void pump_events_ms(int ms) {
//     auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
//     while (std::chrono::steady_clock::now() < end) {
//         QCoreApplication::processEvents();
//         std::this_thread::sleep_for(10ms);
//     }
// }

// // Map normalized |u| in [0,1] to speed percent [min_spd, max_spd]
// static int speed_from_u(double abs_u, int min_spd=1, int max_spd=30) {
//     abs_u = std::clamp(abs_u, 0.0, 1.0);
//     double s = min_spd + abs_u * (max_spd - min_spd);
//     return std::clamp<int>(static_cast<int>(std::round(s)), min_spd, max_spd);
// }

// // Smoothly choose a per-iteration speed cap (1..30) from absolute error.
// static int cap_from_error(double ae_deg, double e_lo=2.0, double e_hi=40.0) {
//     if (ae_deg <= 0.5) return 0; // tiny window -> explicit stop
//     double t = (ae_deg - e_lo) / (e_hi - e_lo);
//     t = std::clamp(t, 0.0, 1.0);
//     t = t*t*(3.0 - 2.0*t); // cubic smoothstep
//     double cap = 1.0 + t * (30.0 - 1.0);
//     return static_cast<int>(std::round(std::clamp(cap, 1.0, 30.0)));
// }

// static int sgn(double x) {
//     return (x > 0.0) - (x < 0.0); // +1, -1, or 0
// }

// // --- main ------------------------------------------------------------------

// int main(int argc, char** argv) {
//     QCoreApplication app(argc, argv);

//     // Targets & tolerances
//     constexpr double TARGET_DEG   = -50.0;
//     constexpr double TARGET_VEL   = 0.0;
//     constexpr double POS_TOL_DEG  = 1.0;
//     constexpr double VEL_TOL_DPS  = 5.0;

//     // PD gains
//     PDController pd{0.14, 0.1, -1.0, 1.0, 0.35};

//     // Derivative protection
//     const double vel_alpha = 0.25;   // EMA (0..1); higher = less smoothing
//     const double vel_clip  = 30.0;   // clamp measured velocity (deg/s)

//     // Reverse/brake policy near target
//     const auto   reverse_hysteresis = 20ms;   // min time between direction flips
//     const double brake_enable_err   = 10.0;   // |e| under this → allow reverse
//     const double brake_vel_min      = 2.0;    // need some motion to justify braking

//     // Runaway detection (Option B)
//     // double       growth_accum_s       = 0.0;   // accumulated "growth time" (s)
//     // const double runaway_enable_err   = 30.0;  // deg
//     // const auto   runaway_min_time     = 75ms;  // must persist this long
//     // const int    runaway_brake_cap    = 8;     // extra-tight cap during brake
//     // const auto   runaway_cmd_cooldown = 25ms;  // min time between forced commands

//     try {
//         auto mc = MyCobot::I();
//         mc.PowerOn();
//         pump_events_ms(1200);
//         mc.StopRobot();
//         pump_events_ms(1200);

//         int last_dir = 0;
//         int last_spd = 0;

//         double vel_ema = 0.0;

//         int speed = 80;
//         Angles home{0, 0, 0, 0, 0, -45};
//         mc.WriteAngles(home, speed);
//         std::this_thread::sleep_for(3s);
        
//         double curr = read_j1(mc);
//         auto   t_prev      = std::chrono::steady_clock::now();
//         double prev_pos    = curr;

//         auto last_dir_change = t_prev;
//         auto last_cmd_time   = t_prev;

//         // runaway state
//         // double prev_ae = std::abs(TARGET_DEG - curr);
//         // bool   runaway_active  = false;

//         // ---- control loop ----
//         while (true) {
//             auto t_now = std::chrono::steady_clock::now();
//             double dt_s = std::chrono::duration<double>(t_now - t_prev).count();
//             if (dt_s <= 0.0) dt_s = 1e-3; // guard
//             t_prev = t_now;

//             curr = read_j1(mc);

//             // Velocity (deg/s)
//             double vel_inst = (curr - prev_pos) / dt_s;
//             prev_pos = curr;

//             // Low-pass + clamp for robustness
//             vel_ema = vel_alpha * vel_inst + (1.0 - vel_alpha) * vel_ema;
//             double vel_filtered = std::clamp(vel_ema, -vel_clip, +vel_clip);

//             // Errors
//             double e    = TARGET_DEG - curr;
//             double ae   = std::abs(e);
//             double v_e  = TARGET_VEL - vel_filtered;
//             double av_e = std::abs(v_e);

//             // Stop condition
//             bool pos_ok = (ae <= POS_TOL_DEG);
//             bool vel_ok = (av_e <= VEL_TOL_DPS);
//             if (pos_ok && vel_ok) {
//                 mc.StopRobot();
//                 pump_events_ms(500);
//                 break;
//             }

//             // --- Runaway detection: large & persistently growing |e| --------
//             // const double growth_eps_deg = 0.05; // ignore tiny noise
//             // const double runaway_min_time_s =
//             //     std::chrono::duration<double>(runaway_min_time).count();

//             // bool ae_growing = (ae > prev_ae + growth_eps_deg);
//             // bool ae_large   = (ae >= runaway_enable_err);

//             // if (ae_large && ae_growing) {
//             //     growth_accum_s += dt_s;                // accumulate while growing
//             // } 
//             // else {
//             //     // gentle decay so the flag doesn't oscillate too fast
//             //     growth_accum_s = std::max(0.0, growth_accum_s - 0.5 * dt_s);
//             // }
//             // runaway_active = (growth_accum_s >= runaway_min_time_s);
//             // prev_ae = ae;

//             // PD compute (normalized u in [-1,1])
//             double u_raw = pd.Kp * e + pd.Kd * v_e;
//             double u = std::clamp(u_raw, -1.0, 1.0);
//             if (std::abs(u) < pd.deadband) u = 0.0;

//             // Desired direction from PD (may be overridden)
//             int desired_dir = sgn(u);

//             bool allow_brake_near_target =
//                 (ae <= brake_enable_err && std::abs(vel_filtered) >= brake_vel_min);

//             if (desired_dir != 0 && desired_dir != sgn(e)) {
//                 // PD wants to reverse; permit if allowed near target
//                 if (!allow_brake_near_target) {
//                     desired_dir = sgn(e);
//                 }
//             }

//             // Hysteresis (disabled if we enter forced-runaway block below)
//             int dir = desired_dir;
//             if (dir != 0 && dir != last_dir && last_dir != 0) {
//                 if (t_now - last_dir_change < reverse_hysteresis) {
//                     dir = last_dir; // hold direction during hysteresis
//                 }
//             }

//             // Continuous slowdown (cap 1..30) based on |error|
//             int spd_cap = cap_from_error(ae); // 0, or 1..30
//             int spd = (dir == 0 || spd_cap == 0) ? 0
//                                                  : speed_from_u(std::abs(u), 1, spd_cap);

//             // ================== FORCED REVERSE DURING RUNAWAY ==================
//             // if (runaway_active) {
//             //     // Reverse relative to what we're currently doing if moving,
//             //     // otherwise reverse the "toward-target" guess.
//             //     int brake_dir = (last_dir != 0) ? -last_dir : -sgn(e);
//             //     if (brake_dir == 0) brake_dir = (e >= 0 ? -1 : 1); // fallback

//             //     // Tight cap while braking runaway
//             //     int brake_cap = runaway_brake_cap;
//             //     // If you prefer to also respect distance-based cap:
//             //     // brake_cap = std::min(runaway_brake_cap, cap_from_error(ae));

//             //     int brake_spd = speed_from_u(1.0, 1, brake_cap); // push firmly within cap

//             // #if NLCTL_VERBOSE > 1
//             //     // std::cout << " ae=" << ae
//             //     //           << " brake_dir=" << (brake_dir>0?"+":"-")
//             //     //           << " brake_spd=" << brake_spd
//             //     //           << " growth_s=" << growth_accum_s
//             //     //           << "[RUNAWAY BRAKE] e=" << e
//             //     //           << "\n";
//             // #endif
//             //     // Skip normal command block this cycle.
//             //     // (Prevents it from immediately undoing the forced reverse.)
//             //     int sleep_ms = 20 - static_cast<int>(std::round(dt_s * 1000.0));
//             //     if (sleep_ms > 0) pump_events_ms(sleep_ms);
//             //     continue;
//             // }
//             // ===================================================================

//             // Command hygiene: stop before reversing; rate-limit command spam
//             bool need_stop   = (dir == 0 || spd == 0);
//             bool dir_changed = (dir != last_dir && last_dir != 0);
//             bool spd_changed = (std::abs(spd - last_spd) >= 1);
//             bool time_ok     = (t_now - last_cmd_time) >= 15ms;

//             if (need_stop) {
//                 if (last_dir != 0 && time_ok) {
//                     mc.StopRobot();
//                     pump_events_ms(100);
//                     last_cmd_time = t_now;
//                     last_dir = 0;
//                     last_spd = 0;
//                 }
//             } else {
//                 if (dir_changed && time_ok) { // if ((dir_changed && time_ok) || runaway_active) {
//                     // we’re about to reverse and we’re not spamming commands
//                     mc.StopRobot();
//                     pump_events_ms(100);
//                     last_dir_change = t_now;
//                 }
//                 if ((dir_changed || spd_changed) && time_ok) { // if (((dir_changed || spd_changed) && time_ok) || runaway_active) {
//                     mc.JogAngle(J1, dir, spd);
//                     last_cmd_time = t_now;
//                     last_dir = dir;
//                     last_spd = spd;
//                 }
//             }

//         #if NLCTL_VERBOSE > 1
//             std::cout << "[J1] pos=" << curr
//                       << " deg, vel_inst=" << vel_inst
//                       << " dps, vel_filt=" << vel_filtered
//                       << " dps, e=" << e
//                       << ", v_e=" << v_e
//                       << ", u_raw=" << u_raw
//                       << " -> dir=" << (dir>0?"+":(dir<0?"-":"0"))
//                       << ", cap=" << spd_cap
//                       << ", spd=" << spd
//                     //   << ", runaway=" << (runaway_active ? "Y" : "N")
//                     //   << ", growth_s=" << growth_accum_s
//                       << "\n";
//         #endif

//             // Service events to ~50 Hz
//             int sleep_ms = 20 - static_cast<int>(std::round(dt_s * 1000.0));
//             if (sleep_ms > 0) pump_events_ms(sleep_ms);
//         }
//         // Done
//         mc.WriteAngles(home, speed);
//         std::this_thread::sleep_for(3s);
//         mc.StopRobot(); // or mc.PowerOff();
//         pump_events_ms(300);
//         std::cout << "YAY! PD control complete. Final J1 ~ " << curr << " deg.\n";
//         return 0;
//     }
//     catch (const std::error_code &e) {
//         std::cerr << "SDK error [" << e.value() << "]: " << e.message() << "\n";
//         return 1;
//     }
// }

// ------ new method for multi joint control --------
#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <array>
#include "mycobot/MyCobot.hpp"
#include "mycobot_cpp_package/pd_controller.hpp"

using namespace mycobot;
using namespace std::chrono_literals;

//#define NLCTL_VERBOSE 1
#define NLCTL_VERBOSE 2

// --- helpers ---------------------------------------------------------------
static Angles read_all(const MyCobot &mc) {
    return mc.GetAngles(); // deg
}

static void pump_events_ms(int ms) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
        QCoreApplication::processEvents();
        std::this_thread::sleep_for(10ms);
    }
}

static int speed_from_u(double abs_u, int min_spd=1, int max_spd=30) {
    abs_u = std::clamp(abs_u, 0.0, 1.0);
    double s = min_spd + abs_u * (max_spd - min_spd);
    return std::clamp<int>(static_cast<int>(std::round(s)), min_spd, max_spd);
}

static int cap_from_error(double ae_deg, double e_lo=2.0, double e_hi=40.0) {
    if (ae_deg <= 0.5) return 0; // explicit stop near zero error
    double t = (ae_deg - e_lo) / (e_hi - e_lo);
    t = std::clamp(t, 0.0, 1.0);
    t = t*t*(3.0 - 2.0*t); // smoothstep
    double cap = 1.0 + t * (30.0 - 1.0);
    return static_cast<int>(std::round(std::clamp(cap, 1.0, 30.0)));
}

static inline int sgn(double x) { return (x > 0.0) - (x < 0.0); }

// --- main ------------------------------------------------------------------
int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);

    // Joints helper
    constexpr Joint J[6] = {J1, J2, J3, J4, J5, J6};

    // Targets & tolerances (deg, deg/s)
    Angles target_deg   = {75, 74.35, -126, 53, 4, -47};  // << set per joint {40, -81, 133, -54, 5, -49}, {75, 74.35, -126, 53, 4, -47}, {127, -92, 120, -25, 28, -49}
    Angles target_vel   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    constexpr double POS_TOL_DEG = 2.0;
    constexpr double VEL_TOL_DPS = 1.0;

    // PD gains (shared; you can make this an array for per-joint tuning)
    PDController pd{0.14, 0.1, -1.0, 1.0, 0.3};

    // Derivative protection
    const double vel_alpha = 0.25; // EMA 0..1
    const double vel_clip  = 30.0; // deg/s

    // Reverse/brake policy near target
    const auto   reverse_hysteresis = 20ms;
    const double brake_enable_err   = 10.0; // |e| under this → allow reverse
    const double brake_vel_min      = 2.0;

    try {
        auto mc = MyCobot::I();
        mc.PowerOn();
        pump_events_ms(1200);
        mc.StopRobot();
        pump_events_ms(800);

        // Move to “home” first (optional)
        int move_speed = 80;
        Angles home{0, 0, 0, 0, 0, -45};
        mc.WriteAngles(home, move_speed);
        std::this_thread::sleep_for(3s);

        // --- per-joint state ---
        Angles prev_pos   = read_all(mc);
        std::array<double, 6> vel_ema{};           // 0 init
        std::array<int,    6> last_dir{};          // 0 = not moving
        std::array<int,    6> last_spd{};          // last speed command
        std::array<std::chrono::steady_clock::time_point, 6> last_dir_change;
        auto now0 = std::chrono::steady_clock::now();
        for (int i=0;i<6;++i) last_dir_change[i] = now0;

        auto t_prev        = now0;
        auto last_cmd_time = now0;

        // ---- control loop ----
        while (true) {
            auto t_now = std::chrono::steady_clock::now();
            double dt_s = std::chrono::duration<double>(t_now - t_prev).count();
            if (dt_s <= 0.0) dt_s = 1e-3; // guard
            t_prev = t_now;

            // Read all joints
            Angles curr = read_all(mc);

            // Per-joint measures & decisions
            bool all_pos_ok = true, all_vel_ok = true;
            bool any_dir_changed_needs_stop = false;
            bool any_joint_needs_stop = false;

            int   desired_dir[6] = {0};
            int   final_dir[6]   = {0};
            int   final_spd[6]   = {0};
            int   spd_cap[6]     = {0};

            for (int i=0;i<6;++i) {
                // Velocity estimate
                double vel_inst = (curr[i] - prev_pos[i]) / dt_s;
                prev_pos[i] = curr[i];
                vel_ema[i] = vel_alpha * vel_inst + (1.0 - vel_alpha) * vel_ema[i];
                double vel_filtered = std::clamp(vel_ema[i], -vel_clip, +vel_clip);

                // Errors
                double e   = target_deg[i] - curr[i];
                double ae  = std::abs(e);
                double v_e = target_vel[i] - vel_filtered;
                double av_e= std::abs(v_e);

                bool pos_ok = (ae <= POS_TOL_DEG);
                bool vel_ok = (av_e <= VEL_TOL_DPS);
                all_pos_ok &= pos_ok;
                all_vel_ok &= vel_ok;

                // PD control
                double u_raw = pd.Kp * e + pd.Kd * v_e;
                double u = std::clamp(u_raw, -1.0, 1.0);
                if (std::abs(u) < pd.deadband) u = 0.0;

                int ddir = sgn(u); // desired direction from PD

                // Brake policy near target
                bool allow_brake_near_target =
                    (ae <= brake_enable_err && std::abs(vel_filtered) >= brake_vel_min);

                if (ddir != 0 && ddir != sgn(e)) {
                    if (!allow_brake_near_target) {
                        ddir = sgn(e); // keep moving toward target, don't reverse yet
                    }
                }
                desired_dir[i] = ddir;

                // Hysteresis (may be overridden by global stop)
                int dir = ddir;
                if (dir != 0 && dir != last_dir[i] && last_dir[i] != 0) {
                    if (t_now - last_dir_change[i] < reverse_hysteresis) {
                        dir = last_dir[i]; // hold during hysteresis window
                    }
                }

                // Speed selection
                spd_cap[i] = cap_from_error(std::abs(e)); // 0 or 1..30
                int spd = (dir == 0 || spd_cap[i] == 0) ? 0
                         : speed_from_u(std::abs(u), 1, spd_cap[i]);

                final_dir[i] = dir;
                final_spd[i] = spd;

                // Determine if this joint implies a global stop this frame
                bool need_stop_i   = (dir == 0 || spd == 0);
                bool dir_changed_i = (dir != last_dir[i] && last_dir[i] != 0);

                any_joint_needs_stop |= need_stop_i && (last_dir[i] != 0);
                any_dir_changed_needs_stop |= dir_changed_i;
            }

            // Exit when ALL joints are within tolerances
            if (all_pos_ok && all_vel_ok) {
                mc.StopRobot();
                pump_events_ms(400);
                break;
            }

            // Frame rate-limit
            bool time_ok = (t_now - last_cmd_time) >= 15ms;
            if (time_ok) {
                bool do_global_stop = (any_joint_needs_stop || any_dir_changed_needs_stop);

                if (do_global_stop) {
                    mc.StopRobot();
                    pump_events_ms(100);
                    last_cmd_time = t_now;
                    // Reset last_dir when we actually came to a stop
                    for (int i=0;i<6;++i) {
                        if (last_dir[i] != 0) {
                            last_dir[i] = 0;
                            last_spd[i] = 0;
                        }
                    }
                }

                // Issue jogs for all joints that should move now
                bool any_cmd_sent = false;
                for (int i=0;i<6;++i) {
                    if (final_dir[i] != 0 && final_spd[i] > 0) {
                        // Send only if changed OR after a global stop
                        bool need_cmd = (do_global_stop
                                         || final_dir[i] != last_dir[i]
                                         || std::abs(final_spd[i] - last_spd[i]) >= 1);
                        if (need_cmd) {
                            mc.JogAngle(J[i], final_dir[i], final_spd[i]);
                            last_dir[i] = final_dir[i];
                            last_spd[i] = final_spd[i];
                            if (final_dir[i] != 0) last_dir_change[i] = t_now;
                            any_cmd_sent = true;
                        }
                    } else {
                        // Not moving; we already issued a global stop if needed
                        last_dir[i] = 0;
                        last_spd[i] = 0;
                    }
                }
                if (any_cmd_sent) last_cmd_time = t_now;
            }

        #if NLCTL_VERBOSE > 1
            // Debug printout
            std::cout << "[Frame]\n";
            for (int i=0;i<6;++i) {
                std::cout << "  J" << (i+1)
                          << " pos=" << prev_pos[i]
                          << " dir=" << (final_dir[i]>0?"+":(final_dir[i]<0?"-":"0"))
                          << " spd=" << final_spd[i]
                          << " cap=" << spd_cap[i]
                          << "\n";
            }
        #endif

            // ~50 Hz service
            int sleep_ms = 20 - static_cast<int>(std::round(dt_s * 1000.0));
            if (sleep_ms > 0) pump_events_ms(sleep_ms);
        }

        // End: go back to home
        mc.WriteAngles(home, move_speed);
        std::this_thread::sleep_for(3s);
        mc.StopRobot();
        pump_events_ms(300);

        Angles final_curr = read_all(mc);
        std::cout << "YAY! PD control complete. Final [J1..J6] ~ ["
                << final_curr[0] << ", " << final_curr[1] << ", " << final_curr[2] << ", "
                << final_curr[3] << ", " << final_curr[4] << ", " << final_curr[5]
                << "] deg.\n";

                return 0;
            }
    catch (const std::error_code &e) {
        std::cerr << "SDK error [" << e.value() << "]: " << e.message() << "\n";
        return 1;
    }
}

// threading method
// #include <QCoreApplication>
// #include <chrono>
// #include <thread>
// #include <iostream>
// #include <cmath>
// #include <algorithm>
// #include <array>
// #include <future>
// #include <atomic>

// #include "mycobot/MyCobot.hpp"
// #include "mycobot_cpp_package/pd_controller.hpp"

// using namespace mycobot;
// using namespace std::chrono_literals;

// //#define NLCTL_VERBOSE 1
// #define NLCTL_VERBOSE 2

// // --- helpers ---------------------------------------------------------------
// static Angles read_all(const MyCobot &mc) {
//     return mc.GetAngles(); // deg
// }

// static void pump_events_ms(int ms) {
//     auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
//     while (std::chrono::steady_clock::now() < end) {
//         QCoreApplication::processEvents();
//         std::this_thread::sleep_for(10ms);
//     }
// }

// static int speed_from_u(double abs_u, int min_spd=1, int max_spd=30) {
//     abs_u = std::clamp(abs_u, 0.0, 1.0);
//     double s = min_spd + abs_u * (max_spd - min_spd);
//     return std::clamp<int>(static_cast<int>(std::round(s)), min_spd, max_spd);
// }

// static int cap_from_error(double ae_deg, double e_lo=2.0, double e_hi=40.0) {
//     if (ae_deg <= 0.5) return 0; // explicit stop near zero error
//     double t = (ae_deg - e_lo) / (e_hi - e_lo);
//     t = std::clamp(t, 0.0, 1.0);
//     t = t*t*(3.0 - 2.0*t); // smoothstep
//     double cap = 1.0 + t * (30.0 - 1.0);
//     return static_cast<int>(std::round(std::clamp(cap, 1.0, 30.0)));
// }

// static inline int sgn(double x) { return (x > 0.0) - (x < 0.0); }

// // --- main ------------------------------------------------------------------
// int main(int argc, char** argv) {
//     QCoreApplication app(argc, argv);

//     // Joints helper
//     constexpr Joint J[6] = {J1, J2, J3, J4, J5, J6};

//     // Targets & tolerances (deg, deg/s)
//     Angles target_deg   = {75, 74.35, -126, 53, 4, -47};  // set per joint
//     Angles target_vel   = {  0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     constexpr double POS_TOL_DEG = 2.0;
//     constexpr double VEL_TOL_DPS = 5.0;

//     // PD gains (shared; can be per-joint if you like)
//     PDController pd{0.14, 0.1, -1.0, 1.0, 0.35};

//     // Derivative protection
//     const double vel_alpha = 0.25; // EMA 0..1
//     const double vel_clip  = 30.0; // deg/s

//     // Reverse/brake policy near target
//     const auto   reverse_hysteresis = 20ms;
//     const double brake_enable_err   = 10.0; // |e| under this → allow reverse
//     const double brake_vel_min      = 2.0;

//     try {
//         auto mc = MyCobot::I();
//         mc.PowerOn();
//         pump_events_ms(1200);
//         mc.StopRobot();
//         pump_events_ms(800);

//         // Move to “home” first (optional)
//         int move_speed = 80;
//         Angles home{0, 0, 0, 0, 0, -45};
//         mc.WriteAngles(home, move_speed);
//         std::this_thread::sleep_for(3s);

//         // --- per-joint state ---
//         Angles prev_pos   = read_all(mc);
//         std::array<double, 6> vel_ema{}; // 0 init

//         // Command state (SDK calls must stay on one thread)
//         std::array<std::atomic<int>, 6> last_dir{};
//         std::array<std::atomic<int>, 6> last_spd{};
//         std::array<std::chrono::steady_clock::time_point, 6> last_dir_change;
//         auto now0 = std::chrono::steady_clock::now();
//         for (int i=0;i<6;++i) { last_dir[i]=0; last_spd[i]=0; last_dir_change[i]=now0; }

//         auto t_prev        = now0;
//         auto last_cmd_time = now0;

//         struct PreCmd {
//             int desired_dir;
//             int speed_cap;
//             int speed;
//             double e;        // position error (deg)
//             double v_filt;   // filtered velocity (deg/s)
//         };

//         // ---- control loop ----
//         while (true) {
//             auto t_now = std::chrono::steady_clock::now();
//             double dt_s = std::chrono::duration<double>(t_now - t_prev).count();
//             if (dt_s <= 0.0) dt_s = 1e-3; // guard
//             t_prev = t_now;

//             // Read all joints once per frame
//             Angles curr = read_all(mc);

//             // --- 1) PARALLEL per-joint compute (no SDK calls in workers) ---
//             std::array<PreCmd,6> pre{};
//             std::array<std::future<void>,6> futs;

//             for (int i=0;i<6;++i) {
//                 futs[i] = std::async(std::launch::async, [&, i]{
//                     // velocity estimate (each worker owns index i)
//                     double vel_inst = (curr[i] - prev_pos[i]) / dt_s;
//                     prev_pos[i] = curr[i];
//                     vel_ema[i] = vel_alpha * vel_inst + (1.0 - vel_alpha) * vel_ema[i];
//                     double v = std::clamp(vel_ema[i], -vel_clip, +vel_clip);

//                     // errors
//                     double e   = target_deg[i] - curr[i];
//                     double ae  = std::abs(e);
//                     double v_e = target_vel[i] - v;

//                     // PD
//                     double u_raw = pd.Kp * e + pd.Kd * v_e;
//                     double u = std::clamp(u_raw, -1.0, 1.0);
//                     if (std::abs(u) < pd.deadband) u = 0.0;

//                     // preliminary desired direction
//                     int ddir = (u>0) ? +1 : (u<0 ? -1 : 0);

//                     // Allow braking only near goal
//                     bool allow_brake_near_target =
//                         (ae <= brake_enable_err && std::abs(v) >= brake_vel_min);
//                     if (ddir != 0 && ddir != sgn(e)) {
//                         if (!allow_brake_near_target) ddir = sgn(e); // keep going toward target
//                     }

//                     int cap = cap_from_error(ae); // 0 or 1..30
//                     int spd = (ddir == 0 || cap == 0) ? 0
//                              : speed_from_u(std::abs(u), 1, cap);

//                     pre[i] = PreCmd{ ddir, cap, spd, e, v };
//                 });
//             }
//             for (auto &f : futs) f.get(); // join workers

//             // --- 2) MAIN THREAD: check tolerances / hysteresis / global stop ---
//             bool all_pos_ok = true, all_vel_ok = true;
//             bool any_joint_needs_stop = false, any_dir_changed_needs_stop = false;

//             int   final_dir[6] = {0};
//             int   final_spd[6] = {0};
//             int   spd_cap[6]   = {0};

//             for (int i=0;i<6;++i) {
//                 double ae  = std::abs(pre[i].e);
//                 double av_e= std::abs(target_vel[i] - pre[i].v_filt);

//                 bool pos_ok = (ae <= POS_TOL_DEG);
//                 bool vel_ok = (av_e <= VEL_TOL_DPS);
//                 all_pos_ok &= pos_ok;
//                 all_vel_ok &= vel_ok;

//                 // Direction hysteresis
//                 int dir = pre[i].desired_dir;
//                 if (dir != 0 && dir != last_dir[i] && last_dir[i] != 0) {
//                     if (t_now - last_dir_change[i] < reverse_hysteresis) {
//                         dir = last_dir[i]; // hold during hysteresis window
//                     }
//                 }

//                 int spd = (dir == 0 || pre[i].speed_cap == 0) ? 0 : pre[i].speed;

//                 // global stop conditions
//                 bool need_stop_i   = (dir == 0 || spd == 0);
//                 bool dir_changed_i = (dir != last_dir[i] && last_dir[i] != 0);
//                 any_joint_needs_stop       |= need_stop_i && (last_dir[i] != 0);
//                 any_dir_changed_needs_stop |= dir_changed_i;

//                 final_dir[i] = dir;
//                 final_spd[i] = spd;
//                 spd_cap[i]   = pre[i].speed_cap;
//             }

//             // Exit when ALL joints are within tolerances
//             if (all_pos_ok && all_vel_ok) {
//                 mc.StopRobot();
//                 pump_events_ms(400);
//                 break;
//             }

//             // Frame rate-limit
//             bool time_ok = (t_now - last_cmd_time) >= 15ms;
//             if (time_ok) {
//                 bool do_global_stop = (any_joint_needs_stop || any_dir_changed_needs_stop);

//                 if (do_global_stop) {
//                     mc.StopRobot();
//                     pump_events_ms(100);
//                     last_cmd_time = t_now;
//                     // Reset last_dir/last_spd when we actually came to a stop
//                     for (int i=0;i<6;++i) {
//                         if (last_dir[i] != 0) {
//                             last_dir[i] = 0;
//                             last_spd[i] = 0;
//                         }
//                     }
//                 }

//                 // --- 3) MAIN THREAD DISPATCH: issue all jogs for this frame ---
//                 bool any_cmd_sent = false;

//                 // (Optional) barrier: decide first, then send in fixed order
//                 for (int i=0;i<6;++i) {
//                     if (final_dir[i] != 0 && final_spd[i] > 0) {
//                         bool need_cmd = (do_global_stop
//                                          || final_dir[i] != last_dir[i]
//                                          || std::abs(final_spd[i] - last_spd[i].load()) >= 1);
//                         if (need_cmd) {
//                             mc.JogAngle(J[i], final_dir[i], final_spd[i]);
//                             last_dir[i] = final_dir[i];
//                             last_spd[i] = final_spd[i];
//                             if (final_dir[i] != 0) last_dir_change[i] = t_now;
//                             any_cmd_sent = true;
//                         }
//                     } else {
//                         // Not moving; we already issued a global stop if needed
//                         last_dir[i] = 0;
//                         last_spd[i] = 0;
//                     }
//                 }
//                 if (any_cmd_sent) last_cmd_time = t_now;
//             }

//         #if NLCTL_VERBOSE > 1
//             // Debug printout
//             std::cout << "[Frame]\n";
//             for (int i=0;i<6;++i) {
//                 std::cout << "  J" << (i+1)
//                           << " pos=" << prev_pos[i]
//                           << " dir=" << (final_dir[i]>0?"+":(final_dir[i]<0?"-":"0"))
//                           << " spd=" << final_spd[i]
//                           << " cap=" << spd_cap[i]
//                           << "\n";
//             }
//         #endif

//             // ~50 Hz service
//             int sleep_ms = 20 - static_cast<int>(std::round(dt_s * 1000.0));
//             if (sleep_ms > 0) pump_events_ms(sleep_ms);
//         }

//         // End: go back to home
//         mc.WriteAngles(home, move_speed);
//         std::this_thread::sleep_for(3s);
//         mc.StopRobot();
//         pump_events_ms(300);

//         Angles final_curr = read_all(mc);
//         std::cout << "YAY! PD control complete. Final [J1..J6] ~ ["
//                 << final_curr[0] << ", " << final_curr[1] << ", " << final_curr[2] << ", "
//                 << final_curr[3] << ", " << final_curr[4] << ", " << final_curr[5]
//                 << "] deg.\n";

//         return 0;
//     }
//     catch (const std::error_code &e) {
//         std::cerr << "SDK error [" << e.value() << "]: " << e.message() << "\n";
//         return 1;
//     }
// }
