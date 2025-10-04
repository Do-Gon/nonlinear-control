#pragma once
#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <QCoreApplication>
#include "mycobot/MyCobot.hpp"

//
// Thin PD math (unchanged)
//
struct PDController {
    double Kp{1.0};
    double Kd{0.1};
    double u_min{-1.0};   // normalized output lower bound
    double u_max{ 1.0};   // normalized output upper bound
    double deadband{0.02}; // |u| below this -> 0

    PDController() = default;
    PDController(double kp, double kd, double umin=-1.0, double umax=1.0, double db=0.02)
        : Kp(kp), Kd(kd), u_min(umin), u_max(umax), deadband(db) {}

    // target_vel defaults to 0 (hold-still). Units: pos in deg, vel in deg/s.
    double compute(double target_pos, double pos, double vel, double target_vel = 0.0) const {
        const double e  = target_pos - pos;
        const double de = target_vel - vel;
        double u = Kp*e + Kd*de;
        u = std::clamp(u, u_min, u_max);
        if (std::abs(u) < deadband) u = 0.0;
        return u;
    }
};

//
// Runtime wrapper that implements filtering, speed capping, direction/brake policy,
// hysteresis and command hygiene, then drives MyCobot J1 with JogAngle().
//
struct PDRunnerConfig {
    // tolerances
    double pos_tol_deg{1.0};
    double vel_tol_dps{5.0};

    // derivative protection
    double vel_alpha{0.25};      // EMA factor (0..1)
    double vel_clip{60.0};       // clamp filtered velocity (deg/s)

    // speed mapping
    int    min_spd{1};           // SDK min jog speed (1..30)
    int    max_spd{50};          // SDK max jog speed
    double cap_e_lo{2.0};        // error where cap starts to rise
    double cap_e_hi{15.0};       // error where cap reaches max

    // reversing/braking policy
    double brake_enable_err{30.0}; // allow reverse only when |e| <= this
    double brake_vel_min{2.0};     // and |vel| >= this (deg/s)

    // command hygiene
    std::chrono::milliseconds reverse_hysteresis{100}; // min time between flips
    std::chrono::milliseconds min_cmd_period{15};     // rate limit to SPAM less

    // verbosity: 0=silent, 1=key, 2=per-iteration
    int verbose{0};
};

class J1PDControllerRunner {
public:
    explicit J1PDControllerRunner(const PDController& pd, const PDRunnerConfig& cfg)
        : pd_(pd), cfg_(cfg) {}

    // Runs the closed-loop jog to target; returns when inside tolerances.
    // target_vel_dps usually 0.0 (hold still at the target).
    void run_to_target(mycobot::MyCobot& mc,
                       double target_deg,
                       double target_vel_dps = 0.0) const
    {
        using namespace std::chrono;
        using namespace std::chrono_literals;

        // --- helpers (header-only safe: inline in method scope) ---
        auto pump_events_ms = [](int ms) {
            auto end = steady_clock::now() + milliseconds(ms);
            while (steady_clock::now() < end) {
                QCoreApplication::processEvents();
                std::this_thread::sleep_for(10ms);
            }
        };
        auto read_j1 = [](const mycobot::MyCobot& m) -> double {
            auto a = m.GetAngles();
            return a[0];
        };
        auto sgn = [](double x) -> int { return (x > 0.0) - (x < 0.0); };

        auto cap_from_error = [&](double ae_deg) -> int {
            if (ae_deg <= 0.5) return 0; // explicit stop window
            double t = (ae_deg - cfg_.cap_e_lo) / (cfg_.cap_e_hi - cfg_.cap_e_lo);
            t = std::clamp(t, 0.0, 1.0);
            // smoothstep
            t = t*t*(3.0 - 2.0*t);
            double cap = cfg_.min_spd + t * (cfg_.max_spd - cfg_.min_spd);
            return static_cast<int>(std::round(std::clamp(cap, double(cfg_.min_spd), double(cfg_.max_spd))));
        };
        auto speed_from_u = [&](double abs_u, int spd_cap) -> int {
            abs_u = std::clamp(abs_u, 0.0, 1.0);
            double s = cfg_.min_spd + abs_u * (spd_cap - cfg_.min_spd);
            return std::clamp<int>(static_cast<int>(std::round(s)), cfg_.min_spd, spd_cap);
        };

        // --- initial reads ---
        double curr = read_j1(mc);
        double vel_ema = 0.0;

        auto t_prev = steady_clock::now();
        double prev_pos = curr;

        int last_dir = 0;
        int last_spd = 0;
        auto last_dir_change = t_prev;
        auto last_cmd_time   = t_prev;

        // --- control loop ---
        while (true) {
            auto t_now = steady_clock::now();
            double dt_s = duration<double>(t_now - t_prev).count();
            if (dt_s <= 0.0) dt_s = 1e-3;
            t_prev = t_now;

            curr = read_j1(mc);

            // velocity estimate (finite difference + EMA + clip)
            double vel_inst = (curr - prev_pos) / dt_s;
            prev_pos = curr;

            vel_ema = cfg_.vel_alpha * vel_inst + (1.0 - cfg_.vel_alpha) * vel_ema;
            double vel_filtered = std::clamp(vel_ema, -cfg_.vel_clip, +cfg_.vel_clip);

            // controller output
            double u = pd_.compute(target_deg, curr, vel_filtered, target_vel_dps);
            int desired_dir = sgn(u);

            // braking near target: only allow reversing past target if close & moving
            double e  = target_deg - curr;
            double ae = std::abs(e);
            if (desired_dir != 0 && desired_dir != sgn(e)) {
                if (!(ae <= cfg_.brake_enable_err && std::abs(vel_filtered) >= cfg_.brake_vel_min)) {
                    desired_dir = sgn(e); // keep moving toward target
                }
            }

            // direction hysteresis
            int dir = desired_dir;
            if (dir != 0 && dir != last_dir && last_dir != 0) {
                if (t_now - last_dir_change < cfg_.reverse_hysteresis) {
                    dir = last_dir; // hold previous direction within hysteresis window
                }
            }

            // speed selection with error-based cap
            int spd_cap = cap_from_error(ae); // 0 or [min..max]
            int spd = (dir == 0 || spd_cap == 0) ? 0 : speed_from_u(std::abs(u), spd_cap);

            // stopping condition (position & velocity tolerance)
            bool pos_ok = (ae <= cfg_.pos_tol_deg);
            bool vel_ok = (std::abs(target_vel_dps - vel_filtered) <= cfg_.vel_tol_dps);
            if (pos_ok && vel_ok) {
                mc.StopRobot();
                pump_events_ms(500);
                if (cfg_.verbose > 0) {
                    std::cout << "PD: settled at J1=" << curr << " deg (|e|=" << ae
                              << ", |v_err|=" << std::abs(target_vel_dps - vel_filtered) << ")\n";
                }
                break;
            }

            // command hygiene
            bool need_stop   = (dir == 0 || spd == 0);
            bool dir_changed = (dir != last_dir && last_dir != 0);
            bool spd_changed = (std::abs(spd - last_spd) >= 1);
            bool time_ok     = (t_now - last_cmd_time) >= cfg_.min_cmd_period;

            if (need_stop) {
                if (last_dir != 0 && time_ok) {
                    mc.StopRobot();
                    pump_events_ms(100);
                    last_cmd_time = t_now;
                    last_dir = 0;
                    last_spd = 0;
                }
            } else {
                if (dir_changed && time_ok) {
                    mc.StopRobot();
                    pump_events_ms(100);
                    last_dir_change = t_now;
                }
                if ((dir_changed || spd_changed) && time_ok) {
                    mc.JogAngle(mycobot::J1, dir, spd);
                    last_cmd_time = t_now;
                    last_dir = dir;
                    last_spd = spd;
                }
            }

            if (cfg_.verbose > 1) {
                std::cout << "[J1] pos=" << curr
                          << " deg, vel_filt=" << vel_filtered
                          << " dps, e=" << e
                          << ", u=" << u
                          << " -> dir=" << (dir>0?"+":(dir<0?"-":"0"))
                          << ", cap=" << spd_cap
                          << ", spd=" << spd << "\n";
            }

            // Aim ~50 Hz service
            int sleep_ms = 20 - static_cast<int>(std::round(dt_s * 1000.0));
            if (sleep_ms > 0) pump_events_ms(sleep_ms);
        }
    }

private:
    PDController   pd_;
    PDRunnerConfig cfg_;
};


// #pragma once
// #include <algorithm>
// #include <cmath>

// struct PDController {
//     double Kp{1.0};
//     double Kd{0.1};
//     double u_min{-1.0};   // normalized output lower bound
//     double u_max{ 1.0};   // normalized output upper bound
//     double deadband{0.02}; // |u| below this -> 0

//     PDController() = default;
//     PDController(double kp, double kd, double umin=-1.0, double umax=1.0, double db=0.02)
//         : Kp(kp), Kd(kd), u_min(umin), u_max(umax), deadband(db) {}

//     // target_vel defaults to 0 (hold-still target). Units: pos in deg, vel in deg/s.
//     // Returns normalized u in [-1, 1].
//     double compute(double target_pos, double pos, double vel, double target_vel = 0.0) const {
//         const double e  = target_pos - pos;
//         const double de = target_vel - vel;  // if target_vel==0 => de = -vel
//         double u = Kp*e + Kd*de;
//         u = std::clamp(u, u_min, u_max);
//         if (std::abs(u) < deadband) u = 0.0;
//         return u;
//     }
// };
