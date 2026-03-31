#ifndef PPCA_SRC_HPP
#define PPCA_SRC_HPP

#include "math.h"
class Monitor; // forward declaration; defined by the judge driver

class Controller {

public:
    Controller(const Vec &_pos_tar, double _v_max, double _r, int _id, Monitor *_monitor) {
        pos_tar = _pos_tar;
        v_max = _v_max;
        r = _r;
        id = _id;
        monitor = _monitor;
    }

    void set_pos_cur(const Vec &_pos_cur) {
        pos_cur = _pos_cur;
    }

    void set_v_cur(const Vec &_v_cur) {
        v_cur = _v_cur;
    }

private:
    int id;
    Vec pos_tar;
    Vec pos_cur;
    Vec v_cur;
    double v_max, r;
    Monitor *monitor;

    // Check whether a candidate velocity is safe against all other robots
    bool safe_velocity(const Vec &v_candidate) const {
        int n = monitor->get_robot_number();
        for (int j = 0; j < n; ++j) {
            if (j == id) continue;
            Vec other_pos = monitor->get_pos_cur(j);
            Vec other_v = monitor->get_v_cur(j);

            Vec delta_pos = pos_cur - other_pos;
            Vec delta_v = v_candidate - other_v;

            double project = delta_pos.dot(delta_v);
            if (project >= 0) {
                // Moving away or not closing in
                continue;
            }

            double delta_v_norm = delta_v.norm();
            // If relative speed is almost zero, treat as safe (no approach)
            if (delta_v_norm <= 1e-12) {
                continue;
            }

            // Follow same logic as simulator to estimate minimal distance within TIME_INTERVAL
            project /= -delta_v_norm;
            double min_dis_sqr;
            double delta_r = r + monitor->get_r(j);
            if (project < delta_v_norm * TIME_INTERVAL) {
                min_dis_sqr = delta_pos.norm_sqr() - project * project;
            } else {
                min_dis_sqr = (delta_pos + delta_v * TIME_INTERVAL).norm_sqr();
            }
            if (min_dis_sqr <= delta_r * delta_r - EPSILON) {
                return false;
            }
        }
        return true;
    }

    Vec clamp_speed(const Vec &v) const {
        double max_allow = v_max > 1e-9 ? (v_max - 1e-4) : 0.0; // keep a small margin
        double n = v.norm();
        if (n <= max_allow) return v;
        if (n <= 1e-12) return Vec();
        return v * (max_allow / n);
    }

public:

    Vec get_v_next() {
        // If already at target (within EPSILON), stop
        Vec to_tar = pos_tar - pos_cur;
        double dist = to_tar.norm();
        if (dist <= EPSILON) {
            return Vec();
        }

        // Desired speed: try to reach target within one step if close enough
        double max_step = TIME_INTERVAL * (v_max > 1e-9 ? (v_max - 1e-4) : 0.0);
        double desired_speed = (dist <= max_step) ? (dist / TIME_INTERVAL) : (v_max - 1e-4);
        if (desired_speed < 0) desired_speed = 0;
        Vec v_desired = (dist > 1e-12) ? to_tar.normalize() * desired_speed : Vec();
        v_desired = clamp_speed(v_desired);

        // Try the desired velocity first
        if (safe_velocity(v_desired)) return v_desired;

        // Try scaled velocities to avoid collisions
        const double scales[] = {0.7, 0.5, 0.3, 0.1, 0.0};
        for (double s : scales) {
            Vec v_try = v_desired * s;
            if (safe_velocity(v_try)) return v_try;
        }

        // Fallback: small detour perpendicular to target direction
        if (dist > 1e-12) {
            Vec dir = to_tar.normalize();
            // Perpendicular vectors
            Vec perp1(-dir.y, dir.x);
            Vec perp2(dir.y, -dir.x);
            double side_speed = (v_max - 1e-4) * 0.3;
            // Choose side preference based on id to break symmetry
            if (id % 2 == 0) {
                Vec side1 = clamp_speed(perp1 * side_speed);
                if (safe_velocity(side1)) return side1;
                Vec side2 = clamp_speed(perp2 * side_speed);
                if (safe_velocity(side2)) return side2;
            } else {
                Vec side2 = clamp_speed(perp2 * side_speed);
                if (safe_velocity(side2)) return side2;
                Vec side1 = clamp_speed(perp1 * side_speed);
                if (safe_velocity(side1)) return side1;
            }
        }

        // As a last resort, stop
        return Vec();
    }
};


#endif // PPCA_SRC_HPP
