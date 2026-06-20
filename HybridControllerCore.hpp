#pragma once
#include "ControlDataTypes.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace control_core {

class HybridControllerCore {
public:
    struct Config {
        double L_base = 1.53;
        double L_min = 3.0;
        double k_pure = 0.75;
        double k_stanley = 0.3;
        double k_soft = 3.5;
        double delta_max = 0.35;
        double mu = 0.45;
        double g = 9.81;
        double v_max = 5.0;
        double v_min = 0.5;
        double a_dec_max = 4.0;
        double a_acc_max = 1.5;
        int lookahead_points = 150;
        double Kp_v = 1.0;
    };

    HybridControllerCore(const Config& config = Config()) : cfg_(config), last_delta_(0.0), last_idx_(0) {}

    void setPath(const std::vector<Point2D>& path) {
        if (path.size() < 3) return;
        path_ = path;
        path_kappa_.assign(path.size(), 0.0);

        // Compute local trajectory curvature profiles
        for (size_t i = 1; i < path.size() - 1; ++i) {
            double ax = path[i-1].x, ay = path[i-1].y;
            double bx = path[i].x,   by = path[i].y;
            double cx = path[i+1].x, cy = path[i+1].y;

            double ab = std::hypot(bx - ax, by - ay);
            double bc = std::hypot(cx - bx, cy - by);
            double ca = std::hypot(ax - cx, ay - cy);

            double cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
            double denom = std::max(ab * bc * ca, 1e-9);
            path_kappa_[i] = 2.0 * cross / denom;
        }
        path_kappa_[0] = path_kappa_[1];
        path_kappa_.back() = path_kappa_[path_.size() - 2];
        last_idx_ = 0;
    }

    ControlCommand computeCommand(const Pose2D& state, double dt) {
        if (path_.empty() || dt <= 0.0) return {0.0, 0.0, 0.0};

        int N = path_.size();
        if (last_idx_ >= N) last_idx_ = 0;

        // 1. Sliding Window Nearest Neighbor Search
        const int SEARCH_WINDOW = 60;
        int search_start = std::max(0, last_idx_ - 5);       
        int search_end   = std::min(N - 1, last_idx_ + SEARCH_WINDOW);

        double min_dist_sq_rear = 1e9;
        int idx_rear = last_idx_;
        for (int i = search_start; i <= search_end; ++i) {
            double dist_sq = std::pow(path_[i].x - state.x, 2) + std::pow(path_[i].y - state.y, 2);
            if (dist_sq < min_dist_sq_rear) { min_dist_sq_rear = dist_sq; idx_rear = i; }
        }
        last_idx_ = idx_rear;  

        // 2. Pure Pursuit Core Logic
        double L_ld = std::max(cfg_.L_min, cfg_.k_pure * std::max(0.0, state.v));
        double arc = 0.0;
        int idx_ld = idx_rear;
        while (idx_ld < N - 1) {
            double ds = std::hypot(path_[idx_ld + 1].x - path_[idx_ld].x,
                                   path_[idx_ld + 1].y - path_[idx_ld].y);
            if (arc + ds >= L_ld) break;
            arc += ds;
            idx_ld++;
        }

        double alpha = std::atan2(path_[idx_ld].y - state.y, path_[idx_ld].x - state.x) - state.yaw;
        alpha = wrapToPi(alpha);
        double delta_pp = std::atan2(2.0 * cfg_.L_base * std::sin(alpha), L_ld);

        // 3. Stanley Core Logic
        double xf = state.x + cfg_.L_base * std::cos(state.yaw);
        double yf = state.y + cfg_.L_base * std::sin(state.yaw);
        
        int idx_front = idx_rear;
        double min_dist_sq_front = 1e9;
        int front_search_end = std::min(N - 1, idx_rear + SEARCH_WINDOW);
        for (int i = idx_rear; i <= front_search_end; ++i) {
            double dist_sq = std::pow(path_[i].x - xf, 2) + std::pow(path_[i].y - yf, 2);
            if (dist_sq < min_dist_sq_front) { min_dist_sq_front = dist_sq; idx_front = i; }
        }

        double sum_x = 0.0, sum_y = 0.0;
        int pts_to_average = std::min(4, N - 1 - idx_front);
        for(int i = 0; i < pts_to_average; ++i) {
            double seg_heading = std::atan2(path_[idx_front + i + 1].y - path_[idx_front + i].y, 
                                            path_[idx_front + i + 1].x - path_[idx_front + i].x);
            sum_x += std::cos(seg_heading);
            sum_y += std::sin(seg_heading);
        }
        
        double path_heading = (pts_to_average > 0) ? std::atan2(sum_y, sum_x) : state.yaw;
        double psi_e = wrapToPi(path_heading - state.yaw);
        double ef = -(path_[idx_front].x - xf) * std::sin(path_heading) + (path_[idx_front].y - yf) * std::cos(path_heading);
        ef = std::clamp(ef, -0.6, 0.6);
        
        double delta_stanley = psi_e + std::atan2(cfg_.k_stanley * ef, cfg_.k_soft + std::max(0.0, state.v));

        // 4. Control Fusion & Low Pass Filter
        double raw_delta = std::clamp((0.5 * delta_stanley) + (0.5 * delta_pp), -cfg_.delta_max, cfg_.delta_max);
        double alpha_lp = std::clamp(dt / (0.05 + dt), 0.0, 1.0);
        double delta = alpha_lp * raw_delta + (1.0 - alpha_lp) * last_delta_;
        last_delta_ = delta;

        // 5. Kinematic Velocity Profiler
        int end_idx = std::min(idx_front + cfg_.lookahead_points, N - 1);
        int seg_len = end_idx - idx_front + 1;
        std::vector<double> v_limits(seg_len, cfg_.v_max);

        for (int i = idx_front; i <= end_idx; ++i) {
            double k = std::abs(path_kappa_[i]);
            v_limits[i - idx_front] = (k > 0.01) ? std::min(cfg_.v_max, std::sqrt((cfg_.mu * cfg_.g) / k)) : cfg_.v_max;
        }

        for (int i = end_idx - 1; i >= idx_front; --i) {
            double ds = std::max(std::hypot(path_[i+1].x - path_[i].x, path_[i+1].y - path_[i].y), 0.01);
            double v_brake = std::sqrt(std::pow(v_limits[i - idx_front + 1], 2) + 2.0 * cfg_.a_dec_max * ds);
            v_limits[i - idx_front] = std::min(v_limits[i - idx_front], v_brake);
        }

        double v_target = std::max(v_limits[0], cfg_.v_min);
        double v_err = v_target - state.v;
        double acc = std::clamp(cfg_.Kp_v * v_err, -cfg_.a_dec_max, cfg_.a_acc_max);

        return {delta, v_target, acc};
    }

private:
    Config cfg_;
    std::vector<Point2D> path_;
    std::vector<double> path_kappa_;
    double last_delta_;
    int last_idx_;
};

} // namespace control_core
