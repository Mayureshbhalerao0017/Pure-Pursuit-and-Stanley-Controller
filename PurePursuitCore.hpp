#pragma once
#include "ControlDataTypes.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace control_core {

class PurePursuitCore {
public:
    struct Config {
        double L_base = 1.53;           // Wheelbase
        double L_min = 2.0;             // Minimum lookahead distance
        double k_pure = 0.5;            // Velocity-scaled lookahead gain
        double delta_max = 0.5;         // Max steering angle (radians)
        double max_speed_limit = 3.0;   // Absolute max speed
        double max_accel = 1.0;         // Max positive acceleration
        double max_decel = 4.0;         // Max braking capability
    };

    PurePursuitCore(const Config& config = Config()) : cfg_(config), last_steering_(0.0), last_closest_idx_(0) {}

    // Load the physical path coordinates
    void setPath(const std::vector<Point2D>& path) {
        path_ = path;
        last_closest_idx_ = 0;
    }

    // Load an externally calculated speed profile (e.g., from a behavioral planner)
    void setSpeedProfile(const std::vector<double>& speed_profile) {
        speed_profile_ = speed_profile;
    }

    ControlCommand computeCommand(const Pose2D& state, double dt) {
        if (path_.size() < 2 || dt <= 0.0) return {0.0, 0.0, 0.0};

        int N = path_.size();
        if (last_closest_idx_ >= N) last_closest_idx_ = 0;

        // 1. Find the closest point to the rear axle
        double min_d = 1e9;
        for (int i = 0; i < N; ++i) {
            double d = std::hypot(path_[i].x - state.x, path_[i].y - state.y);
            if (d < min_d) { min_d = d; last_closest_idx_ = i; }
        }

        // 2. Calculate dynamic lookahead and find the target point
        double Ld = cfg_.L_min + cfg_.k_pure * std::abs(state.v);
        int idx_ld = last_closest_idx_;
        
        for (int i = last_closest_idx_; i < N; ++i) {
            double dx = path_[i].x - state.x;
            double dy = path_[i].y - state.y;
            
            // Check distance AND ensure the point is physically in front of the car
            if (std::hypot(dx, dy) >= Ld && (dx * std::cos(state.yaw) + dy * std::sin(state.yaw)) > 0.0) { 
                idx_ld = i; 
                break; 
            }
            if (i == N - 1) idx_ld = N - 1; // Fallback to end of path
        }

        // 3. Pure Pursuit Lateral Steering Calculation
        double tx = path_[idx_ld].x;
        double ty = path_[idx_ld].y;
        double dx = tx - state.x; 
        double dy = ty - state.y;
        
        // Transform target into vehicle's local coordinate frame
        double ly = -dx * std::sin(state.yaw) + dy * std::cos(state.yaw);
        
        double Ld_sq = dx * dx + dy * dy;
        if (Ld_sq < 0.001) Ld_sq = 0.001; // Prevent division by zero
        
        double raw_delta = std::atan2(2.0 * cfg_.L_base * ly, Ld_sq);
        raw_delta = std::clamp(raw_delta, -cfg_.delta_max, cfg_.delta_max);

        // Simple low-pass filter to prevent steering jitter
        double smoothed_steering = (0.60 * raw_delta) + (0.40 * last_steering_);
        last_steering_ = smoothed_steering;

        // 4. Longitudinal Control (Predictive Braking Scanner)
        double target_velocity = cfg_.max_speed_limit; 
        int velocity_scan_limit = std::min(last_closest_idx_ + 80, N - 1);
        
        for (int i = last_closest_idx_; i <= velocity_scan_limit; ++i) {
            double node_speed = cfg_.max_speed_limit;
            if (i < static_cast<int>(speed_profile_.size())) {
                node_speed = speed_profile_[i];
            }
            
            double dist_to_node = std::hypot(path_[i].x - state.x, path_[i].y - state.y);
            
            // Kinematic formula: v_initial = sqrt(v_final^2 + 2 * a * d)
            double required_current_speed = std::sqrt(std::max(0.0, (node_speed * node_speed) + (2.0 * cfg_.max_decel * dist_to_node)));
            
            if (required_current_speed < target_velocity) {
                target_velocity = required_current_speed;
            }
        }

        // Final safety bounds to prevent full-stopping on sharp hairpins
        target_velocity = std::clamp(target_velocity, 1.5, cfg_.max_speed_limit);
        
        // Bang-bang acceleration target based on required velocity
        double acceleration = (target_velocity < state.v) ? -cfg_.max_decel : cfg_.max_accel;

        return {smoothed_steering, target_velocity, acceleration};
    }

    // Helper to get the Lookahead coordinates for external visualization (e.g., RViz)
    Point2D getLookaheadPoint() const {
        if (path_.empty() || last_closest_idx_ >= path_.size()) return {0.0, 0.0};
        
        int idx_ld = last_closest_idx_;
        // Simple recalculation to return the point without running the full math
        double Ld = cfg_.L_min; 
        for (size_t i = last_closest_idx_; i < path_.size(); ++i) {
            // Approximation for visualization purposes
            idx_ld = i;
            if (i - last_closest_idx_ > 15) break; 
        }
        return path_[idx_ld];
    }

private:
    Config cfg_;
    std::vector<Point2D> path_;
    std::vector<double> speed_profile_;
    double last_steering_;
    int last_closest_idx_;
};

} // namespace control_core
