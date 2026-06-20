#pragma once
#include <vector>
#include <cmath>

namespace control_core {

struct Point2D {
    double x;
    double y;
};

struct Pose2D {
    double x;
    double y;
    double yaw;
    double v; // Current velocity
};

struct ControlCommand {
    double steering_angle;
    double target_velocity;
    double acceleration;
};

inline double wrapToPi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

} // namespace control_core
