# 🏎️ High-Speed Control Library

![C++](https://img.shields.io/badge/C++-17-blue)
![Control](https://img.shields.io/badge/Algorithm-Stanley%20%7C%20Pure%20Pursuit-orange)
![State](https://img.shields.io/badge/Status-Production%20Ready-green)

A pure C++ mathematical library for high-speed lateral and longitudinal autonomous vehicle control. This repository strips away all middleware (like ROS/ROS 2) to provide a lightweight, hyper-fast, and highly portable suite of control algorithms designed for Formula Student and high-performance autonomous racing.

This library is designed using the **Wrapper Pattern**, meaning it can be directly compiled into embedded systems (like an STM32), Software-in-the-Loop (SIL) physics engines, or wrapped into a proprietary ROS 2 node.

---

## 📦 Core Algorithms

### 1. Hybrid Controller (`HybridControllerCore.hpp`)
Our flagship control algorithm that fuses the best of two paradigms into a single, cohesive actuator command:
* **Lateral Tracking:** Blends a **Stanley Controller** (which perfectly eliminates localized cross-track error using the front-axle reference) with a **Pure Pursuit Controller** (which provides high-speed lookahead stability to prevent oscillation).
* **Longitudinal Profiling:** Includes a built-in kinematic velocity profiler. It computes the mathematical curvature ($\kappa$) of the path and performs a forward-pass calculation of the maximum friction-limited cornering speed ($v = \sqrt{\mu g / \kappa}$), followed by a backward-pass predictive deceleration profile.

### 2. Standalone Pure Pursuit (`PurePursuitCore.hpp`)
A lightweight, highly optimized Pure Pursuit implementation designed for environments where the speed profile is pre-computed and provided externally.
* **Smart Behavioral Tracking:** Accepts an external `speed_profile` array (e.g., from an upstream Machine Learning or Optimal Control node). 
* **Predictive Braking Scanner:** Acts as a safety layer by scanning up to 80 waypoints ahead to ensure the car physically obeys the maximum deceleration limits required to hit those external target speeds safely.

---

## 🏗️ Data Architecture

To ensure maximum portability, the library relies strictly on standard C++ vectors and lightweight structs defined in `ControlDataTypes.hpp`:

* **`Point2D`**: Simple coordinate points (`x`, `y`).
* **`Pose2D`**: The full vehicle state (`x`, `y`, `yaw`, current `v`).
* **`ControlCommand`**: The optimized output to send to your vehicle's drive-by-wire system (`steering_angle`, `target_velocity`, `acceleration`).

---

## ⚙️ Prerequisites

* **C++ Compiler:** C++17 or higher
* **Build System:** CMake (3.10+)
* *Zero external dependencies! (No Eigen, No ROS, No Gazebo)*

---
