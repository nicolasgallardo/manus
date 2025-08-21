// File: include/ManusHandIKBridge.h
#pragma once

#include "hand_ik.hpp"
#include <Eigen/Dense>
#include <memory>

enum class HandSide {
    Left,
    Right,
    Both
};

// Structure for finger target positions from Manus
struct FingerTargets {
    std::array<Eigen::Vector3d, 4> finger_positions;  // index, middle, ring, pinky
    Eigen::Vector3d thumb_position;
    Eigen::Matrix3d thumb_rotation;
};

// Structure for solved joint configuration
struct JointConfiguration {
    std::array<double, 6> joint_angles;  // 4 MCP + 2 thumb joints
    bool valid = false;
    double solve_time_ms = 0.0;
    int iterations = 0;
};

// Bridge between Manus coordinate system and Hand IK
class ManusHandIKBridge {
public:
    explicit ManusHandIKBridge(const std::string& urdf_path);
    ~ManusHandIKBridge() = default;

    // Main solving interface
    bool Solve(const FingerTargets& manus_targets, JointConfiguration& joint_config);

    // Coordinate system conversion utilities
    Eigen::Vector3d ManusToIKPosition(const Eigen::Vector3d& manus_pos) const;
    Eigen::Matrix3d ManusToIKOrientation(const Eigen::Matrix3d& manus_rot) const;

    // Configuration and diagnostics
    bool IsInitialized() const { return ik_solver_ != nullptr; }
    void SetVerbose(bool verbose);
    bool RunDiagnostics();

private:
    std::unique_ptr<hand_ik::HandIK> ik_solver_;
    hand_ik::HandIKConfig config_;

    // Coordinate system parameters
    double position_scale_ = 1.0;    // Manus to IK position scaling
    Eigen::Matrix3d coord_transform_; // Coordinate frame transformation matrix

    // Performance tracking
    mutable uint64_t solve_count_ = 0;
    mutable double total_solve_time_ms_ = 0.0;

    void InitializeConfig();
    void SetupCoordinateTransform();
    hand_ik::Targets ConvertTargets(const FingerTargets& manus_targets) const;
};