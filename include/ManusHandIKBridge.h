// File: include/ManusHandIKBridge.h
#pragma once

#include "hand_ik.hpp"
#include "ManusConfig.h"
#include <Eigen/Dense>
#include <memory>
#include <array>

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
    // Constructor with config file support
    explicit ManusHandIKBridge(const std::string& urdf_path, const std::string& config_path = "");
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

    // Access to loaded configuration
    const ManusIntegrationConfig& GetConfig() const { return manus_config_; }

    // NEW: Internal state management for proper FK/Jacobian updates
    void ForceKinematicsUpdate(const Eigen::VectorXd& q_full);
    void ApplyWorkingJointLimits(Eigen::VectorXd& qa) const;

    // NEW: Frame ID access for debugging
    void PrintFrameMapping() const;
    bool ValidateFrameIDs() const;

private:
    std::unique_ptr<hand_ik::HandIK> ik_solver_;
    hand_ik::HandIKConfig config_;
    ManusIntegrationConfig manus_config_;

    // Coordinate system parameters
    double position_scale_ = 1.0;    // Manus to IK position scaling
    Eigen::Matrix3d coord_transform_; // Coordinate frame transformation matrix

    // Performance tracking
    mutable uint64_t solve_count_ = 0;
    mutable double total_solve_time_ms_ = 0.0;

    // NEW: Cached frame IDs for consistent fingertip access
    std::array<pinocchio::FrameIndex, 4> fingertip_frame_ids_;
    pinocchio::FrameIndex thumb_tip_frame_id_;

    // NEW: Working joint limits (from successful test configuration)
    std::array<std::array<double, 2>, 4> working_mcp_limits_;
    std::array<double, 2> working_thumb_rot_limits_;
    std::array<double, 2> working_thumb_flex_limits_;

    // NEW: Iteration counter for FK/Jacobian recompute tracking
    mutable uint32_t fk_recompute_count_ = 0;

    void InitializeConfig();
    void SetupCoordinateTransform();
    hand_ik::Targets ConvertTargets(const FingerTargets& manus_targets) const;

    // NEW: Enhanced solver methods with proper FK/Jacobian management
    bool SolveWithRecomputedJacobians(const FingerTargets& manus_targets, JointConfiguration& joint_config);
    void RecomputeKinematicsAndJacobians(const Eigen::VectorXd& q_full);
    bool ValidateJacobianColumns(const Eigen::MatrixXd& J_analytical) const;

    // NEW: Frame validation and setup
    void CacheFingertipFrameIDs();
    void SetupWorkingJointLimits();
};