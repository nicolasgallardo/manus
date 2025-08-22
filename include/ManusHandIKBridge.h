// File: include/ManusHandIKBridge.h
#pragma once

#include "hand_ik.hpp"
#include "ManusConfig.h"
#include <Eigen/Dense>
#include <memory>
#include <array>
#include <map>

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
    double jacobian_error = 0.0;  // NEW: Max FD error for diagnostics
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

    // NEW: Enhanced joint mapping and validation
    void ValidateJointMapping();
    bool ValidateFrameIDs() const;
    void PrintFrameMapping() const;

    // NEW: Moving planes and FD checking
    void SetUseFDCheck(bool enable) { use_fd_check_ = enable; }
    void SetUseMovingPlanes(bool enable) { use_moving_planes_ = enable; }
    void SetPlaneTolerance(double tolerance);

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

    // NEW: Joint mapping resolved by name lookup
    std::map<std::string, pinocchio::JointIndex> joint_name_to_id_;
    std::map<std::string, pinocchio::FrameIndex> frame_name_to_id_;

    // Cached joint indices (resolved by name)
    std::array<pinocchio::JointIndex, 4> mcp_joint_ids_;
    std::array<pinocchio::JointIndex, 4> distal_joint_ids_;
    pinocchio::JointIndex thumb_rot_joint_id_;
    pinocchio::JointIndex thumb_flex_joint_id_;

    // Cached frame indices (resolved by name)
    std::array<pinocchio::FrameIndex, 4> fingertip_frame_ids_;
    pinocchio::FrameIndex thumb_tip_frame_id_;

    // NEW: Enhanced solver options
    bool use_moving_planes_ = true;     // Recompute planes each iteration
    bool use_fd_check_ = false;         // Enable finite difference validation
    uint32_t jacobian_recompute_count_ = 0;

    void InitializeConfig();
    void SetupCoordinateTransform();
    hand_ik::Targets ConvertTargets(const FingerTargets& manus_targets) const;

    // NEW: Enhanced joint mapping by name
    void ResolveJointIndicesByName();
    void BuildJointNameMaps();
    pinocchio::JointIndex LookupJointByName(const std::string& joint_name) const;
    pinocchio::FrameIndex LookupFrameByName(const std::string& frame_name) const;

    // NEW: Robust plane construction from joint axes
    Eigen::Vector3d GetJointAxis(pinocchio::JointIndex joint_id) const;
    Eigen::Vector3d GetRobustPlaneNormal(int finger_idx) const;

    // NEW: Enhanced solver with FD validation option
    bool SolveWithEnhancedValidation(const FingerTargets& manus_targets, JointConfiguration& joint_config);
    double ValidateJacobianFiniteDifference(const Eigen::VectorXd& qa) const;
};