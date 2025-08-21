// File: src/ManusHandIKBridge.cpp (REPLACE your existing manushandikbridge.cpp)
#include "ManusHandIKBridge.h"
#include "math_constants.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>

ManusHandIKBridge::ManusHandIKBridge(const std::string& urdf_path) {
    InitializeConfig();
    SetupCoordinateTransform();

    try {
        ik_solver_ = std::make_unique<hand_ik::HandIK>(config_, urdf_path);
        std::cout << "✓ Hand IK bridge initialized with URDF: " << urdf_path << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "❌ Failed to initialize Hand IK solver: " << e.what() << std::endl;
        throw;
    }
}

void ManusHandIKBridge::InitializeConfig() {
    // Use the same configuration as our validated tests
    config_.mcp_joint_names = {
        "Index_MCP_Joint", "Middle_MCP_Joint", "Ring_MCP_Joint", "Pinky_MCP_Joint"
    };
    config_.distal_joint_names = {
        "Index_PIP_Joint", "Middle_PIP_Joint", "Ring_PIP_Joint", "Pinky_PIP_Joint"
    };
    config_.fingertip_frame_names = {
        "Index_Distal", "Middle_Distal", "Ring_Distal", "Pinky_Distal"
    };

    config_.thumb_rot_joint_name = "Metacarpal_Joint";
    config_.thumb_flex_joint_name = "Thumb_Joint";
    config_.thumb_tip_frame_name = "Thumb";

    // Verified passive coupling coefficients (locked from validation)
    constexpr double b_expected = 0.137056;
    constexpr double c_expected = 0.972037;
    constexpr double d_expected = 0.0129125;

    for (int i = 0; i < 4; ++i) {
        config_.passive_coeffs[i] = {
            0.0,         // a: no cubic term
            b_expected,  // b: quadratic coefficient 
            c_expected,  // c: linear coefficient
            d_expected   // d: constant offset
        };
    }

    // Joint limits (from URDF)
    for (int i = 0; i < 4; ++i) {
        config_.mcp_limits[i] = { 0.0, 1.774 };
    }
    config_.thumb_rot_limits = { 0.0, 1.774 };
    config_.thumb_flex_limits = { 0.0, 1.774 };

    // Optimized solver parameters for real-time performance
    config_.max_iterations = 30;           // Reduced for real-time
    config_.residual_tolerance = 5e-4;     // Slightly relaxed for speed
    config_.step_tolerance = 1e-6;
    config_.damping_init = 1e-4;
    config_.damping_factor = 2.0;
    config_.line_search_factor = 0.5;
    config_.max_line_search_steps = 6;     // Reduced for speed

    // Weights and tolerances
    config_.thumb_pos_weight = 1.0;
    config_.thumb_rot_weight = 0.1;        // Reduced orientation weight
    config_.finger_weights = { 1.0, 1.0, 1.0, 1.0 };
    config_.plane_tolerance = 0.008;       // 8mm tolerance

    config_.verbose = false; // Disable for real-time performance
}

void ManusHandIKBridge::SetupCoordinateTransform() {
    // Manus Core uses right-handed, Z-up coordinate system (same as our IK)
    // So transformation should be identity, but we set it up for flexibility
    coord_transform_ = Eigen::Matrix3d::Identity();
    position_scale_ = 1.0; // Both systems use meters
}

bool ManusHandIKBridge::Solve(const FingerTargets& manus_targets, JointConfiguration& joint_config) {
    if (!ik_solver_) {
        return false;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    // Convert Manus targets to Hand IK format
    hand_ik::Targets ik_targets = ConvertTargets(manus_targets);

    // Solve IK
    Eigen::VectorXd qa_solution = Eigen::VectorXd::Zero(6);
    hand_ik::SolveReport report;
    bool success = ik_solver_->solve(ik_targets, qa_solution, &report);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto solve_time = std::chrono::duration<double, std::milli>(end_time - start_time);

    // Update performance tracking
    solve_count_++;
    total_solve_time_ms_ += solve_time.count();

    // Fill result structure
    joint_config.valid = success && report.converged;
    joint_config.solve_time_ms = solve_time.count();
    joint_config.iterations = report.iterations;

    if (joint_config.valid) {
        for (int i = 0; i < 6; ++i) {
            joint_config.joint_angles[i] = qa_solution[i];
        }
    }

    return joint_config.valid;
}

Eigen::Vector3d ManusHandIKBridge::ManusToIKPosition(const Eigen::Vector3d& manus_pos) const {
    // Apply coordinate transformation and scaling
    return coord_transform_ * (manus_pos * position_scale_);
}

Eigen::Matrix3d ManusHandIKBridge::ManusToIKOrientation(const Eigen::Matrix3d& manus_rot) const {
    // Apply coordinate transformation to rotation matrix
    return coord_transform_ * manus_rot * coord_transform_.transpose();
}

hand_ik::Targets ManusHandIKBridge::ConvertTargets(const FingerTargets& manus_targets) const {
    hand_ik::Targets ik_targets;

    // Convert finger positions
    for (int i = 0; i < 4; ++i) {
        ik_targets.p_fingers[i] = ManusToIKPosition(manus_targets.finger_positions[i]);
    }

    // Convert thumb position and orientation
    ik_targets.p_thumb = ManusToIKPosition(manus_targets.thumb_position);
    ik_targets.R_thumb = ManusToIKOrientation(manus_targets.thumb_rotation);

    return ik_targets;
}

void ManusHandIKBridge::SetVerbose(bool verbose) {
    config_.verbose = verbose;
    if (ik_solver_) {
        // Note: Would need to update HandIK to support runtime verbose changes
        // For now, this just updates our local config
    }
}

bool ManusHandIKBridge::RunDiagnostics() {
    if (!ik_solver_) {
        std::cerr << "❌ IK solver not initialized" << std::endl;
        return false;
    }

    std::cout << "\n=== Hand IK Bridge Diagnostics ===" << std::endl;

    // Test Jacobian computation
    Eigen::VectorXd qa_test = Eigen::VectorXd::Zero(6);
    bool jacobian_ok = ik_solver_->checkJacobianFiniteDiff(qa_test, 1e-3);
    std::cout << "Jacobian validation: " << (jacobian_ok ? "✓ PASS" : "❌ FAIL") << std::endl;

    // Test reachability
    bool reachability_ok = ik_solver_->testReachability(20, 0.1); // Quick test
    std::cout << "Reachability test: " << (reachability_ok ? "✓ PASS" : "❌ FAIL") << std::endl;

    // Performance statistics
    if (solve_count_ > 0) {
        double avg_solve_time = total_solve_time_ms_ / solve_count_;
        std::cout << "Performance stats:" << std::endl;
        std::cout << "  • Total solves: " << solve_count_ << std::endl;
        std::cout << "  • Average solve time: " << std::fixed << std::setprecision(3)
            << avg_solve_time << " ms" << std::endl;
    }

    std::cout << "===============================" << std::endl;

    return jacobian_ok && reachability_ok;
}