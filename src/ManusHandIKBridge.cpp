// File: src/ManusHandIKBridge.cpp
#include "ManusHandIKBridge.h"
#include "math_constants.hpp"
#include <chrono>
#include <iostream>
#include <filesystem>
#include <iomanip>
#include <random>

ManusHandIKBridge::ManusHandIKBridge(const std::string& urdf_path, const std::string& config_path) {
    try {
        // Load configuration from JSON file
        if (!config_path.empty() && std::filesystem::exists(config_path)) {
            manus_config_ = ManusIntegrationConfig::LoadFromFile(config_path);
        }
        else {
            if (!config_path.empty()) {
                std::cout << "[Bridge] Config file not found, using defaults: " << config_path << std::endl;
            }
            manus_config_ = ManusIntegrationConfig{}; // Use defaults
        }

        // Initialize Hand IK configuration with defaults
        InitializeConfig();

        // Apply loaded config to Hand IK settings
        manus_config_.ApplyToHandIKConfig(config_);

        // Setup coordinate system transformation
        SetupCoordinateTransform();

        // Use config URDF path if provided, otherwise use parameter
        std::string resolved_urdf_path = urdf_path;
        if (!manus_config_.urdf_path.empty() && manus_config_.urdf_path != "surge_v13_hand_right_pybullet.urdf") {
            resolved_urdf_path = manus_config_.urdf_path;
        }

        // Resolve URDF path with fallback search
        if (!std::filesystem::exists(resolved_urdf_path)) {
            std::filesystem::path exe_dir = std::filesystem::current_path();
            std::vector<std::filesystem::path> search_paths = {
                exe_dir / resolved_urdf_path,
                exe_dir / "build" / "Release" / resolved_urdf_path,
                exe_dir / ".." / resolved_urdf_path,
                std::filesystem::path(resolved_urdf_path) // Try as absolute path
            };

            bool found = false;
            for (const auto& search_path : search_paths) {
                if (std::filesystem::exists(search_path)) {
                    resolved_urdf_path = std::filesystem::absolute(search_path).string();
                    std::cout << "[Bridge] Found URDF at: " << resolved_urdf_path << std::endl;
                    found = true;
                    break;
                }
            }

            if (!found) {
                std::string error_msg = "[Bridge] URDF not found. Checked:\n";
                for (const auto& path : search_paths) {
                    error_msg += "  - " + path.string() + "\n";
                }
                error_msg += "Current working directory: " + exe_dir.string();
                std::cerr << error_msg << std::endl;
                throw std::runtime_error("URDF file not found: " + resolved_urdf_path);
            }
        }
        else {
            resolved_urdf_path = std::filesystem::absolute(resolved_urdf_path).string();
            std::cout << "[Bridge] Using provided URDF: " << resolved_urdf_path << std::endl;
        }

        std::cout << "[Bridge] Final URDF path: " << resolved_urdf_path << std::endl;

        // Initialize Hand IK solver with resolved path and applied config
        ik_solver_ = std::make_unique<hand_ik::HandIK>(config_, resolved_urdf_path);

        std::cout << "[Bridge] ManusHandIKBridge initialized successfully with WORKING configuration" << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "[Bridge] Failed to initialize: " << e.what() << std::endl;
        ik_solver_.reset();
    }
}

bool ManusHandIKBridge::Solve(const FingerTargets& manus_targets, JointConfiguration& joint_config) {
    if (!ik_solver_) {
        std::cerr << "[Bridge] IK solver not initialized" << std::endl;
        joint_config.valid = false;
        return false;
    }

    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Convert Manus targets to Hand IK format
        hand_ik::Targets ik_targets = ConvertTargets(manus_targets);

        // Prepare solution vector (6 active joints: 4 MCP + 2 thumb)
        Eigen::VectorXd qa_solution = Eigen::VectorXd::Zero(6);
        hand_ik::SolveReport report;

        // Solve IK
        bool success = ik_solver_->solve(ik_targets, qa_solution, &report);

        // Measure solve time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double solve_time_ms = duration.count() / 1000.0;

        // Fill joint configuration result
        joint_config.valid = success && report.converged;
        joint_config.solve_time_ms = solve_time_ms;
        joint_config.iterations = report.iterations;

        if (joint_config.valid) {
            // Copy joint angles (6 elements: 4 MCP + 2 thumb)
            for (int i = 0; i < 6; ++i) {
                joint_config.joint_angles[i] = qa_solution[i];
            }

            // DEBUG: Check thumb flexion is working
            if (manus_config_.logging.verbose_ik && qa_solution[5] != 0.0) {
                std::cout << "[Bridge] DEBUG: Thumb flexion working: " << qa_solution[5]
                    << " rad (" << (qa_solution[5] * 180.0 / hand_ik::kPi) << "°)" << std::endl;
            }
        }

        // Update performance statistics
        solve_count_++;
        total_solve_time_ms_ += solve_time_ms;

        // Log successful solves (throttled based on config)
        if (manus_config_.logging.performance_stats && joint_config.valid &&
            (solve_count_ % (manus_config_.performance.target_fps * manus_config_.performance.stats_interval_seconds) == 0)) {
            std::cout << "[Bridge] Performance [" << manus_config_.performance.stats_interval_seconds << "s]: "
                << "Solve #" << solve_count_
                << ", Avg: " << std::fixed << std::setprecision(3) << (total_solve_time_ms_ / solve_count_) << "ms"
                << ", Last: " << solve_time_ms << "ms (" << report.iterations << " iter)" << std::endl;
        }

        // Check for performance issues
        if (solve_time_ms > manus_config_.performance.max_solve_time_ms) {
            static int slow_solve_count = 0;
            slow_solve_count++;
            if (slow_solve_count % 10 == 1) { // Throttle warnings
                std::cout << "[Bridge] WARNING: Slow solve " << solve_time_ms << "ms > "
                    << manus_config_.performance.max_solve_time_ms << "ms target" << std::endl;
            }
        }

        return joint_config.valid;

    }
    catch (const std::exception& e) {
        std::cerr << "[Bridge] Solve error: " << e.what() << std::endl;
        joint_config.valid = false;
        return false;
    }
}

Eigen::Vector3d ManusHandIKBridge::ManusToIKPosition(const Eigen::Vector3d& manus_pos) const {
    // Apply coordinate transformation and scaling
    return position_scale_ * (coord_transform_ * manus_pos);
}

Eigen::Matrix3d ManusHandIKBridge::ManusToIKOrientation(const Eigen::Matrix3d& manus_rot) const {
    // Apply coordinate transformation to rotation matrix
    return coord_transform_ * manus_rot * coord_transform_.transpose();
}

void ManusHandIKBridge::SetVerbose(bool verbose) {
    if (ik_solver_) {
        // Access config through the solver and modify it
        const_cast<hand_ik::HandIKConfig&>(ik_solver_->getConfig()).verbose = verbose;
    }
}

bool ManusHandIKBridge::RunDiagnostics() {
    if (!ik_solver_) {
        std::cerr << "[Bridge] Cannot run diagnostics - IK solver not initialized" << std::endl;
        return false;
    }

    std::cout << "[Bridge] Running Hand IK diagnostics with PROPER analytical Jacobian..." << std::endl;

    try {
        // Test 1: Jacobian validation
        Eigen::VectorXd qa_test = Eigen::VectorXd::Zero(6);
        qa_test[0] = 0.5;  // Index finger
        qa_test[4] = 0.3;  // Thumb rotation
        qa_test[5] = 0.4;  // Thumb flexion

        bool jacobian_ok = ik_solver_->checkJacobianFiniteDiff(qa_test, 1e-3);
        std::cout << "[Bridge] Jacobian test: " << (jacobian_ok ? "PASS" : "FAIL") << std::endl;

        // Test 2: Reachability test
        bool reachability_ok = ik_solver_->testReachability(20, 0.1); // 20 tests, 0.1 noise
        std::cout << "[Bridge] Reachability test: " << (reachability_ok ? "PASS" : "FAIL") << std::endl;

        // Test 3: Bridge-specific coordinate conversion test with WORKING targets
        FingerTargets working_targets;

        // Use targets from the working range (5-9cm from origin)
        working_targets.finger_positions[0] = Eigen::Vector3d(0.06, 0.01, 0.04);   // Index
        working_targets.finger_positions[1] = Eigen::Vector3d(0.065, 0.005, 0.045); // Middle  
        working_targets.finger_positions[2] = Eigen::Vector3d(0.06, -0.01, 0.04);   // Ring
        working_targets.finger_positions[3] = Eigen::Vector3d(0.055, -0.02, 0.035); // Pinky
        working_targets.thumb_position = Eigen::Vector3d(0.04, 0.04, 0.04);
        working_targets.thumb_rotation = Eigen::Matrix3d::Identity();

        JointConfiguration joint_config;
        bool solve_ok = Solve(working_targets, joint_config);
        std::cout << "[Bridge] Working targets test: " << (solve_ok ? "PASS" : "FAIL");
        if (solve_ok) {
            std::cout << " (" << joint_config.solve_time_ms << "ms, "
                << joint_config.iterations << " iterations)";
        }
        std::cout << std::endl;

        bool all_tests_passed = jacobian_ok && reachability_ok && solve_ok;

        std::cout << "[Bridge] Overall diagnostics: " << (all_tests_passed ? "PASS" : "FAIL") << std::endl;
        std::cout << "[Bridge] Performance stats: " << solve_count_ << " solves, "
            << "avg time: " << (solve_count_ > 0 ? total_solve_time_ms_ / solve_count_ : 0.0)
            << "ms" << std::endl;

        // Print configuration summary
        std::cout << "[Bridge] Configuration summary:" << std::endl;
        std::cout << "  Max iterations: " << manus_config_.solver_params.max_iterations << std::endl;
        std::cout << "  Residual tolerance: " << manus_config_.solver_params.residual_tolerance << std::endl;
        std::cout << "  Target FPS: " << manus_config_.performance.target_fps << std::endl;
        std::cout << "  Performance stats: " << (manus_config_.logging.performance_stats ? "enabled" : "disabled") << std::endl;

        return all_tests_passed;

    }
    catch (const std::exception& e) {
        std::cerr << "[Bridge] Diagnostics error: " << e.what() << std::endl;
        return false;
    }
}

void ManusHandIKBridge::InitializeConfig() {
    // CRITICAL FIX: Use EXACT configuration from working test suite
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

    // Validated passive coupling coefficients (LOCKED from test suite)
    constexpr double a = 0.0;          // cubic coefficient (0.0)
    constexpr double b = 0.137056;     // quadratic coefficient 
    constexpr double c = 0.972037;     // linear coefficient
    constexpr double d = 0.0129125;    // constant offset

    for (int i = 0; i < 4; ++i) {
        config_.passive_coeffs[i] = { a, b, c, d };
    }

    // CRITICAL FIX: Use WORKING joint limits from successful tests (NOT conservative!)
    std::cout << "[Bridge] Using WORKING joint limits from successful test suite" << std::endl;
    for (int i = 0; i < 4; ++i) {
        config_.mcp_limits[i] = { 0.0, 1.774 };  // Full range as in working tests
    }
    config_.thumb_rot_limits = { 0.0, 1.774 };
    config_.thumb_flex_limits = { 0.0, 1.774 };

    std::cout << "[Bridge] Applied working joint limits: [0.0, 1.774] rad (101.6 degrees)" << std::endl;

    // CRITICAL FIX: Use WORKING solver parameters from successful tests
    config_.max_iterations = 100;        // Same as working tests
    config_.residual_tolerance = 1e-6;   // Same as working tests  
    config_.step_tolerance = 1e-8;       // Same as working tests
    config_.damping_init = 1e-3;         // Same as working tests
    config_.damping_factor = 10.0;       // Same as working tests
    config_.line_search_factor = 0.8;    // Same as working tests
    config_.max_line_search_steps = 10;  // Same as working tests
    config_.verbose = false;

    // CRITICAL FIX: Use WORKING weights from successful tests
    config_.thumb_pos_weight = 1.0;      // Same as working tests
    config_.thumb_rot_weight = 0.2;      // Same as working tests
    config_.plane_tolerance = 0.005;     // Same as working tests (5mm)

    for (int i = 0; i < 4; ++i) {
        config_.finger_weights[i] = 1.0; // Same as working tests
    }

    std::cout << "[Bridge] Applied WORKING solver configuration from successful test suite:" << std::endl;
    std::cout << "  Max iterations: " << config_.max_iterations << std::endl;
    std::cout << "  Residual tolerance: " << config_.residual_tolerance << std::endl;
    std::cout << "  Damping init: " << config_.damping_init << std::endl;
    std::cout << "  Plane tolerance: " << config_.plane_tolerance << " m" << std::endl;
}

void ManusHandIKBridge::SetupCoordinateTransform() {
    // Setup coordinate system transformation between Manus and Hand IK
    // Based on the config coordinate system settings

    if (manus_config_.logging.coordinate_debug) {
        std::cout << "[Bridge] Coordinate system config:" << std::endl;
        std::cout << "  Handedness: " << manus_config_.coordinate_system.handedness << std::endl;
        std::cout << "  Up axis: " << manus_config_.coordinate_system.up_axis << std::endl;
        std::cout << "  View axis: " << manus_config_.coordinate_system.view_axis << std::endl;
        std::cout << "  Units: " << manus_config_.coordinate_system.units << std::endl;
    }

    // For now, use identity transformation (no coordinate conversion)
    // This assumes both Manus and Hand IK use compatible coordinate systems
    coord_transform_ = Eigen::Matrix3d::Identity();

    // Set position scale based on units
    if (manus_config_.coordinate_system.units == "meters") {
        position_scale_ = 1.0;
    }
    else if (manus_config_.coordinate_system.units == "centimeters") {
        position_scale_ = 0.01;
    }
    else if (manus_config_.coordinate_system.units == "millimeters") {
        position_scale_ = 0.001;
    }
    else {
        position_scale_ = 1.0;
        std::cout << "[Bridge] Unknown units, using meters" << std::endl;
    }

    std::cout << "[Bridge] Coordinate transform: Identity (no conversion)" << std::endl;
    std::cout << "[Bridge] Position scale: " << position_scale_ << " (" << manus_config_.coordinate_system.units << ")" << std::endl;
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