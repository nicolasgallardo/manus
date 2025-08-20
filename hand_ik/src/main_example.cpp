#include "hand_ik.hpp"
#include "math_constants.hpp"
#include <iostream>
#include <fstream>
#include <random>
#include <filesystem>
#include <cstdlib>

using namespace hand_ik;

// Simple YAML-like config loader (basic implementation)
HandIKConfig loadExampleConfig() {
    HandIKConfig config;

    // Joint names (updated to match actual URDF)
    config.mcp_joint_names = {
        "Index_MCP_Joint",
        "Middle_MCP_Joint",
        "Ring_MCP_Joint",
        "Pinky_MCP_Joint"
    };

    config.distal_joint_names = {
        "Index_PIP_Joint",      // PIP joints act as the "distal" passive joints
        "Middle_PIP_Joint",
        "Ring_PIP_Joint",
        "Pinky_PIP_Joint"
    };

    config.fingertip_frame_names = {
        "Index_Distal",         // Using distal links as tip frames
        "Middle_Distal",
        "Ring_Distal",
        "Pinky_Distal"
    };

    config.thumb_rot_joint_name = "Metacarpal_Joint";  // Thumb metacarpal rotation
    config.thumb_flex_joint_name = "Thumb_Joint";      // Thumb flexion
    config.thumb_tip_frame_name = "Thumb";

    // Measured passive coupling coefficients from actual hand data
    // Based on Excel analysis: y = -7E-05x⁴ + 0.0124x³ + 0.622x + 3.0025
    // Simplified to cubic: q_distal = a*q_mcp^3 + b*q_mcp^2 + c*q_mcp + d
    // Note: Excel data appears to be in degrees, need to convert to radians
    for (int i = 0; i < 4; ++i) {
        config.passive_coeffs[i] = {
            0.0124 * degToRad(1.0),     // a: cubic coefficient 
            0.0,                        // b: quadratic (set to 0 for simplification)
            0.622 * degToRad(1.0),      // c: linear coefficient (dominant)
            3.0025 * degToRad(1.0)      // d: constant offset
        };
    }

    // Joint limits in radians (updated based on URDF limits)
    // URDF shows limits in radians: lower="0" upper="1.774" for most joints
    for (int i = 0; i < 4; ++i) {
        config.mcp_limits[i] = { 0.0, 1.774 }; // MCP joint limits from URDF
    }
    config.thumb_rot_limits = { 0.0, 1.774 };   // Metacarpal joint limits
    config.thumb_flex_limits = { 0.0, 1.774 };  // Thumb joint limits

    // Solver parameters
    config.max_iterations = 100;
    config.residual_tolerance = 1e-6;
    config.step_tolerance = 1e-8;
    config.damping_init = 1e-6;
    config.verbose = true;

    return config;
}

void generateRandomReachableTargets(HandIK& ik, Targets& targets) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5, 0.5);

    // Generate random active configuration
    Eigen::VectorXd qa_random(6);
    for (int i = 0; i < 6; ++i) {
        qa_random[i] = dis(gen);
    }

    // Compute forward kinematics for this configuration
    ik.computeForwardKinematics(qa_random);

    // Extract fingertip positions as targets
    for (int i = 0; i < 4; ++i) {
        targets.p_fingers[i] = ik.getFingerTip(i);
    }
    targets.p_thumb = ik.getThumbTip();
    targets.R_thumb = ik.getThumbOrientation();

    std::cout << "Generated reachable targets:\n";
    for (int i = 0; i < 4; ++i) {
        std::cout << "  Finger " << i << ": " << targets.p_fingers[i].transpose() << "\n";
    }
    std::cout << "  Thumb pos: " << targets.p_thumb.transpose() << "\n";
}

int main(int argc, char** argv) {
    try {
        std::cout << "=== Hand IK Example ===" << std::endl;

        // Resolve URDF path using new resolution system
        std::string urdf_path = resolveUrdfPath(argc, argv);
        std::cout << "Using URDF: " << urdf_path << std::endl;

        // Load configuration
        HandIKConfig config = loadExampleConfig();

        // Initialize IK solver
        HandIK ik(config, urdf_path);

        std::cout << "\n=== Running Jacobian Check ===" << std::endl;
        Eigen::VectorXd qa_test = Eigen::VectorXd::Zero(6);
        bool jacobian_ok = ik.checkJacobianFiniteDiff(qa_test, 1e-5);
        std::cout << "Jacobian check: " << (jacobian_ok ? "PASS" : "FAIL") << std::endl;

        std::cout << "\n=== Running Reachability Test ===" << std::endl;
        bool reachability_ok = ik.testReachability(50, 0.1);
        std::cout << "Reachability test: " << (reachability_ok ? "PASS" : "FAIL") << std::endl;

        std::cout << "\n=== Solving IK for Random Targets ===" << std::endl;

        // Generate random reachable targets
        Targets targets;
        generateRandomReachableTargets(ik, targets);

        // Perturb initial guess
        Eigen::VectorXd qa_init = Eigen::VectorXd::Random(6) * 0.2;

        // Solve IK
        Eigen::VectorXd qa_solution = qa_init;
        SolveReport report;

        std::cout << "\nSolving IK..." << std::endl;
        bool success = ik.solve(targets, qa_solution, &report);

        std::cout << "\n=== Results ===" << std::endl;
        std::cout << "Success: " << (success ? "YES" : "NO") << std::endl;
        std::cout << "Iterations: " << report.iterations << std::endl;
        std::cout << "Initial error: " << report.initial_error << std::endl;
        std::cout << "Final error: " << report.final_error << std::endl;
        std::cout << "Converged: " << (report.converged ? "YES" : "NO") << std::endl;

        std::cout << "\nSolved active joint angles (radians):" << std::endl;
        std::cout << "  MCP joints: ";
        for (int i = 0; i < 4; ++i) {
            std::cout << qa_solution[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "  Thumb rotation: " << qa_solution[4] << std::endl;
        std::cout << "  Thumb flexion: " << qa_solution[5] << std::endl;

        // Check out-of-plane flags
        bool any_out_of_plane = false;
        for (int i = 0; i < 4; ++i) {
            if (report.out_of_plane_flags[i]) {
                std::cout << "Warning: Finger " << i << " target is out of flexion plane by "
                    << report.out_of_plane_distances[i] << " m" << std::endl;
                any_out_of_plane = true;
            }
        }
        if (!any_out_of_plane) {
            std::cout << "All targets are within flexion planes." << std::endl;
        }

        // Verify solution by computing forward kinematics
        ik.computeForwardKinematics(qa_solution);
        std::cout << "\n=== Forward Kinematics Verification ===" << std::endl;
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d achieved = ik.getFingerTip(i);
            Eigen::Vector3d error = achieved - targets.p_fingers[i];
            std::cout << "Finger " << i << " error: " << error.norm() << " m" << std::endl;
        }

        Eigen::Vector3d thumb_achieved = ik.getThumbTip();
        Eigen::Vector3d thumb_error = thumb_achieved - targets.p_thumb;
        std::cout << "Thumb position error: " << thumb_error.norm() << " m" << std::endl;

        if (targets.R_thumb.has_value()) {
            Eigen::Matrix3d R_achieved = ik.getThumbOrientation();
            Eigen::Matrix3d R_error = targets.R_thumb.value() * R_achieved.transpose();
            Eigen::Vector3d rot_error = pinocchio::log3(R_error);
            std::cout << "Thumb orientation error: " << rot_error.norm() << " rad" << std::endl;
        }

        std::cout << "\n=== Test Complete ===" << std::endl;

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}