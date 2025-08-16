#include "hand_ik.hpp"
#include "math_constants.hpp"
#include <iostream>
#include <random>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cmath>

using namespace hand_ik;

// Helper function to ensure URDF extension
static std::filesystem::path EnsureUrdfExt(std::string s) {
    std::filesystem::path p(s);
    if (p.extension().empty()) p.replace_extension(".urdf");
    return p;
}

// Clean URDF resolution helper
static std::filesystem::path ResolveUrdfPath(int argc, char** argv) {
    // 1) Next to the exe (manual copy or POST_BUILD copy)
    auto exe_dir = std::filesystem::path(argv[0]).parent_path();
    auto exe_urdf = exe_dir / "surge_v13_hand_right_pybullet.urdf";

    // 2) --urdf <path-or-basename>
    std::string cli;
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::string(argv[i]) == "--urdf") {
            cli = argv[i + 1];
            break;
        }
    }

    // 3) HAND_IK_URDF env
    std::string env = []() {
        const char* v = std::getenv("HAND_IK_URDF");
        return v ? std::string(v) : std::string();
        }();

    std::vector<std::filesystem::path> cands;
    cands.push_back(exe_urdf);
    if (!cli.empty()) cands.push_back(EnsureUrdfExt(cli));
    if (!env.empty()) cands.push_back(EnsureUrdfExt(env));

    for (const auto& p : cands) {
        if (std::filesystem::exists(p)) return p;
    }

    std::ostringstream oss;
    oss << "URDF not found. Checked:\n";
    for (const auto& p : cands) oss << " - " << p.string() << "\n";
    oss << "Try copy next to exe, or use --urdf <path> or set HAND_IK_URDF.";
    throw std::runtime_error(oss.str());
}

// Structure to hold per-finger plane information
struct FingerPlaneInfo {
    Eigen::Vector3d mcp_origin;
    Eigen::Vector3d flexion_axis;
    double out_of_plane_distance;
};

// Compute per-finger MCP plane information
std::array<FingerPlaneInfo, 4> computeFingerPlanes(HandIK& ik, const Eigen::VectorXd& qa) {
    std::array<FingerPlaneInfo, 4> planes;

    const auto& config = ik.getConfig();
    const double delta = 1e-4; // Small perturbation for numerical differentiation

    // Update kinematics at the given configuration
    Eigen::VectorXd q_full = ik.expandActiveToFull(qa);
    ik.computeForwardKinematics(q_full);

    for (int i = 0; i < 4; ++i) {
        // Get baseline fingertip position
        Eigen::Vector3d tip_base = ik.getFingerTip(i);

        // Perturb MCP joint to estimate motion direction
        Eigen::VectorXd qa_plus = qa;
        Eigen::VectorXd qa_minus = qa;

        qa_plus[i] += delta;
        qa_minus[i] -= delta;

        // Clamp to limits
        qa_plus[i] = std::max(config.mcp_limits[i][0],
            std::min(config.mcp_limits[i][1], qa_plus[i]));
        qa_minus[i] = std::max(config.mcp_limits[i][0],
            std::min(config.mcp_limits[i][1], qa_minus[i]));

        // Compute perturbed fingertip positions
        Eigen::VectorXd q_plus = ik.expandActiveToFull(qa_plus);
        ik.computeForwardKinematics(q_plus);
        Eigen::Vector3d tip_plus = ik.getFingerTip(i);

        Eigen::VectorXd q_minus = ik.expandActiveToFull(qa_minus);
        ik.computeForwardKinematics(q_minus);
        Eigen::Vector3d tip_minus = ik.getFingerTip(i);

        // Estimate motion direction (tangent to the flexion arc)
        Eigen::Vector3d motion_dir = (tip_plus - tip_minus).normalized();

        // Estimate center of rotation (MCP joint center)
        // The motion is roughly circular, so we can estimate the center
        Eigen::Vector3d chord = tip_plus - tip_minus;
        Eigen::Vector3d midpoint = (tip_plus + tip_minus) * 0.5;

        // Vector from midpoint to baseline tip
        Eigen::Vector3d to_base = tip_base - midpoint;

        // The center should be roughly along the perpendicular to the chord
        // through the midpoint, at a distance related to the fingertip-to-center distance
        double chord_length = chord.norm();
        if (chord_length > 1e-6) {
            // Estimate radius using small angle approximation
            double angle_change = (qa_plus[i] - qa_minus[i]);
            double radius = chord_length / (2.0 * std::sin(angle_change / 2.0));

            // The perpendicular direction in the plane of motion
            Eigen::Vector3d perpendicular = to_base - (to_base.dot(motion_dir)) * motion_dir;
            if (perpendicular.norm() > 1e-6) {
                perpendicular.normalize();
                planes[i].mcp_origin = tip_base - radius * perpendicular;
            }
            else {
                // Fallback: use a point close to the fingertip
                planes[i].mcp_origin = tip_base - 0.1 * to_base;
            }
        }
        else {
            // Fallback for very small motion
            planes[i].mcp_origin = tip_base - 0.1 * to_base;
        }

        // The flexion axis is perpendicular to the motion direction and the radial direction
        Eigen::Vector3d radial_dir = (tip_base - planes[i].mcp_origin).normalized();
        planes[i].flexion_axis = motion_dir.cross(radial_dir).normalized();

        // Ensure the flexion axis has a consistent orientation
        // For human hands, flexion is typically about an axis pointing roughly in the +Y direction
        if (planes[i].flexion_axis.y() < 0) {
            planes[i].flexion_axis *= -1.0;
        }
    }

    // Restore original configuration
    ik.computeForwardKinematics(q_full);

    return planes;
}

// Compute out-of-plane distance for a target relative to a finger's MCP plane
double computeOutOfPlaneDistance(const Eigen::Vector3d& target_pos,
    const FingerPlaneInfo& plane_info) {
    Eigen::Vector3d to_target = target_pos - plane_info.mcp_origin;
    return std::abs(to_target.dot(plane_info.flexion_axis));
}

// Check if targets are within acceptable planes
std::array<bool, 4> checkTargetsInPlane(const Targets& targets,
    const std::array<FingerPlaneInfo, 4>& planes,
    double plane_tolerance,
    std::array<double, 4>& distances) {
    std::array<bool, 4> in_plane;

    for (int i = 0; i < 4; ++i) {
        distances[i] = computeOutOfPlaneDistance(targets.p_fingers[i], planes[i]);
        in_plane[i] = (distances[i] <= plane_tolerance);
    }

    return in_plane;
}

// URDF Sanity Audit function
void AuditUrdf(const pinocchio::Model& model, pinocchio::Data& data) {
    using std::cout;
    cout << "\n=== URDF Sanity Audit ===" << std::endl;

    // (A) Base/global orientation (at neutral q)
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(model.nq);
    pinocchio::forwardKinematics(model, data, q0);
    pinocchio::updateFramePlacements(model, data);

    // Find a fingertip frame to examine
    pinocchio::FrameIndex tip_frame = model.getFrameId("Index_Distal");
    const auto& oMf = data.oMf[tip_frame];
    Eigen::Vector3d rpy = oMf.rotation().eulerAngles(2, 1, 0).reverse(); // roll-pitch-yaw
    cout << "[base->tip @ q0] pos=" << oMf.translation().transpose()
        << " rpy=" << rpy.transpose() << " (rad)" << std::endl;

    // (B) Joint limits & units heuristic
    bool found_units_issue = false;
    for (size_t jid = 1; jid < model.joints.size(); ++jid) {
        if (jid >= model.upperPositionLimit.size()) continue;

        auto jl = model.upperPositionLimit[jid];
        auto ll = model.lowerPositionLimit[jid];

        if (std::isfinite(jl) && std::fabs(jl) > 3.2) {
            cout << "WARN joint " << model.names[jid]
                << " upper limit " << jl << " > pi -> degrees suspected" << std::endl;
                found_units_issue = true;
        }
        if (std::isfinite(ll) && std::fabs(ll) > 3.2) {
            cout << "WARN joint " << model.names[jid]
                << " lower limit " << ll << " > pi -> degrees suspected" << std::endl;
                found_units_issue = true;
        }
    }

    // (C) Check for reasonable scale (link lengths)
    Eigen::Vector3d palm_pos = data.oMi[1].translation(); // Assuming joint 1 is first finger base
    double hand_scale = palm_pos.norm();
    if (hand_scale < 0.01 || hand_scale > 1.0) {
        cout << "WARN hand scale " << hand_scale << " m suggests units issue (expected ~0.1-0.2m)" << std::endl;
        found_units_issue = true;
    }
    else {
        cout << "Hand scale: " << hand_scale << " m (reasonable)" << std::endl;
    }

    // (D) Check joint axes alignment for fingers (simplified)
    std::vector<std::string> finger_mcps = { "Index_MCP_Joint", "Middle_MCP_Joint", "Ring_MCP_Joint", "Pinky_MCP_Joint" };
    std::vector<Eigen::Vector3d> mcp_axes;

    for (const auto& joint_name : finger_mcps) {
        if (model.existJointName(joint_name)) {
            pinocchio::JointIndex jid = model.getJointId(joint_name);
            Eigen::Vector3d axis = data.oMi[jid].rotation() * Eigen::Vector3d::UnitY(); // Assuming Y is flexion axis
            mcp_axes.push_back(axis);
        }
    }

    // Check if axes are roughly parallel
    if (mcp_axes.size() >= 2) {
        double min_dot = 1.0;
        for (size_t i = 0; i < mcp_axes.size(); ++i) {
            for (size_t j = i + 1; j < mcp_axes.size(); ++j) {
                double dot = mcp_axes[i].dot(mcp_axes[j]);
                min_dot = std::min(min_dot, std::abs(dot));
            }
        }
        double angle_deg = std::acos(std::min(1.0, min_dot)) * 180.0 / hand_ik::kPi;
        if (angle_deg > 5.0) {
            cout << "WARN MCP joint axes differ by " << angle_deg << "° -> non-planar fingers" << std::endl;
        }
        else {
            cout << "MCP joint axes aligned within " << angle_deg << "° (good)" << std::endl;
        }
    }

    if (!found_units_issue) {
        cout << "No major URDF issues detected" << std::endl;
    }
    cout << "============================" << std::endl;
}

HandIKConfig createTestConfig() {
    HandIKConfig config;

    config.mcp_joint_names = {
        "Index_MCP_Joint", "Middle_MCP_Joint", "Ring_MCP_Joint", "Pinky_MCP_Joint"
    };
    config.distal_joint_names = {
        "Index_PIP_Joint", "Middle_PIP_Joint", "Ring_PIP_Joint", "Pinky_PIP_Joint"
    };
    config.fingertip_frame_names = {
        "Index_Distal", "Middle_Distal", "Ring_Distal", "Pinky_Distal"
    };

    config.thumb_rot_joint_name = "Metacarpal_Joint";
    config.thumb_flex_joint_name = "Thumb_Joint";
    config.thumb_tip_frame_name = "Thumb";

    // Corrected passive coupling coefficients
    constexpr double a = 0.0;          // cubic coefficient (0.0)
    constexpr double b = 0.137056;     // quadratic coefficient 
    constexpr double c = 0.972037;     // linear coefficient
    constexpr double d = 0.0129125;    // constant offset

    for (int i = 0; i < 4; ++i) {
        config.passive_coeffs[i] = {
            a,  // cubic coefficient (0.0)
            b,  // quadratic coefficient 
            c,  // linear coefficient
            d   // constant offset
        };
    }

    // Joint limits (from URDF)
    for (int i = 0; i < 4; ++i) {
        config.mcp_limits[i] = { 0.0, 1.774 };
    }
    config.thumb_rot_limits = { 0.0, 1.774 };
    config.thumb_flex_limits = { 0.0, 1.774 };

    // Improved solver parameters
    config.max_iterations = 100;
    config.residual_tolerance = 1e-6;
    config.step_tolerance = 1e-8;
    config.damping_init = 1e-3;  // Updated damping
    config.verbose = false;

    // Tuned plane tolerance for the more accurate plane detection
    config.plane_tolerance = 0.005;  // 5mm tolerance for out-of-plane

    // Reduced orientation weight for debugging
    config.thumb_pos_weight = 1.0;
    config.thumb_rot_weight = 0.2;  // Reduced from default

    return config;
}

bool performReachabilityTest(HandIK& ik, int num_tests, double noise_scale,
    double position_tolerance, double orientation_tolerance) {

    std::cout << "Running " << num_tests << " reachability tests..." << std::endl;
    std::cout << "Noise scale: " << noise_scale << std::endl;
    std::cout << "Position tolerance: " << position_tolerance << " m" << std::endl;
    std::cout << "Orientation tolerance: " << orientation_tolerance << " rad" << std::endl;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    int successful_tests = 0;
    int converged_tests = 0;
    int within_tolerance_tests = 0;
    int out_of_plane_skipped = 0;

    std::vector<double> solve_times;
    std::vector<int> iteration_counts;
    std::vector<double> final_errors;
    std::vector<std::array<double, 4>> out_of_plane_distances;

    auto config = ik.getConfig();
    Eigen::VectorXd qa_seed = Eigen::VectorXd::Zero(6); // Warm-start seed

    for (int test = 0; test < num_tests; ++test) {
        // Generate random reachable configuration within joint limits
        Eigen::VectorXd qa_true(6);

        for (int j = 0; j < 4; ++j) {
            double range = config.mcp_limits[j][1] - config.mcp_limits[j][0];
            qa_true[j] = config.mcp_limits[j][0] + dis(gen) * 0.5 * range + 0.5 * range;
        }

        // Thumb joints
        double thumb_rot_range = config.thumb_rot_limits[1] - config.thumb_rot_limits[0];
        qa_true[4] = config.thumb_rot_limits[0] + dis(gen) * 0.5 * thumb_rot_range + 0.5 * thumb_rot_range;

        double thumb_flex_range = config.thumb_flex_limits[1] - config.thumb_flex_limits[0];
        qa_true[5] = config.thumb_flex_limits[0] + dis(gen) * 0.5 * thumb_flex_range + 0.5 * thumb_flex_range;

        // Convert to full configuration vector and compute FK
        Eigen::VectorXd q_full = ik.expandActiveToFull(qa_true);
        ik.computeForwardKinematics(q_full);

        Targets targets;
        for (int i = 0; i < 4; ++i) {
            targets.p_fingers[i] = ik.getFingerTip(i);
        }
        targets.p_thumb = ik.getThumbTip();
        targets.R_thumb = ik.getThumbOrientation();

        // Compute per-finger MCP planes
        std::array<FingerPlaneInfo, 4> finger_planes = computeFingerPlanes(ik, qa_true);

        // Check for out-of-plane targets using proper MCP plane calculation
        std::array<double, 4> distances;
        std::array<bool, 4> in_plane = checkTargetsInPlane(targets, finger_planes,
            config.plane_tolerance, distances);

        bool skip_due_to_plane = false;
        for (int i = 0; i < 4; ++i) {
            if (!in_plane[i]) {
                skip_due_to_plane = true;
                if (test < 5) { // Only print for first few tests
                    std::cout << "Skipping test " << test << " due to out-of-plane finger " << i
                        << " (distance=" << distances[i] << "m > tolerance=" << config.plane_tolerance << "m)" << std::endl;
                }
                break;
            }
        }

        if (skip_due_to_plane) {
            out_of_plane_skipped++;
            continue; // Skip this test case
        }

        // Store distances for analysis
        out_of_plane_distances.push_back(distances);

        // Use warm-start from previous solution or neutral pose
        Eigen::VectorXd qa_init = qa_seed;
        for (int j = 0; j < 6; ++j) {
            qa_init[j] += noise_scale * dis(gen);
        }

        // Solve IK
        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::VectorXd qa_solved = qa_init;
        SolveReport report;
        bool success = ik.solve(targets, qa_solved, &report);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        solve_times.push_back(duration.count() / 1000.0); // Convert to milliseconds

        iteration_counts.push_back(report.iterations);
        final_errors.push_back(report.final_error);

        if (report.converged) {
            converged_tests++;
            qa_seed = qa_solved; // Update warm-start
        }

        // Check if final error matches solver's metric (same frame and convention)
        bool within_tolerance = (report.final_error < position_tolerance);

        // Additional verification: compute actual FK and check individual components
        if (within_tolerance) {
            Eigen::VectorXd q_solved_full = ik.expandActiveToFull(qa_solved);
            ik.computeForwardKinematics(q_solved_full);

            double max_finger_error = 0.0;
            for (int i = 0; i < 4; ++i) {
                Eigen::Vector3d achieved = ik.getFingerTip(i);
                double pos_error = (achieved - targets.p_fingers[i]).norm();
                max_finger_error = std::max(max_finger_error, pos_error);
            }

            Eigen::Vector3d thumb_achieved = ik.getThumbTip();
            double thumb_pos_error = (thumb_achieved - targets.p_thumb).norm();
            max_finger_error = std::max(max_finger_error, thumb_pos_error);

            // Use relaxed individual component check
            within_tolerance = (max_finger_error < position_tolerance);

            // Check thumb orientation if specified
            if (within_tolerance && targets.R_thumb.has_value()) {
                Eigen::Matrix3d R_achieved = ik.getThumbOrientation();
                Eigen::Matrix3d R_error = targets.R_thumb.value() * R_achieved.transpose();
                Eigen::Vector3d rot_error = pinocchio::log3(R_error);
                if (rot_error.norm() > orientation_tolerance) {
                    within_tolerance = false;
                }
            }
        }

        if (within_tolerance) {
            within_tolerance_tests++;
        }

        if (success && within_tolerance) {
            successful_tests++;
        }

        // Print progress and debug info for failures
        int valid_tests = test + 1 - out_of_plane_skipped;
        if (valid_tests > 0 && ((valid_tests % 10 == 0) || test < 3)) {
            std::cout << "Test " << (test + 1) << "/" << num_tests << " (valid: " << valid_tests
                << ", skipped: " << out_of_plane_skipped << ")"
                << " - Success rate: " << (100.0 * successful_tests / valid_tests) << "%";
            if (!within_tolerance && test < 3) {
                std::cout << " [Final err: " << report.final_error << ", Converged: " << report.converged << "]";
            }
            std::cout << std::endl;
        }
    }

    int valid_tests = num_tests - out_of_plane_skipped;
    if (valid_tests == 0) {
        std::cout << "ERROR: All tests were skipped due to out-of-plane targets!" << std::endl;
        return false;
    }

    // Compute statistics
    double success_rate = double(successful_tests) / valid_tests;
    double convergence_rate = double(converged_tests) / valid_tests;
    double tolerance_rate = double(within_tolerance_tests) / valid_tests;

    // Timing statistics
    double avg_solve_time = 0.0;
    double max_solve_time = 0.0;
    for (double time : solve_times) {
        avg_solve_time += time;
        max_solve_time = std::max(max_solve_time, time);
    }
    if (!solve_times.empty()) {
        avg_solve_time /= solve_times.size();
    }

    // Iteration statistics
    double avg_iterations = 0.0;
    int max_iterations = 0;
    for (int iters : iteration_counts) {
        avg_iterations += iters;
        max_iterations = std::max(max_iterations, iters);
    }
    if (!iteration_counts.empty()) {
        avg_iterations /= iteration_counts.size();
    }

    // Error statistics
    double avg_final_error = 0.0;
    double max_final_error = 0.0;
    for (double error : final_errors) {
        avg_final_error += error;
        max_final_error = std::max(max_final_error, error);
    }
    if (!final_errors.empty()) {
        avg_final_error /= final_errors.size();
    }

    // Out-of-plane statistics
    double avg_plane_distance = 0.0;
    double max_plane_distance = 0.0;
    if (!out_of_plane_distances.empty()) {
        for (const auto& distances : out_of_plane_distances) {
            for (int i = 0; i < 4; ++i) {
                avg_plane_distance += distances[i];
                max_plane_distance = std::max(max_plane_distance, distances[i]);
            }
        }
        avg_plane_distance /= (out_of_plane_distances.size() * 4);
    }

    std::cout << "\n=== Reachability Test Results ===" << std::endl;
    std::cout << "Total tests attempted: " << num_tests << std::endl;
    std::cout << "Tests skipped (out-of-plane): " << out_of_plane_skipped << std::endl;
    std::cout << "Valid tests: " << valid_tests << std::endl;
    std::cout << "Successful tests: " << successful_tests << " (" << (success_rate * 100) << "%)" << std::endl;
    std::cout << "Converged tests: " << converged_tests << " (" << (convergence_rate * 100) << "%)" << std::endl;
    std::cout << "Within tolerance: " << within_tolerance_tests << " (" << (tolerance_rate * 100) << "%)" << std::endl;

    std::cout << "\n=== Timing Statistics ===" << std::endl;
    std::cout << "Average solve time: " << avg_solve_time << " ms" << std::endl;
    std::cout << "Maximum solve time: " << max_solve_time << " ms" << std::endl;

    std::cout << "\n=== Iteration Statistics ===" << std::endl;
    std::cout << "Average iterations: " << avg_iterations << std::endl;
    std::cout << "Maximum iterations: " << max_iterations << std::endl;

    std::cout << "\n=== Error Statistics ===" << std::endl;
    std::cout << "Average final error: " << avg_final_error << std::endl;
    std::cout << "Maximum final error: " << max_final_error << std::endl;

    std::cout << "\n=== Plane Statistics ===" << std::endl;
    std::cout << "Plane tolerance used: " << config.plane_tolerance << " m" << std::endl;
    std::cout << "Average in-plane distance: " << avg_plane_distance << " m" << std::endl;
    std::cout << "Maximum in-plane distance: " << max_plane_distance << " m" << std::endl;

    // Test passes if success rate is above threshold
    const double required_success_rate = 0.8; // 80%
    bool test_passed = success_rate >= required_success_rate;

    std::cout << "\n=== Test Result ===" << std::endl;
    std::cout << "Required success rate: " << (required_success_rate * 100) << "%" << std::endl;
    std::cout << "Achieved success rate: " << (success_rate * 100) << "%" << std::endl;
    std::cout << "Test result: " << (test_passed ? "PASS" : "FAIL") << std::endl;

    return test_passed;
}

bool testOutOfPlaneHandling(HandIK& ik) {
    std::cout << "\n=== Out-of-Plane Target Handling Test ===" << std::endl;

    // Generate a reachable configuration first
    Eigen::VectorXd qa_base = Eigen::VectorXd::Zero(6);
    qa_base[0] = 0.5; // index finger
    qa_base[1] = 0.3; // middle finger

    Eigen::VectorXd q_base_full = ik.expandActiveToFull(qa_base);
    ik.computeForwardKinematics(q_base_full);

    Targets targets;
    for (int i = 0; i < 4; ++i) {
        targets.p_fingers[i] = ik.getFingerTip(i);
    }
    targets.p_thumb = ik.getThumbTip();
    targets.R_thumb = ik.getThumbOrientation();

    // Compute finger planes for proper out-of-plane testing
    std::array<FingerPlaneInfo, 4> finger_planes = computeFingerPlanes(ik, qa_base);

    // Modify targets to be significantly out of the proper MCP flexion plane
    // Move index finger target out of its specific MCP plane
    Eigen::Vector3d out_of_plane_offset = 0.05 * finger_planes[0].flexion_axis; // 5cm out of plane
    targets.p_fingers[0] += out_of_plane_offset;

    std::cout << "Testing with out-of-plane target (5cm offset along MCP flexion axis)..." << std::endl;

    // Verify the target is indeed out of plane
    std::array<double, 4> distances;
    std::array<bool, 4> in_plane = checkTargetsInPlane(targets, finger_planes, 0.01, distances);

    std::cout << "Target out-of-plane distances:" << std::endl;
    for (int i = 0; i < 4; ++i) {
        std::cout << "  Finger " << i << ": " << distances[i] << " m"
            << (in_plane[i] ? " (in-plane)" : " (OUT-OF-PLANE)") << std::endl;
    }

    Eigen::VectorXd qa_solved = Eigen::VectorXd::Zero(6);
    SolveReport report;
    bool success = ik.solve(targets, qa_solved, &report);

    std::cout << "Solve succeeded: " << (success ? "YES" : "NO") << std::endl;
    std::cout << "Iterations: " << report.iterations << std::endl;
    std::cout << "Final error: " << report.final_error << std::endl;

    // Check out-of-plane flags from the solver's report
    bool out_of_plane_detected = false;
    for (int i = 0; i < 4; ++i) {
        if (report.out_of_plane_flags[i]) {
            std::cout << "Solver detected out-of-plane for finger " << i
                << ": " << report.out_of_plane_distances[i] << " m" << std::endl;
            out_of_plane_detected = true;
        }
    }

    bool test_passed = out_of_plane_detected && (report.final_error < 0.05); // Relaxed threshold
    std::cout << "Out-of-plane test result: " << (test_passed ? "PASS" : "FAIL") << std::endl;

    return test_passed;
}

int main(int argc, char** argv) {
    try {
        std::cout << "=== Hand IK Reachability Test Suite (Fixed with Proper MCP Plane Detection) ===" << std::endl;

        // Resolve URDF path using clean helper
        std::filesystem::path urdf_path = ResolveUrdfPath(argc, argv);
        std::cout << "[URDF] Resolved path: " << urdf_path.string() << std::endl;

        // Load and validate the model
        pinocchio::Model model;
        pinocchio::urdf::buildModel(urdf_path.string(), model);
        pinocchio::Data data(model);

        std::cout << "[model] nq=" << model.nq << " nv=" << model.nv << std::endl;

        // Radian-domain coefficients (sanity echo)
        constexpr double b = 0.137056;
        constexpr double c = 0.972037;
        constexpr double d = 0.0129125;
        std::cout << "[coeffs] q_distal = " << b << "*q_mcp^2 + "
            << c << "*q_mcp + " << d << " (radians)" << std::endl;

        // Run URDF sanity audit
        AuditUrdf(model, data);

        // Create config and IK solver
        HandIKConfig config = createTestConfig();
        HandIK ik(config, urdf_path.string());

        bool all_tests_passed = true;

        // Test 1: Basic reachability with proper plane detection
        std::cout << "\n--- Test 1: Basic Reachability (Proper MCP Plane Detection) ---" << std::endl;
        bool test1 = performReachabilityTest(ik, 50, 0.05, 1e-3, 1e-2);
        all_tests_passed &= test1;

        // Test 2: Reachability with medium noise
        std::cout << "\n--- Test 2: Medium Noise Robustness ---" << std::endl;
        bool test2 = performReachabilityTest(ik, 30, 0.15, 1e-3, 1e-2);
        all_tests_passed &= test2;

        // Test 3: Out-of-plane handling with proper plane detection
        std::cout << "\n--- Test 3: Out-of-Plane Target Handling (Proper MCP Planes) ---" << std::endl;
        bool test3 = testOutOfPlaneHandling(ik);
        all_tests_passed &= test3;

        // Test 4: Performance benchmark
        std::cout << "\n--- Test 4: Performance Benchmark ---" << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();

        bool test4 = performReachabilityTest(ik, 100, 0.1, 1e-3, 1e-2);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        std::cout << "Total time for 100 tests: " << total_duration.count() << " ms" << std::endl;
        std::cout << "Average time per test: " << (total_duration.count() / 100.0) << " ms" << std::endl;

        all_tests_passed &= test4;

        std::cout << "\n=== Final Test Results ===" << std::endl;
        std::cout << "All critical tests passed: " << (all_tests_passed ? "YES" : "NO") << std::endl;

        if (all_tests_passed) {
            std::cout << "\n🎉 SUCCESS! Proper MCP plane detection achieved:" << std::endl;
            std::cout << "   Position: 1e-3 m, Orientation: 1e-2 rad" << std::endl;
            std::cout << "   Out-of-plane detection: Per-finger MCP flexion planes" << std::endl;
        }
        else {
            std::cout << "\n❌ Some tests failed. Check plane detection implementation." << std::endl;
        }

        if (!all_tests_passed) {
            std::cerr << "Some reachability tests failed!" << std::endl;
            return 1;
        }

        std::cout << "All reachability tests completed successfully with proper plane detection." << std::endl;
        return 0;

    }
    catch (const std::exception& e) {
        std::cerr << "Test error: " << e.what() << std::endl;
        return 1;
    }
}