// tests/test_manus_integration.cpp - Headless smoke test for Manus integration
#include "ManusHandIKBridge.h"
#include "ManusSkeletonSetup.h"
#include "hand_ik.hpp"
#include <iostream>
#include <chrono>
#include <random>
#include <filesystem>

class ManusIntegrationTest {
public:
    bool RunAllTests() {
        std::cout << "=== Manus Integration Smoke Test ===" << std::endl;

        bool all_passed = true;

        all_passed &= TestUrdfResolution();
        all_passed &= TestHandIKBridgeCreation();
        all_passed &= TestCoordinateConversion();
        all_passed &= TestIKSolving();
        all_passed &= TestPerformance();

        std::cout << "\n=== Test Summary ===" << std::endl;
        std::cout << "Overall result: " << (all_passed ? "✓ PASS" : "❌ FAIL") << std::endl;

        return all_passed;
    }

private:
    bool TestUrdfResolution() {
        std::cout << "\n--- Test 1: URDF Resolution ---" << std::endl;

        try {
            // Test URDF resolution logic
            std::vector<std::string> test_paths = {
                "surge_v13_hand_right_pybullet.urdf",
                "./surge_v13_hand_right_pybullet.urdf",
                "hand_ik/surge_v13_hand_right_pybullet.urdf"
            };

            std::string resolved_path;
            bool found = false;

            for (const auto& path : test_paths) {
                if (std::filesystem::exists(path)) {
                    resolved_path = path;
                    found = true;
                    break;
                }
            }

            if (found) {
                std::cout << "✓ URDF found at: " << resolved_path << std::endl;
                return true;
            }
            else {
                std::cout << "❌ URDF file not found in expected locations" << std::endl;
                std::cout << "   Checked paths:" << std::endl;
                for (const auto& path : test_paths) {
                    std::cout << "   - " << path << std::endl;
                }
                return false;
            }
        }
        catch (const std::exception& e) {
            std::cout << "❌ URDF resolution test failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool TestHandIKBridgeCreation() {
        std::cout << "\n--- Test 2: Hand IK Bridge Creation ---" << std::endl;

        try {
            // Find URDF file
            std::string urdf_path = "surge_v13_hand_right_pybullet.urdf";
            if (!std::filesystem::exists(urdf_path)) {
                urdf_path = "./surge_v13_hand_right_pybullet.urdf";
            }
            if (!std::filesystem::exists(urdf_path)) {
                std::cout << "❌ URDF file not found for bridge test" << std::endl;
                return false;
            }

            // Create bridge
            std::unique_ptr<ManusHandIKBridge> bridge =
                std::make_unique<ManusHandIKBridge>(urdf_path);

            if (!bridge->IsInitialized()) {
                std::cout << "❌ Bridge failed to initialize" << std::endl;
                return false;
            }

            std::cout << "✓ Hand IK Bridge created successfully" << std::endl;

            // Test diagnostics
            bool diagnostics_pass = bridge->RunDiagnostics();
            std::cout << "Bridge diagnostics: " << (diagnostics_pass ? "✓ PASS" : "⚠️  WARN") << std::endl;

            return true;

        }
        catch (const std::exception& e) {
            std::cout << "❌ Bridge creation failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool TestCoordinateConversion() {
        std::cout << "\n--- Test 3: Coordinate System Conversion ---" << std::endl;

        try {
            std::string urdf_path = "surge_v13_hand_right_pybullet.urdf";
            if (!std::filesystem::exists(urdf_path)) {
                urdf_path = "./surge_v13_hand_right_pybullet.urdf";
            }

            ManusHandIKBridge bridge(urdf_path);

            // Test position conversion
            Eigen::Vector3d manus_pos(0.1, 0.05, 0.12); // 10cm, 5cm, 12cm
            Eigen::Vector3d ik_pos = bridge.ManusToIKPosition(manus_pos);

            // Should be identity transform for our coordinate system
            double pos_error = (ik_pos - manus_pos).norm();
            if (pos_error > 1e-6) {
                std::cout << "❌ Position conversion error: " << pos_error << std::endl;
                return false;
            }

            // Test orientation conversion
            Eigen::Matrix3d manus_rot = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d ik_rot = bridge.ManusToIKOrientation(manus_rot);

            double rot_error = (ik_rot - manus_rot).norm();
            if (rot_error > 1e-6) {
                std::cout << "❌ Orientation conversion error: " << rot_error << std::endl;
                return false;
            }

            std::cout << "✓ Coordinate conversions working correctly" << std::endl;
            return true;

        }
        catch (const std::exception& e) {
            std::cout << "❌ Coordinate conversion test failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool TestIKSolving() {
        std::cout << "\n--- Test 4: IK Solving Pipeline ---" << std::endl;

        try {
            std::string urdf_path = "surge_v13_hand_right_pybullet.urdf";
            if (!std::filesystem::exists(urdf_path)) {
                urdf_path = "./surge_v13_hand_right_pybullet.urdf";
            }

            ManusHandIKBridge bridge(urdf_path);

            // Create realistic finger targets
            FingerTargets targets;
            targets.finger_positions[0] = Eigen::Vector3d(0.15, 0.05, 0.12);  // Index
            targets.finger_positions[1] = Eigen::Vector3d(0.16, 0.02, 0.14);  // Middle  
            targets.finger_positions[2] = Eigen::Vector3d(0.15, -0.02, 0.13); // Ring
            targets.finger_positions[3] = Eigen::Vector3d(0.13, -0.05, 0.11); // Pinky
            targets.thumb_position = Eigen::Vector3d(0.08, 0.08, 0.10);       // Thumb
            targets.thumb_rotation = Eigen::Matrix3d::Identity();

            // Solve IK
            JointConfiguration joint_config;
            bool success = bridge.Solve(targets, joint_config);

            if (!success || !joint_config.valid) {
                std::cout << "❌ IK solve failed" << std::endl;
                return false;
            }

            // Validate joint angles are within reasonable bounds
            for (int i = 0; i < 6; ++i) {
                double angle = joint_config.joint_angles[i];
                if (std::isnan(angle) || std::isinf(angle)) {
                    std::cout << "❌ Invalid joint angle[" << i << "]: " << angle << std::endl;
                    return false;
                }

                if (angle < -0.1 || angle > 2.0) { // Reasonable bounds
                    std::cout << "❌ Joint angle[" << i << "] out of bounds: " << angle << std::endl;
                    return false;
                }
            }

            std::cout << "✓ IK solve successful" << std::endl;
            std::cout << "  Solve time: " << joint_config.solve_time_ms << " ms" << std::endl;
            std::cout << "  Iterations: " << joint_config.iterations << std::endl;

            // Print joint angles for inspection
            std::cout << "  Joint angles (rad): ";
            for (int i = 0; i < 6; ++i) {
                std::cout << std::fixed << std::setprecision(3) << joint_config.joint_angles[i];
                if (i < 5) std::cout << ", ";
            }
            std::cout << std::endl;

            return true;

        }
        catch (const std::exception& e) {
            std::cout << "❌ IK solving test failed: " << e.what() << std::endl;
            return false;
        }
    }

    bool TestPerformance() {
        std::cout << "\n--- Test 5: Performance Benchmark ---" << std::endl;

        try {
            std::string urdf_path = "surge_v13_hand_right_pybullet.urdf";
            if (!std::filesystem::exists(urdf_path)) {
                urdf_path = "./surge_v13_hand_right_pybullet.urdf";
            }

            ManusHandIKBridge bridge(urdf_path);

            // Performance test parameters
            const int num_tests = 100;
            const double max_acceptable_time_ms = 5.0; // 5ms max for real-time

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(0.08, 0.18); // Reasonable finger reach

            std::vector<double> solve_times;
            int successful_solves = 0;

            auto start_time = std::chrono::high_resolution_clock::now();

            for (int test = 0; test < num_tests; ++test) {
                // Generate random but reachable targets
                FingerTargets targets;
                for (int i = 0; i < 4; ++i) {
                    targets.finger_positions[i] = Eigen::Vector3d(
                        dis(gen),
                        dis(gen) - 0.1,
                        dis(gen)
                    );
                }
                targets.thumb_position = Eigen::Vector3d(0.08, 0.08, 0.10);
                targets.thumb_rotation = Eigen::Matrix3d::Identity();

                // Solve and time
                JointConfiguration joint_config;
                bool success = bridge.Solve(targets, joint_config);

                if (success && joint_config.valid) {
                    successful_solves++;
                    solve_times.push_back(joint_config.solve_time_ms);
                }
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto total_time = std::chrono::duration<double, std::milli>(end_time - start_time);

            // Calculate statistics
            double success_rate = double(successful_solves) / num_tests;
            double avg_time = 0.0;
            double max_time = 0.0;

            if (!solve_times.empty()) {
                for (double time : solve_times) {
                    avg_time += time;
                    max_time = std::max(max_time, time);
                }
                avg_time /= solve_times.size();
            }

            std::cout << "Performance Results:" << std::endl;
            std::cout << "  Total tests: " << num_tests << std::endl;
            std::cout << "  Successful: " << successful_solves << " ("
                << std::fixed << std::setprecision(1) << success_rate * 100 << "%)" << std::endl;
            std::cout << "  Average solve time: " << std::setprecision(3) << avg_time << " ms" << std::endl;
            std::cout << "  Maximum solve time: " << std::setprecision(3) << max_time << " ms" << std::endl;
            std::cout << "  Total benchmark time: " << std::setprecision(1) << total_time.count() << " ms" << std::endl;

            // Performance criteria
            bool success_rate_ok = success_rate >= 0.8; // 80% success
            bool avg_time_ok = avg_time <= max_acceptable_time_ms;
            bool max_time_ok = max_time <= max_acceptable_time_ms * 2; // 2x tolerance for max

            std::cout << "Performance Criteria:" << std::endl;
            std::cout << "  Success rate ≥80%: " << (success_rate_ok ? "✓ PASS" : "❌ FAIL") << std::endl;
            std::cout << "  Avg time ≤5ms: " << (avg_time_ok ? "✓ PASS" : "❌ FAIL") << std::endl;
            std::cout << "  Max time ≤10ms: " << (max_time_ok ? "✓ PASS" : "❌ FAIL") << std::endl;

            return success_rate_ok && avg_time_ok && max_time_ok;

        }
        catch (const std::exception& e) {
            std::cout << "❌ Performance test failed: " << e.what() << std::endl;
            return false;
        }
    }
};

int main(int argc, char* argv[]) {
    std::cout << "Manus Integration Smoke Test" << std::endl;
    std::cout << "============================" << std::endl;
    std::cout << "Running headless validation of Manus + Hand IK integration..." << std::endl;

    try {
        ManusIntegrationTest test;
        bool success = test.RunAllTests();

        if (success) {
            std::cout << "\n🎉 All smoke tests passed!" << std::endl;
            std::cout << "Integration is ready for real-time use with Manus Core." << std::endl;
            return 0;
        }
        else {
            std::cout << "\n❌ Some tests failed!" << std::endl;
            std::cout << "Please fix issues before deploying integration." << std::endl;
            return 1;
        }

    }
    catch (const std::exception& e) {
        std::cerr << "❌ Test suite error: " << e.what() << std::endl;
        return 1;
    }
}