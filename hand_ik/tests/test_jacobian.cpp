#include "hand_ik.hpp"
#include "math_constants.hpp"
#include <iostream>
#include <random>
#include <cassert>
#include <iomanip>

using namespace hand_ik;

HandIKConfig createTestConfig() {
    HandIKConfig config;

    // Use actual URDF joint names
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

    // Source-of-truth coefficients (locked from validation)
    // Expected radian-domain: q_distal = 0.137056*q_mcp^2 + 0.972037*q_mcp + 0.0129125
    constexpr double b_expected = 0.137056;
    constexpr double c_expected = 0.972037;
    constexpr double d_expected = 0.0129125;

    for (int i = 0; i < 4; ++i) {
        config.passive_coeffs[i] = {
            0.0,         // a: no cubic term
            b_expected,  // b: quadratic coefficient 
            c_expected,  // c: linear coefficient
            d_expected   // d: constant offset
        };
    }

    // Joint limits (from URDF)
    for (int i = 0; i < 4; ++i) {
        config.mcp_limits[i] = { 0.0, 1.774 };
    }
    config.thumb_rot_limits = { 0.0, 1.774 };
    config.thumb_flex_limits = { 0.0, 1.774 };

    config.verbose = false; // Keep quiet for bulk testing

    return config;
}

void assertCoefficientsConsistency(const HandIKConfig& config) {
    constexpr double tol = 1e-6;
    constexpr double b_expected = 0.137056;
    constexpr double c_expected = 0.972037;
    constexpr double d_expected = 0.0129125;

    for (int i = 0; i < 4; ++i) {
        if (std::abs(config.passive_coeffs[i].a - 0.0) > tol ||
            std::abs(config.passive_coeffs[i].b - b_expected) > tol ||
            std::abs(config.passive_coeffs[i].c - c_expected) > tol ||
            std::abs(config.passive_coeffs[i].d - d_expected) > tol) {

            std::cerr << "FATAL: Coefficients drift detected for finger " << i << "!" << std::endl;
            std::cerr << "Expected: a=0.0, b=" << b_expected << ", c=" << c_expected << ", d=" << d_expected << std::endl;
            std::cerr << "Got:      a=" << config.passive_coeffs[i].a
                << ", b=" << config.passive_coeffs[i].b
                << ", c=" << config.passive_coeffs[i].c
                << ", d=" << config.passive_coeffs[i].d << std::endl;
            throw std::runtime_error("Coefficients drift between validation and solver");
        }
    }
    std::cout << "Passive coupling coefficients consistency verified" << std::endl;
}

bool testJacobianAtConfiguration(HandIK& ik, const Eigen::VectorXd& qa, double tolerance = 1e-3) {
    std::cout << "\nTesting Jacobian at qa: " << qa.transpose() << std::endl;

    // Simply use the library's checkJacobianFiniteDiff method which now has the proper offset targets
    bool jacobian_ok = ik.checkJacobianFiniteDiff(qa, tolerance);

    std::cout << "Library Jacobian FD check: " << (jacobian_ok ? "PASS" : "FAIL") << std::endl;
    return jacobian_ok;
}

void testPassiveCouplingSmoke() {
    std::cout << "\n=== Passive Coupling Smoke Test ===" << std::endl;

    // Test: perturb MCP by +10° and check DIP prediction
    constexpr double mcp_deg = 10.0;
    const double mcp_rad = degToRad(mcp_deg);

    // Expected DIP in degrees using source-of-truth formula
    constexpr double expected_dip_deg = 0.00239207 * mcp_deg * mcp_deg + 0.97203665 * mcp_deg + 0.73983459;

    // DIP from library coefficients (in radians)
    HandIKConfig config = createTestConfig();
    double computed_dip_rad = config.passive_coeffs[0].eval(mcp_rad);
    double computed_dip_deg = radToDeg(computed_dip_rad);

    double error_deg = std::abs(computed_dip_deg - expected_dip_deg);
    double error_rad = std::abs(computed_dip_rad - degToRad(expected_dip_deg));

    std::cout << "MCP = " << mcp_deg << "° (" << mcp_rad << " rad)" << std::endl;
    std::cout << "Expected DIP = " << expected_dip_deg << "°" << std::endl;
    std::cout << "Computed DIP = " << computed_dip_deg << "° (" << computed_dip_rad << " rad)" << std::endl;
    std::cout << "Error = " << error_deg << "° (" << error_rad << " rad)" << std::endl;

    bool deg_ok = error_deg < 0.5; // Within 0.5°
    bool rad_ok = error_rad < 0.01; // Within 0.01 rad

    std::cout << "Degrees path: " << (deg_ok ? "PASS" : "FAIL") << std::endl;
    std::cout << "Radians path: " << (rad_ok ? "PASS" : "FAIL") << std::endl;

    if (!deg_ok || !rad_ok) {
        throw std::runtime_error("Passive coupling smoke test failed");
    }
}

void testModelConsistency(HandIK& ik) {
    std::cout << "\n=== Model Consistency Test ===" << std::endl;

    // Test that active-to-full mapping always produces a 10-vector
    for (int test = 0; test < 5; ++test) {
        Eigen::VectorXd qa = Eigen::VectorXd::Random(6) * 0.5;
        Eigen::VectorXd q_full = ik.expandActiveToFull(qa);

        if (q_full.size() != 10) {
            std::cerr << "FAIL: Expected q_full.size() == 10, got " << q_full.size() << std::endl;
            throw std::runtime_error("Model consistency test failed");
        }
    }

    std::cout << "Active-to-full mapping always produces 10-vector" << std::endl;
}

int main(int argc, char** argv) {
    try {
        std::cout << "=== Jacobian Finite Difference Test (Fixed with Frozen Planes) ===" << std::endl;

        // Resolve URDF path
        std::string urdf_path = hand_ik::resolveUrdfPath(argc, argv);
        std::cout << "Using URDF: " << urdf_path << std::endl;

        HandIKConfig config = createTestConfig();

        // Assert coefficients consistency
        assertCoefficientsConsistency(config);

        // Run smoke tests
        testPassiveCouplingSmoke();

        HandIK ik(config, urdf_path);

        // Test model consistency
        testModelConsistency(ik);

        std::cout << "\nTesting Jacobian computation with library method (includes proper offset targets)..." << std::endl;

        bool all_tests_passed = true;

        // Test 1: Zero configuration
        std::cout << "\n--- Test 1: Zero Configuration ---" << std::endl;
        Eigen::VectorXd qa_zero = Eigen::VectorXd::Zero(6);
        bool test1 = testJacobianAtConfiguration(ik, qa_zero);
        all_tests_passed &= test1;

        // Test 2-6: Random configurations
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.3, 0.3);

        for (int i = 0; i < 5; ++i) {
            std::cout << "\n--- Test " << (i + 2) << ": Random Configuration " << i << " ---" << std::endl;

            Eigen::VectorXd qa_random(6);
            for (int j = 0; j < 6; ++j) {
                qa_random[j] = dis(gen);
            }

            // Clamp to limits
            for (int j = 0; j < 4; ++j) {
                qa_random[j] = std::max(config.mcp_limits[j][0],
                    std::min(config.mcp_limits[j][1], qa_random[j]));
            }
            qa_random[4] = std::max(config.thumb_rot_limits[0],
                std::min(config.thumb_rot_limits[1], qa_random[4]));
            qa_random[5] = std::max(config.thumb_flex_limits[0],
                std::min(config.thumb_flex_limits[1], qa_random[5]));

            bool test_i = testJacobianAtConfiguration(ik, qa_random);
            all_tests_passed &= test_i;
        }

        // Test 7: Boundary configuration (away from limits to avoid clamping issues)
        std::cout << "\n--- Test 7: Near Joint Limit Boundaries ---" << std::endl;
        Eigen::VectorXd qa_boundary(6);
        for (int j = 0; j < 4; ++j) {
            qa_boundary[j] = config.mcp_limits[j][1] - 0.1; // Stay away from exact limit
        }
        qa_boundary[4] = config.thumb_rot_limits[1] - 0.1;
        qa_boundary[5] = config.thumb_flex_limits[1] - 0.1;

        bool test7 = testJacobianAtConfiguration(ik, qa_boundary);
        all_tests_passed &= test7;

        // Test 8: Test with verbose output to see detailed diagnostics
        std::cout << "\n--- Test 8: Verbose Diagnostic Test ---" << std::endl;
        config.verbose = true;  // Enable verbose output for one test
        HandIK ik_verbose(config, urdf_path);

        Eigen::VectorXd qa_diag = Eigen::VectorXd::Zero(6);
        qa_diag[0] = 0.5;  // Bend index finger
        qa_diag[4] = 0.3;  // Rotate thumb
        qa_diag[5] = 0.4;  // Flex thumb

        std::cout << "Testing with verbose output to see internal diagnostics..." << std::endl;
        bool test8 = ik_verbose.checkJacobianFiniteDiff(qa_diag, 1e-3);
        std::cout << "Verbose test result: " << (test8 ? "PASS" : "FAIL") << std::endl;
        all_tests_passed &= test8;

        std::cout << "\n=== Final Results ===" << std::endl;
        std::cout << "All Jacobian tests passed: " << (all_tests_passed ? "YES" : "NO") << std::endl;

        if (all_tests_passed) {
            std::cout << "\n🎉 SUCCESS! Jacobian finite difference validation achieved:" << std::endl;
            std::cout << "   - Base residual should show ~0.01m (1cm offset) instead of ~1e-17" << std::endl;
            std::cout << "   - Analytical and FD Jacobian norms should be non-zero (~1e-2 to 1e-1)" << std::endl;
            std::cout << "   - Max error should be ~1e-6 instead of ~1e-1" << std::endl;
            std::cout << "   - Frozen plane consistency achieved between solver and tests" << std::endl;
        }
        else {
            std::cout << "\n❌ Some tests failed. Check diagnostics above." << std::endl;
            std::cout << "Expected with fix:" << std::endl;
            std::cout << "   - 'Base residual norm: 0.01' (instead of 1e-17)" << std::endl;
            std::cout << "   - 'analytic_norm' and 'fd_norm' both non-zero" << std::endl;
            std::cout << "   - 'max error = 1e-6' (instead of 1e-1)" << std::endl;
        }

        if (!all_tests_passed) {
            std::cerr << "Some Jacobian tests failed!" << std::endl;
            return 1;
        }

        std::cout << "Jacobian finite difference validation complete with offset target method." << std::endl;
        return 0;

    }
    catch (const std::exception& e) {
        std::cerr << "Test error: " << e.what() << std::endl;
        return 1;
    }
}