#include "hand_ik.hpp"
#include "math_constants.hpp"
#include <iostream>
#include <filesystem>

using namespace hand_ik;

int main(int argc, char** argv) {
    try {
        std::cout << "=== Hand IK Example (Stub) ===" << std::endl;
        std::cout << "This is a minimal example demonstrating hand_ik library usage." << std::endl;
        std::cout << std::endl;

        // Try to find URDF file
        std::string urdf_path;
        try {
            urdf_path = resolveUrdfPath(argc, argv);
            std::cout << "Found URDF: " << urdf_path << std::endl;
        }
        catch (const std::exception& e) {
            std::cout << "URDF not found: " << e.what() << std::endl;
            std::cout << "This is expected in a stub example." << std::endl;
        }

        // Create a minimal configuration
        HandIKConfig config;
        
        // Set basic joint names (these may not match actual URDF)
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

        // Set passive coupling coefficients (corrected values)
        constexpr double b = 0.137056;     // quadratic coefficient 
        constexpr double c = 0.972037;     // linear coefficient
        constexpr double d = 0.0129125;    // constant offset

        for (int i = 0; i < 4; ++i) {
            config.passive_coeffs[i] = {
                0.0,  // cubic coefficient (0.0)
                b,    // quadratic coefficient 
                c,    // linear coefficient
                d     // constant offset
            };
        }

        // Set joint limits
        for (int i = 0; i < 4; ++i) {
            config.mcp_limits[i] = { 0.0, 1.774 };
        }
        config.thumb_rot_limits = { 0.0, 1.774 };
        config.thumb_flex_limits = { 0.0, 1.774 };

        config.verbose = true;

        std::cout << "\nConfiguration created successfully!" << std::endl;
        std::cout << "Passive coupling coefficients (per finger):" << std::endl;
        std::cout << "  q_distal = " << b << "*q_mcp^2 + " << c << "*q_mcp + " << d << std::endl;

        // Test basic math functions
        std::cout << "\nTesting math utilities:" << std::endl;
        double test_deg = 45.0;
        double test_rad = degToRad(test_deg);
        double back_to_deg = radToDeg(test_rad);
        
        std::cout << "  " << test_deg << "° = " << test_rad << " rad = " << back_to_deg << "°" << std::endl;
        std::cout << "  kPi = " << kPi << std::endl;

        // If URDF was found, try to initialize (will likely fail in stub)
        if (!urdf_path.empty() && std::filesystem::exists(urdf_path)) {
            try {
                std::cout << "\nAttempting to initialize HandIK with URDF..." << std::endl;
                HandIK ik(config, urdf_path);
                std::cout << "HandIK initialized successfully!" << std::endl;
                
                // Test basic functionality
                Eigen::VectorXd qa_test = Eigen::VectorXd::Zero(6);
                bool jacobian_ok = ik.checkJacobianFiniteDiff(qa_test, 1e-3);
                std::cout << "Jacobian check: " << (jacobian_ok ? "PASS" : "FAIL") << std::endl;
                
            }
            catch (const std::exception& e) {
                std::cout << "HandIK initialization failed: " << e.what() << std::endl;
                std::cout << "This is expected if joint/frame names don't match the URDF." << std::endl;
            }
        }

        std::cout << "\n=== Example completed successfully ===" << std::endl;
        return 0;

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}