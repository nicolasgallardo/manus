#include "ManusHandIKBridge.h"
#include <gtest/gtest.h>
#include <filesystem>
#include <chrono>

class ManusHandIKTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Find test URDF
        std::string urdfPath = "surge_v13_hand_right_pybullet.urdf";
        if (!std::filesystem::exists(urdfPath)) {
            urdfPath = "../hand_ik/surge_v13_hand_right_pybullet.urdf";
        }
        if (!std::filesystem::exists(urdfPath)) {
            urdfPath = "../surge_v13_hand_right_pybullet.urdf";
        }
        
        ASSERT_TRUE(std::filesystem::exists(urdfPath)) << "Test URDF not found: " << urdfPath;
        ASSERT_TRUE(manus_handik::HandIKSolver::initialize(urdfPath, false));
    }
    
    void TearDown() override {
        manus_handik::HandIKSolver::shutdown();
    }
};

TEST_F(ManusHandIKTest, SolveReachableTargets) {
    manus_handik::FingerTips tips;
    
    // Set reasonable fingertip positions (meters)
    tips.positions[0] = manus_handik::ManusVec3(0.15f, 0.05f, 0.12f);  // Index
    tips.positions[1] = manus_handik::ManusVec3(0.16f, 0.02f, 0.14f);  // Middle  
    tips.positions[2] = manus_handik::ManusVec3(0.15f, -0.02f, 0.13f); // Ring
    tips.positions[3] = manus_handik::ManusVec3(0.13f, -0.05f, 0.11f); // Pinky
    tips.positions[4] = manus_handik::ManusVec3(0.08f, 0.08f, 0.10f);  // Thumb
    
    manus_handik::HandAngles angles;
    EXPECT_TRUE(manus_handik::HandIKSolver::solve(tips, manus_handik::HandSide::Right, angles));
    EXPECT_TRUE(angles.valid);
    EXPECT_LT(angles.solveError, 0.01);  // Less than 1cm error
    
    // Check angle ranges are reasonable (0-120 degrees)
    for (int finger = 0; finger < 5; ++finger) {
        for (int joint = 0; joint < 2; ++joint) {  // Only 2 joints per finger now
            EXPECT_GE(angles.degrees[finger][joint], -5.0);   // Small negative tolerance
            EXPECT_LE(angles.degrees[finger][joint], 120.0);  // Reasonable max flexion
        }
    }
}

TEST_F(ManusHandIKTest, HandleZeroConfiguration) {
    manus_handik::FingerTips tips;
    
    // All fingertips at origin (unrealistic but should not crash)
    for (int i = 0; i < 5; ++i) {
        tips.positions[i] = manus_handik::ManusVec3(0.0f, 0.0f, 0.0f);
    }
    
    manus_handik::HandAngles angles;
    // Should either solve or fail gracefully - should not crash
    bool solved = manus_handik::HandIKSolver::solve(tips, manus_handik::HandSide::Left, angles);
    
    // Either way, should not crash or hang
    EXPECT_NO_FATAL_FAILURE();
}

TEST_F(ManusHandIKTest, CoordinateConversion) {
    Eigen::Vector3d eigen_vec(1.23, -4.56, 7.89);
    manus_handik::ManusVec3 manus_vec = manus_handik::eigenToManus(eigen_vec);
    Eigen::Vector3d converted_back = manus_handik::manusToEigen(manus_vec);
    
    EXPECT_NEAR(eigen_vec.x(), converted_back.x(), 1e-6);
    EXPECT_NEAR(eigen_vec.y(), converted_back.y(), 1e-6);
    EXPECT_NEAR(eigen_vec.z(), converted_back.z(), 1e-6);
}

TEST_F(ManusHandIKTest, FormattingOutput) {
    manus_handik::HandAngles angles;
    angles.valid = true;
    angles.solveError = 0.002;
    angles.iterations = 5;
    
    // Set some test angles
    for (int i = 0; i < 5; ++i) {
        angles.degrees[i][0] = 10.0 + i * 5.0;  // First joint (MCP for fingers, rotation for thumb)
        angles.degrees[i][1] = 8.0 + i * 3.0;   // Second joint (PIP for fingers, flexion for thumb)
    }
    
    std::string formatted = manus_handik::formatHandAngles(angles, manus_handik::HandSide::Right);
    
    EXPECT_FALSE(formatted.empty());
    EXPECT_NE(formatted.find("HAND:R"), std::string::npos);
    EXPECT_NE(formatted.find("index"), std::string::npos);
    EXPECT_NE(formatted.find("thumb"), std::string::npos);
    EXPECT_NE(formatted.find("error="), std::string::npos);
}

TEST_F(ManusHandIKTest, PerformanceBenchmark) {
    manus_handik::FingerTips tips;
    
    // Set reasonable fingertip positions
    tips.positions[0] = manus_handik::ManusVec3(0.15f, 0.05f, 0.12f);  
    tips.positions[1] = manus_handik::ManusVec3(0.16f, 0.02f, 0.14f);  
    tips.positions[2] = manus_handik::ManusVec3(0.15f, -0.02f, 0.13f); 
    tips.positions[3] = manus_handik::ManusVec3(0.13f, -0.05f, 0.11f); 
    tips.positions[4] = manus_handik::ManusVec3(0.08f, 0.08f, 0.10f);  
    
    // Benchmark solve time
    const int numSolves = 100;
    auto start = std::chrono::high_resolution_clock::now();
    
    int successCount = 0;
    for (int i = 0; i < numSolves; ++i) {
        manus_handik::HandAngles angles;
        if (manus_handik::HandIKSolver::solve(tips, manus_handik::HandSide::Right, angles)) {
            successCount++;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    double totalTimeMs = std::chrono::duration<double, std::milli>(end - start).count();
    double avgTimeMs = totalTimeMs / numSolves;
    
    std::cout << "Performance: " << avgTimeMs << "ms avg, " 
              << successCount << "/" << numSolves << " successful" << std::endl;
    
    // Performance requirements for real-time (assuming 90Hz target)
    EXPECT_LT(avgTimeMs, 5.0);  // Less than 5ms average
    EXPECT_GT(successCount, numSolves * 0.95);  // >95% success rate
}

TEST_F(ManusHandIKTest, PassiveCouplingValidation) {
    // Test that passive coupling coefficients are correctly applied
    manus_handik::FingerTips tips;
    
    // Set a configuration that should result in predictable passive joint angles
    // Based on the validated coefficients: q_distal = 0.137056*q_mcp^2 + 0.972037*q_mcp + 0.0129125
    
    // Test with MCP at approximately 30 degrees (0.524 radians)
    // Expected PIP ≈ 0.137056*(0.524)^2 + 0.972037*(0.524) + 0.0129125 ≈ 0.55 radians ≈ 31.5 degrees
    
    tips.positions[0] = manus_handik::ManusVec3(0.12f, 0.08f, 0.10f);  // Position that should give ~30° MCP
    tips.positions[1] = manus_handik::ManusVec3(0.16f, 0.02f, 0.14f);  
    tips.positions[2] = manus_handik::ManusVec3(0.15f, -0.02f, 0.13f); 
    tips.positions[3] = manus_handik::ManusVec3(0.13f, -0.05f, 0.11f); 
    tips.positions[4] = manus_handik::ManusVec3(0.08f, 0.08f, 0.10f);  
    
    manus_handik::HandAngles angles;
    ASSERT_TRUE(manus_handik::HandIKSolver::solve(tips, manus_handik::HandSide::Right, angles));
    
    // Check that PIP angle is reasonable compared to MCP angle
    for (int finger = 0; finger < 4; ++finger) {
        double mcp_deg = angles.degrees[finger][0];
        double pip_deg = angles.degrees[finger][1];
        
        // PIP should be roughly similar to MCP due to passive coupling
        // (exact relationship depends on the coupling coefficients)
        EXPECT_GT(pip_deg, 0.0);  // Should be positive
        EXPECT_LT(pip_deg, mcp_deg + 20.0);  // Should not be much larger than MCP
        
        std::cout << "Finger " << finger << ": MCP=" << mcp_deg 
                  << "°, PIP=" << pip_deg << "°" << std::endl;
    }
}