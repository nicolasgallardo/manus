#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>
#include <Eigen/Dense>

// Forward declarations to avoid SDK header dependencies
struct ManusVec3 { 
    float x, y, z; 
    ManusVec3() : x(0), y(0), z(0) {}
    ManusVec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

namespace manus_handik {

enum class HandSide { Left, Right };

struct FingerTips {
    // Fingertip positions in world coordinates (meters)
    // Order: Index=0, Middle=1, Ring=2, Pinky=3, Thumb=4
    std::array<ManusVec3, 5> positions;
    
    FingerTips() {
        for (auto& pos : positions) {
            pos = ManusVec3(0.0f, 0.0f, 0.0f);
        }
    }
};

struct HandAngles {
    // Joint angles in degrees for output
    // [finger][joint]: 
    // - Fingers 0-3 (Index/Middle/Ring/Pinky): [0]=MCP(active), [1]=PIP(passive)
    // - Thumb (finger 4): [0]=rotation(active), [1]=flexion(active)
    std::array<std::array<double, 2>, 5> degrees;
    
    // Active parameters from your IK solver (6 DOF total)
    // [0-3]: MCP joints for Index/Middle/Ring/Pinky  
    // [4-5]: Thumb rotation and flexion
    std::array<double, 6> activeParams;
    
    bool valid = false;
    double solveError = 0.0;
    int iterations = 0;
    
    HandAngles() {
        for (auto& finger : degrees) {
            finger.fill(0.0);
        }
        activeParams.fill(0.0);
    }
};

class HandIKSolver {
public:
    // Initialize with your URDF and config
    static bool initialize(const std::string& urdfPath, bool verbose = false);
    
    // Solve IK for fingertip targets
    static bool solve(const FingerTips& tips, HandSide side, HandAngles& outAngles);
    
    // Get solver diagnostics
    static std::string getLastError();
    
    // Performance monitoring
    static void getPerformanceStats(double& avgTimeMs, double& maxTimeMs, size_t& solveCount);
    
    // Shutdown and cleanup
    static void shutdown();

private:
    static class HandIKSolverImpl* impl_;
};

// Utility functions for coordinate conversion
ManusVec3 eigenToManus(const Eigen::Vector3d& v);
Eigen::Vector3d manusToEigen(const ManusVec3& v);

// Format output for your existing pipeline
std::string formatHandAngles(const HandAngles& angles, HandSide side);

} // namespace manus_handik