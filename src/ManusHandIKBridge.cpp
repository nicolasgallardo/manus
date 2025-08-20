#include "ManusHandIKBridge.h"
#include "hand_ik.hpp"
#include "math_constants.hpp"
#include <iostream>
#include <sstream>
#include <memory>
#include <filesystem>
#include <numeric>
#include <algorithm>
#include <mutex>

namespace manus_handik {

// Performance monitoring class
class PerformanceMonitor {
private:
    std::vector<double> solveTimes_;
    std::chrono::steady_clock::time_point lastReport_;
    mutable std::mutex mutex_;
    
public:
    PerformanceMonitor() : lastReport_(std::chrono::steady_clock::now()) {}
    
    void recordSolveTime(double timeMs) {
        std::lock_guard<std::mutex> lock(mutex_);
        solveTimes_.push_back(timeMs);
        
        auto now = std::chrono::steady_clock::now();
        if (now - lastReport_ > std::chrono::seconds(10)) {
            reportStats();
            solveTimes_.clear();
            lastReport_ = now;
        }
    }
    
    void getStats(double& avgTimeMs, double& maxTimeMs, size_t& count) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (solveTimes_.empty()) {
            avgTimeMs = maxTimeMs = 0.0;
            count = 0;
            return;
        }
        
        double sum = std::accumulate(solveTimes_.begin(), solveTimes_.end(), 0.0);
        avgTimeMs = sum / solveTimes_.size();
        maxTimeMs = *std::max_element(solveTimes_.begin(), solveTimes_.end());
        count = solveTimes_.size();
    }
    
private:
    void reportStats() {
        if (solveTimes_.empty()) return;
        
        double sum = std::accumulate(solveTimes_.begin(), solveTimes_.end(), 0.0);
        double avg = sum / solveTimes_.size();
        double maxTime = *std::max_element(solveTimes_.begin(), solveTimes_.end());
        
        std::cout << "[HandIK] Performance - Avg: " << std::fixed << std::setprecision(2) 
                  << avg << "ms, Max: " << maxTime << "ms, Solves: " << solveTimes_.size() << std::endl;
    }
};

class HandIKSolver::HandIKSolverImpl {
public:
    std::unique_ptr<hand_ik::HandIK> leftHandIK;
    std::unique_ptr<hand_ik::HandIK> rightHandIK;
    hand_ik::HandIKConfig config;
    std::string lastError;
    bool initialized = false;
    PerformanceMonitor perfMonitor;
    
    // Cache for warm-start
    std::array<Eigen::VectorXd, 2> lastSolutions;  // [left=0, right=1]
    std::array<FingerTips, 2> lastTargets;
    
    HandIKSolverImpl() {
        lastSolutions[0] = Eigen::VectorXd::Zero(6);
        lastSolutions[1] = Eigen::VectorXd::Zero(6);
    }
    
    bool initializeConfig() {
        // Use your existing validated config
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
        
        // Use your validated passive coupling coefficients (LOCKED VALUES)
        constexpr double b_coeff = 0.137056;     // quadratic coefficient 
        constexpr double c_coeff = 0.972037;     // linear coefficient
        constexpr double d_coeff = 0.0129125;    // constant offset
        
        for (int i = 0; i < 4; ++i) {
            config.passive_coeffs[i] = {
                0.0,       // no cubic term
                b_coeff,   // quadratic coefficient 
                c_coeff,   // linear coefficient
                d_coeff    // constant offset
            };
        }
        
        // Joint limits from your URDF
        for (int i = 0; i < 4; ++i) {
            config.mcp_limits[i] = { 0.0, 1.774 };
        }
        config.thumb_rot_limits = { 0.0, 1.774 };
        config.thumb_flex_limits = { 0.0, 1.774 };
        
        // Solver parameters - optimized for real-time
        config.max_iterations = 50;           // Reduced for speed
        config.residual_tolerance = 1e-3;     // Relaxed for real-time
        config.step_tolerance = 1e-6;
        config.damping_init = 1e-3;
        config.plane_tolerance = 0.005;       // 5mm out-of-plane tolerance
        
        // Weights
        config.thumb_pos_weight = 1.0;
        config.thumb_rot_weight = 0.1;        // Reduced orientation weight
        config.finger_weights = { 1.0, 1.0, 1.0, 1.0 };
        
        return true;
    }
    
    double calculateMovement(const FingerTips& current, const FingerTips& previous) {
        double totalMovement = 0.0;
        for (int i = 0; i < 5; ++i) {
            auto& curr = current.positions[i];
            auto& prev = previous.positions[i];
            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;  
            double dz = curr.z - prev.z;
            totalMovement += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        return totalMovement;
    }
};

HandIKSolver::HandIKSolverImpl* HandIKSolver::impl_ = nullptr;

bool HandIKSolver::initialize(const std::string& urdfPath, bool verbose) {
    try {
        // Clean up any existing instance
        if (impl_) {
            delete impl_;
        }
        
        impl_ = new HandIKSolverImpl();
        impl_->config.verbose = verbose;
        
        // Verify URDF exists
        if (!std::filesystem::exists(urdfPath)) {
            impl_->lastError = "URDF file not found: " + urdfPath;
            return false;
        }
        
        if (!impl_->initializeConfig()) {
            impl_->lastError = "Failed to initialize configuration";
            return false;
        }
        
        // Create left and right hand solvers
        impl_->leftHandIK = std::make_unique<hand_ik::HandIK>(impl_->config, urdfPath);
        impl_->rightHandIK = std::make_unique<hand_ik::HandIK>(impl_->config, urdfPath);
        
        // Validate with Jacobian test
        Eigen::VectorXd testQa = Eigen::VectorXd::Zero(6);
        bool leftJacOk = impl_->leftHandIK->checkJacobianFiniteDiff(testQa, 1e-3);
        bool rightJacOk = impl_->rightHandIK->checkJacobianFiniteDiff(testQa, 1e-3);
        
        if (!leftJacOk || !rightJacOk) {
            impl_->lastError = "Jacobian validation failed";
            return false;
        }
        
        impl_->initialized = true;
        
        if (verbose) {
            std::cout << "[HandIK] Initialized successfully with URDF: " << urdfPath << std::endl;
            std::cout << "[HandIK] Passive coupling: q_distal = " 
                      << impl_->config.passive_coeffs[0].b << "*q_mcp^2 + "
                      << impl_->config.passive_coeffs[0].c << "*q_mcp + "
                      << impl_->config.passive_coeffs[0].d << std::endl;
        }
        
        return true;
        
    } catch (const std::exception& e) {
        if (impl_) {
            impl_->lastError = std::string("Initialization failed: ") + e.what();
        }
        return false;
    }
}

bool HandIKSolver::solve(const FingerTips& tips, HandSide side, HandAngles& outAngles) {
    if (!impl_ || !impl_->initialized) {
        return false;
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    
    try {
        // Select the appropriate hand solver
        auto& solver = (side == HandSide::Left) ? impl_->leftHandIK : impl_->rightHandIK;
        int sideIndex = (side == HandSide::Left) ? 0 : 1;
        
        // Check for minimal movement (optimization)
        double movement = impl_->calculateMovement(tips, impl_->lastTargets[sideIndex]);
        const double minMovementThreshold = 0.001; // 1mm total movement
        
        if (movement < minMovementThreshold && impl_->lastSolutions[sideIndex].size() == 6) {
            // Use cached solution for minimal movement
            outAngles.valid = true;
            outAngles.solveError = 0.0;
            outAngles.iterations = 0;
            
            // Convert cached solution to output format
            auto& qa = impl_->lastSolutions[sideIndex];
            for (int i = 0; i < 6; ++i) {
                outAngles.activeParams[i] = qa[i];
            }
            
            // Convert to joint angles
            for (int finger = 0; finger < 4; ++finger) {
                double mcp_rad = qa[finger];
                double pip_rad = impl_->config.passive_coeffs[finger].eval(mcp_rad);
                
                outAngles.degrees[finger][0] = hand_ik::radToDeg(mcp_rad);
                outAngles.degrees[finger][1] = hand_ik::radToDeg(pip_rad);
            }
            
            // Thumb
            outAngles.degrees[4][0] = hand_ik::radToDeg(qa[4]);  // rotation
            outAngles.degrees[4][1] = hand_ik::radToDeg(qa[5]);  // flexion
            
            return true;
        }
        
        // Convert Manus fingertip positions to hand_ik targets
        hand_ik::Targets targets;
        
        // Apply coordinate transform if needed (Manus to hand_ik coordinate system)
        auto transformPos = [side](const ManusVec3& pos) -> Eigen::Vector3d {
            Eigen::Vector3d result = manusToEigen(pos);
            
            // Apply any necessary coordinate transforms here
            // For example, if left hand needs mirroring:
            if (side == HandSide::Left) {
                // result.y() *= -1.0;  // Uncomment if mirroring needed
            }
            
            return result;
        };
        
        // Map Manus order to hand_ik order
        // Manus: Index=0, Middle=1, Ring=2, Pinky=3, Thumb=4
        // hand_ik: Index=0, Middle=1, Ring=2, Pinky=3
        for (int i = 0; i < 4; ++i) {
            targets.p_fingers[i] = transformPos(tips.positions[i]);
        }
        
        // Thumb position (hand_ik expects position + optional orientation)
        targets.p_thumb = transformPos(tips.positions[4]);
        // Don't constrain thumb orientation for now
        targets.R_thumb = std::nullopt;
        
        // Use warm-start from previous solution
        Eigen::VectorXd qa_solution = impl_->lastSolutions[sideIndex];
        if (qa_solution.size() != 6) {
            qa_solution = Eigen::VectorXd::Zero(6); // Fallback to neutral
        }
        
        hand_ik::SolveReport report;
        
        // Solve IK
        bool success = solver->solve(targets, qa_solution, &report);
        
        // Record timing
        auto end = std::chrono::high_resolution_clock::now();
        double timeMs = std::chrono::duration<double, std::milli>(end - start).count();
        impl_->perfMonitor.recordSolveTime(timeMs);
        
        // Fill output structure
        outAngles.valid = success;
        outAngles.solveError = report.final_error;
        outAngles.iterations = report.iterations;
        
        if (success) {
            // Cache solution and targets
            impl_->lastSolutions[sideIndex] = qa_solution;
            impl_->lastTargets[sideIndex] = tips;
            
            // Copy active parameters
            for (int i = 0; i < 6; ++i) {
                outAngles.activeParams[i] = qa_solution[i];
            }
            
            // Convert to individual joint angles (degrees)
            // Fingers: MCP (active), PIP (passive computed from MCP)
            for (int finger = 0; finger < 4; ++finger) {
                double mcp_rad = qa_solution[finger];
                double pip_rad = impl_->config.passive_coeffs[finger].eval(mcp_rad);
                
                outAngles.degrees[finger][0] = hand_ik::radToDeg(mcp_rad);      // MCP (active)
                outAngles.degrees[finger][1] = hand_ik::radToDeg(pip_rad);      // PIP (passive)
            }
            
            // Thumb: 2 active joints
            outAngles.degrees[4][0] = hand_ik::radToDeg(qa_solution[4]);  // rotation (active)
            outAngles.degrees[4][1] = hand_ik::radToDeg(qa_solution[5]);  // flexion (active)
        }
        
        // Log warnings for out-of-plane targets
        if (impl_->config.verbose && success) {
            for (int i = 0; i < 4; ++i) {
                if (report.out_of_plane_flags[i]) {
                    std::cout << "[HandIK] Warning: Finger " << i << " target out of flexion plane by "
                              << report.out_of_plane_distances[i] << " m" << std::endl;
                }
            }
        }
        
        return success;
        
    } catch (const std::exception& e) {
        impl_->lastError = std::string("Solve failed: ") + e.what();
        outAngles.valid = false;
        return false;
    }
}

std::string HandIKSolver::getLastError() {
    return impl_ ? impl_->lastError : "Not initialized";
}

void HandIKSolver::getPerformanceStats(double& avgTimeMs, double& maxTimeMs, size_t& solveCount) {
    if (impl_) {
        impl_->perfMonitor.getStats(avgTimeMs, maxTimeMs, solveCount);
    } else {
        avgTimeMs = maxTimeMs = 0.0;
        solveCount = 0;
    }
}

void HandIKSolver::shutdown() {
    delete impl_;
    impl_ = nullptr;
}

// Utility functions
ManusVec3 eigenToManus(const Eigen::Vector3d& v) {
    return ManusVec3(static_cast<float>(v.x()), static_cast<float>(v.y()), static_cast<float>(v.z()));
}

Eigen::Vector3d manusToEigen(const ManusVec3& v) {
    return Eigen::Vector3d(static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z));
}

std::string formatHandAngles(const HandAngles& angles, HandSide side) {
    if (!angles.valid) {
        return "";
    }
    
    std::ostringstream ss;
    ss << "HAND:" << (side == HandSide::Left ? "L" : "R") << ";";
    
    const char* fingerNames[] = {"index", "middle", "ring", "pinky", "thumb"};
    
    for (int i = 0; i < 5; ++i) {
        ss << fingerNames[i] << "("
           << std::fixed << std::setprecision(1)
           << angles.degrees[i][0] << ","
           << angles.degrees[i][1] << ")";
        if (i < 4) ss << ";";
    }
    
    ss << ";error=" << std::setprecision(3) << angles.solveError
       << ";iter=" << angles.iterations;
    
    return ss.str();
}

} // namespace manus_handik