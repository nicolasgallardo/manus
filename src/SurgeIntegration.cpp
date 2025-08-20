#include "SurgeIntegration.h"
#include <iostream>

// Optional integration with existing surge.cpp
// Use this if you already have a surge namespace and want to add Hand IK support

namespace surge {

bool HandIKIntegration::initialized_ = false;

bool HandIKIntegration::initializeHandIK(const std::string& urdfPath) {
    if (initialized_) {
        return true;
    }
    
    initialized_ = manus_handik::HandIKSolver::initialize(urdfPath, false);
    if (initialized_) {
        std::cout << "[Surge] Hand IK integrated successfully" << std::endl;
    } else {
        std::cerr << "[Surge] Hand IK integration failed: " 
                  << manus_handik::HandIKSolver::getLastError() << std::endl;
    }
    
    return initialized_;
}

void HandIKIntegration::processHandData(const manus_handik::FingerTips& tips, 
                                       manus_handik::HandSide side) {
    if (!initialized_) {
        return;
    }
    
    manus_handik::HandAngles angles;
    if (manus_handik::HandIKSolver::solve(tips, side, angles)) {
        std::string output = manus_handik::formatHandAngles(angles, side);
        
        // Send via your existing communication system
        // Uncomment and modify based on your existing sendString implementation:
        // sendString(output);
        
        // For now, just print to console
        std::cout << "[Surge] " << output << std::endl;
    } else {
        std::cerr << "[Surge] Hand IK solve failed: " 
                  << manus_handik::HandIKSolver::getLastError() << std::endl;
    }
}

std::string HandIKIntegration::getHandDataString(const manus_handik::FingerTips& tips,
                                                manus_handik::HandSide side) {
    if (!initialized_) {
        return "";
    }
    
    manus_handik::HandAngles angles;
    if (manus_handik::HandIKSolver::solve(tips, side, angles)) {
        return manus_handik::formatHandAngles(angles, side);
    }
    
    return "";
}

void HandIKIntegration::getPerformanceStats(double& avgTimeMs, double& maxTimeMs, size_t& solveCount) {
    if (initialized_) {
        manus_handik::HandIKSolver::getPerformanceStats(avgTimeMs, maxTimeMs, solveCount);
    } else {
        avgTimeMs = maxTimeMs = 0.0;
        solveCount = 0;
    }
}

void HandIKIntegration::shutdownHandIK() {
    if (initialized_) {
        manus_handik::HandIKSolver::shutdown();
        initialized_ = false;
        std::cout << "[Surge] Hand IK shutdown complete" << std::endl;
    }
}

} // namespace surge