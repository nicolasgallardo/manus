#pragma once

#include "ManusHandIKBridge.h"
#include <string>

// Optional integration with existing surge.h/surge.cpp
// Use this if you already have a surge namespace and want to add Hand IK support

namespace surge {

class HandIKIntegration {
public:
    // Initialize hand IK system
    static bool initializeHandIK(const std::string& urdfPath);
    
    // Process Manus hand data and send via existing surge pipeline  
    static void processHandData(const manus_handik::FingerTips& tips, 
                               manus_handik::HandSide side);
    
    // Get formatted output without sending
    static std::string getHandDataString(const manus_handik::FingerTips& tips,
                                        manus_handik::HandSide side);
    
    // Performance monitoring
    static void getPerformanceStats(double& avgTimeMs, double& maxTimeMs, size_t& solveCount);
    
    // Shutdown hand IK
    static void shutdownHandIK();
    
private:
    static bool initialized_;
};

// If you already have a sendString function in your surge system, declare it here:
// extern void sendString(const std::string& data);

} // namespace surge