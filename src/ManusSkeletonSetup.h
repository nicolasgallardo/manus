#pragma once

#include <cstdint>

// Forward declarations for Manus SDK types
enum class SDKReturnCode : int;
enum class NodeType : int;
enum class ChainType : int;  
enum class Side : int;

namespace manus_skeleton {

class HandSkeletonSetup {
public:
    struct SkeletonIDs {
        uint32_t leftHandId = 0;
        uint32_t rightHandId = 0;
        bool valid = false;
    };
    
    // Create retargeted hand skeletons with fingertip nodes
    static bool createHandSkeletons(SkeletonIDs& outIds);
    
    // Extract fingertip positions from skeleton data
    static bool extractFingerTips(uint32_t skeletonId, 
                                  float* fingerTips,  // 15 floats: 5 fingers * 3 coords
                                  size_t maxNodes);
    
private:
    static bool setupLeftHand(uint32_t& skeletonId);
    static bool setupRightHand(uint32_t& skeletonId);
    static bool addHandNodes(uint32_t setupIndex);
    static bool addHandChains(uint32_t setupIndex, Side handSide);
};

} // namespace manus_skeleton