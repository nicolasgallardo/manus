#include "ManusSkeletonSetup.h"
#include <iostream>
#include <vector>

// You'll need to include your actual Manus SDK headers here
// #include "CoreSdk.h"
// For now, we'll use placeholders that match the Manus SDK API

// Placeholder declarations - replace with your actual Manus SDK includes
extern "C" {
    enum class SDKReturnCode : int { SDKReturnCode_Success = 0 };
    enum class NodeType : int { NodeType_Joint = 0 };
    enum class ChainType : int { 
        ChainType_Hand = 0, 
        ChainType_FingerIndex = 1,
        ChainType_FingerMiddle = 2, 
        ChainType_FingerRing = 3,
        ChainType_FingerPinky = 4,
        ChainType_FingerThumb = 5
    };
    enum class Side : int { Side_Left = 0, Side_Right = 1 };
    
    struct SkeletonNode {
        struct { float x, y, z; } position;
        struct { float w, x, y, z; } rotation;
    };
    
    // Function declarations - replace with your actual Manus SDK
    SDKReturnCode CoreSdk_CreateSkeletonSetup(uint32_t* setupIndex);
    SDKReturnCode CoreSdk_AddNodeToSkeletonSetup(uint32_t setupIndex, uint32_t nodeId, 
                                                  float x, float y, float z, NodeType type);
    SDKReturnCode CoreSdk_AddChainToSkeletonSetup(uint32_t setupIndex, uint32_t chainId,
                                                   uint32_t nodeIdStart, uint32_t nodeIdEnd,
                                                   ChainType type, Side side);
    SDKReturnCode CoreSdk_LoadSkeleton(uint32_t setupIndex, uint32_t* skeletonId);
    SDKReturnCode CoreSdk_GetSkeletonData(uint32_t skeletonIndex, SkeletonNode* nodes, uint32_t nodeCount);
}

namespace manus_skeleton {

bool HandSkeletonSetup::createHandSkeletons(SkeletonIDs& outIds) {
    std::cout << "[Skeleton] Creating hand skeletons with fingertip nodes..." << std::endl;
    
    // Setup left hand
    if (!setupLeftHand(outIds.leftHandId)) {
        std::cerr << "[Skeleton] Failed to setup left hand" << std::endl;
        return false;
    }
    
    // Setup right hand  
    if (!setupRightHand(outIds.rightHandId)) {
        std::cerr << "[Skeleton] Failed to setup right hand" << std::endl;
        return false;
    }
    
    outIds.valid = true;
    
    std::cout << "[Skeleton] Hand skeletons created successfully:" << std::endl;
    std::cout << "  Left hand ID: " << outIds.leftHandId << std::endl;
    std::cout << "  Right hand ID: " << outIds.rightHandId << std::endl;
    
    return true;
}

bool HandSkeletonSetup::setupLeftHand(uint32_t& skeletonId) {
    uint32_t setupIndex = 0;
    
    if (CoreSdk_CreateSkeletonSetup(&setupIndex) != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    if (!addHandNodes(setupIndex)) {
        return false;
    }
    
    if (!addHandChains(setupIndex, Side::Side_Left)) {
        return false;
    }
    
    return CoreSdk_LoadSkeleton(setupIndex, &skeletonId) == SDKReturnCode::SDKReturnCode_Success;
}

bool HandSkeletonSetup::setupRightHand(uint32_t& skeletonId) {
    uint32_t setupIndex = 0;
    
    if (CoreSdk_CreateSkeletonSetup(&setupIndex) != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    if (!addHandNodes(setupIndex)) {
        return false;
    }
    
    if (!addHandChains(setupIndex, Side::Side_Right)) {
        return false;
    }
    
    return CoreSdk_LoadSkeleton(setupIndex, &skeletonId) == SDKReturnCode::SDKReturnCode_Success;
}

bool HandSkeletonSetup::addHandNodes(uint32_t setupIndex) {
    // Hand skeleton with 21 nodes:
    // Node 0: Wrist
    // Nodes 1-4: Index finger (metacarpal, proximal, intermediate, distal) - TIP = 4
    // Nodes 5-8: Middle finger - TIP = 8  
    // Nodes 9-12: Ring finger - TIP = 12
    // Nodes 13-16: Pinky finger - TIP = 16
    // Nodes 17-20: Thumb - TIP = 20
    
    // Wrist (root)
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 0, 0.0f, 0.0f, 0.0f, NodeType::NodeType_Joint) 
        != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    // Index finger chain: 1->2->3->4 (tip)
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 1, 0.095f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 2, 0.04f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 3, 0.025f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 4, 0.02f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false; // INDEX TIP
    
    // Middle finger chain: 5->6->7->8 (tip)
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 5, 0.095f, 0.0f, 0.03f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 6, 0.045f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 7, 0.025f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 8, 0.022f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false; // MIDDLE TIP
    
    // Ring finger chain: 9->10->11->12 (tip)
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 9, 0.095f, 0.0f, -0.03f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 10, 0.04f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 11, 0.025f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 12, 0.02f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false; // RING TIP
    
    // Pinky finger chain: 13->14->15->16 (tip)
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 13, 0.095f, 0.0f, -0.055f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 14, 0.032f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 15, 0.02f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 16, 0.018f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false; // PINKY TIP
    
    // Thumb chain: 17->18->19->20 (tip)
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 17, 0.025f, 0.0f, 0.055f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 18, 0.035f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 19, 0.03f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false;
    if (CoreSdk_AddNodeToSkeletonSetup(setupIndex, 20, 0.025f, 0.0f, 0.0f, NodeType::NodeType_Joint) != SDKReturnCode::SDKReturnCode_Success) return false; // THUMB TIP
    
    return true;
}

bool HandSkeletonSetup::addHandChains(uint32_t setupIndex, Side handSide) {
    // Chain 0: Hand (wrist only)
    if (CoreSdk_AddChainToSkeletonSetup(setupIndex, 0, 0, 0, ChainType::ChainType_Hand, handSide) 
        != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    // Chain 1: Index finger (nodes 1-4)
    if (CoreSdk_AddChainToSkeletonSetup(setupIndex, 1, 0, 1, ChainType::ChainType_FingerIndex, handSide) 
        != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    // Chain 2: Middle finger (nodes 5-8)
    if (CoreSdk_AddChainToSkeletonSetup(setupIndex, 2, 0, 5, ChainType::ChainType_FingerMiddle, handSide) 
        != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    // Chain 3: Ring finger (nodes 9-12)
    if (CoreSdk_AddChainToSkeletonSetup(setupIndex, 3, 0, 9, ChainType::ChainType_FingerRing, handSide) 
        != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    // Chain 4: Pinky finger (nodes 13-16)
    if (CoreSdk_AddChainToSkeletonSetup(setupIndex, 4, 0, 13, ChainType::ChainType_FingerPinky, handSide) 
        != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    // Chain 5: Thumb (nodes 17-20)
    if (CoreSdk_AddChainToSkeletonSetup(setupIndex, 5, 0, 17, ChainType::ChainType_FingerThumb, handSide) 
        != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    return true;
}

bool HandSkeletonSetup::extractFingerTips(uint32_t skeletonId, float* fingerTips, size_t maxNodes) {
    if (maxNodes < 21) {
        return false; // Need at least 21 nodes
    }
    
    std::vector<SkeletonNode> nodes(maxNodes);
    if (CoreSdk_GetSkeletonData(skeletonId, nodes.data(), maxNodes) != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }
    
    // Extract fingertip positions
    // Order: Index=0, Middle=1, Ring=2, Pinky=3, Thumb=4 (15 floats total)
    
    // Index tip (node 4)
    fingerTips[0] = nodes[4].position.x;
    fingerTips[1] = nodes[4].position.y;
    fingerTips[2] = nodes[4].position.z;
    
    // Middle tip (node 8)
    fingerTips[3] = nodes[8].position.x;
    fingerTips[4] = nodes[8].position.y;
    fingerTips[5] = nodes[8].position.z;
    
    // Ring tip (node 12)
    fingerTips[6] = nodes[12].position.x;
    fingerTips[7] = nodes[12].position.y;
    fingerTips[8] = nodes[12].position.z;
    
    // Pinky tip (node 16)
    fingerTips[9] = nodes[16].position.x;
    fingerTips[10] = nodes[16].position.y;
    fingerTips[11] = nodes[16].position.z;
    
    // Thumb tip (node 20)
    fingerTips[12] = nodes[20].position.x;
    fingerTips[13] = nodes[20].position.y;
    fingerTips[14] = nodes[20].position.z;
    
    return true;
}

} // namespace manus_skeleton