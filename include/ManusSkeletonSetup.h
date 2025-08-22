// File: include/ManusSkeletonSetup.h
#pragma once

#include <ManusSDK.h>
#include <vector>
#include <string>
#include <memory>
#include <array>

enum class HandSide;

class ManusSkeletonSetup {
public:
    ManusSkeletonSetup() = default;
    ~ManusSkeletonSetup();

    // Main setup interface
    bool CreateSkeletons(HandSide side);
    void CleanupSkeletons();

    // Skeleton management
    std::vector<uint32_t> GetSkeletonIds() const { return skeleton_ids_; }
    bool IsSkeletonValid(uint32_t skeleton_id) const;

    // Node management - ensure fingertip nodes exist
    bool EnsureFingertipNodes(uint32_t skeleton_id);
    bool ValidateSkeletonStructure(uint32_t skeleton_id) const;

    // NEW: Frame mapping for Hand IK integration
    struct FrameMapping {
        std::array<uint32_t, 4> fingertip_node_ids;  // Index, Middle, Ring, Pinky
        uint32_t thumb_tip_node_id;
        std::array<uint32_t, 4> mcp_node_ids;        // MCP joint nodes
        std::array<uint32_t, 4> distal_node_ids;     // Distal joint nodes
        uint32_t thumb_rot_node_id;
        uint32_t thumb_flex_node_id;
        bool valid = false;
    };

    // Get frame mapping for a skeleton
    FrameMapping GetFrameMapping(uint32_t skeleton_id) const;

    // NEW: Enhanced node creation with proper tip frames
    bool CreateFingertipFramesForHandIK(uint32_t skeleton_id);

    // NEW: Validate that all required frames exist for Hand IK
    bool ValidateHandIKFrames(uint32_t skeleton_id) const;

private:
    std::vector<uint32_t> skeleton_ids_;

    // Skeleton creation helpers
    bool CreateHandSkeleton(bool is_left_hand, uint32_t& skeleton_id);
    bool SetupHandNodes(uint32_t skeleton_id, bool is_left_hand);
    bool AddFingertipNode(uint32_t skeleton_id, const std::string& finger_name,
        uint32_t parent_node_id, const ManusVec3& offset);

    // Node utilities
    uint32_t FindNodeByName(uint32_t skeleton_id, const std::string& node_name) const;
    std::vector<std::string> GetRequiredFingertipNodes(bool is_left_hand) const;
    ManusVec3 EstimateFingertipOffset(const std::string& finger_name) const;
    ManusVec3 EstimateFingerBasePosition(const std::string& finger, bool is_left_hand) const;

    // NEW: Enhanced frame creation for Hand IK compatibility
    bool CreateNodeWithExactName(uint32_t skeleton_id, const std::string& node_name,
        uint32_t parent_id, const ManusVec3& position);
    std::vector<std::string> GetHandIKRequiredFrameNames() const;

    // NEW: Plane computation helpers (for non-degenerate axes)
    ManusVec3 ComputeFingerFlexionAxis(uint32_t skeleton_id, int finger_index) const;
    bool ValidateFingerPlaneGeometry(uint32_t skeleton_id) const;

    // Cleanup helpers
    void CleanupSkeleton(uint32_t skeleton_id);
};