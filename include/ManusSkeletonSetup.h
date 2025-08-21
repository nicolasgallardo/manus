// File: include/ManusSkeletonSetup.h
#pragma once

#include <ManusSDK.h>
#include <vector>
#include <string>
#include <memory>

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

    // Cleanup helpers
    void CleanupSkeleton(uint32_t skeleton_id);
};