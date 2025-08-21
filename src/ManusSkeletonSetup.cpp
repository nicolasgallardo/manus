// File: src/ManusSkeletonSetup.cpp (REPLACE your existing manusskeletonsetup.cpp)
#include "ManusSkeletonSetup.h"
#include "ManusHandIKBridge.h"
#include <iostream>
#include <algorithm>
#include <cstring>

ManusSkeletonSetup::~ManusSkeletonSetup() {
    CleanupSkeletons();
}

bool ManusSkeletonSetup::CreateSkeletons(HandSide side) {
    CleanupSkeletons(); // Clean up any existing skeletons

    bool success = true;

    if (side == HandSide::Left || side == HandSide::Both) {
        uint32_t left_skeleton_id;
        if (CreateHandSkeleton(true, left_skeleton_id)) {
            skeleton_ids_.push_back(left_skeleton_id);
            std::cout << "✓ Created left hand skeleton (ID: " << left_skeleton_id << ")" << std::endl;
        }
        else {
            std::cerr << "❌ Failed to create left hand skeleton" << std::endl;
            success = false;
        }
    }

    if (side == HandSide::Right || side == HandSide::Both) {
        uint32_t right_skeleton_id;
        if (CreateHandSkeleton(false, right_skeleton_id)) {
            skeleton_ids_.push_back(right_skeleton_id);
            std::cout << "✓ Created right hand skeleton (ID: " << right_skeleton_id << ")" << std::endl;
        }
        else {
            std::cerr << "❌ Failed to create right hand skeleton" << std::endl;
            success = false;
        }
    }

    // Validate all created skeletons
    for (uint32_t skeleton_id : skeleton_ids_) {
        if (!ValidateSkeletonStructure(skeleton_id)) {
            std::cerr << "⚠️  Warning: Skeleton " << skeleton_id << " structure validation failed" << std::endl;
        }

        if (!EnsureFingertipNodes(skeleton_id)) {
            std::cerr << "⚠️  Warning: Failed to ensure fingertip nodes for skeleton " << skeleton_id << std::endl;
        }
    }

    return success && !skeleton_ids_.empty();
}

bool ManusSkeletonSetup::CreateHandSkeleton(bool is_left_hand, uint32_t& skeleton_id) {
    // Create skeleton setup information
    SkeletonSetupInfo skeleton_setup = {};

    // Set skeleton name
    std::string skeleton_name = is_left_hand ? "left_hand" : "right_hand";
    strncpy(skeleton_setup.name, skeleton_name.c_str(), sizeof(skeleton_setup.name) - 1);
    skeleton_setup.name[sizeof(skeleton_setup.name) - 1] = '\0';

    // Set skeleton type to hand
    skeleton_setup.type = SkeletonType::SkeletonType_Hand;
    skeleton_setup.side = is_left_hand ? Side::Side_Left : Side::Side_Right;

    // Create the skeleton
    SDKReturnCode result = CoreSdk_CreateSkeleton(&skeleton_setup, &skeleton_id);
    if (result != SDKReturnCode::SDKReturnCode_Success) {
        std::cerr << "Failed to create " << skeleton_name << " skeleton: " << static_cast<int>(result) << std::endl;
        return false;
    }

    // Set up the hand nodes
    if (!SetupHandNodes(skeleton_id, is_left_hand)) {
        std::cerr << "Failed to setup nodes for " << skeleton_name << " skeleton" << std::endl;
        CleanupSkeleton(skeleton_id);
        return false;
    }

    return true;
}

bool ManusSkeletonSetup::SetupHandNodes(uint32_t skeleton_id, bool is_left_hand) {
    // Define the basic hand node structure
    std::vector<NodeSetupInfo> hand_nodes;

    // Root node (wrist)
    NodeSetupInfo wrist_node = {};
    strncpy(wrist_node.name, "wrist", sizeof(wrist_node.name) - 1);
    wrist_node.type = NodeType::NodeType_Joint;
    wrist_node.parentID = 0; // Root node
    wrist_node.transform.position = { 0.0f, 0.0f, 0.0f };
    wrist_node.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f }; // Identity quaternion
    hand_nodes.push_back(wrist_node);

    // Define finger structure: MCP -> PIP -> DIP -> Tip
    std::vector<std::string> finger_names = { "index", "middle", "ring", "pinky" };
    std::vector<std::string> joint_types = { "mcp", "pip", "dip" };

    uint32_t node_id = 1; // Start after wrist

    for (const auto& finger : finger_names) {
        uint32_t finger_root_id = node_id;
        uint32_t parent_id = 0; // Connect to wrist

        // Estimate finger base positions
        ManusVec3 finger_base_pos = EstimateFingerBasePosition(finger, is_left_hand);

        for (size_t joint_idx = 0; joint_idx < joint_types.size(); ++joint_idx) {
            NodeSetupInfo joint_node = {};
            std::string joint_name = finger + "_" + joint_types[joint_idx];
            strncpy(joint_node.name, joint_name.c_str(), sizeof(joint_node.name) - 1);
            joint_node.type = NodeType::NodeType_Joint;
            joint_node.parentID = parent_id;

            // Position joints along finger
            float joint_offset = (joint_idx + 1) * 0.025f; // 2.5cm between joints
            joint_node.transform.position = {
                finger_base_pos.x,
                finger_base_pos.y + joint_offset,
                finger_base_pos.z
            };
            joint_node.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };

            hand_nodes.push_back(joint_node);
            parent_id = node_id++;
        }
    }

    // Add thumb structure (simplified)
    NodeSetupInfo thumb_mcp = {};
    strncpy(thumb_mcp.name, "thumb_mcp", sizeof(thumb_mcp.name) - 1);
    thumb_mcp.type = NodeType::NodeType_Joint;
    thumb_mcp.parentID = 0; // Connect to wrist
    ManusVec3 thumb_pos = EstimateFingerBasePosition("thumb", is_left_hand);
    thumb_mcp.transform.position = thumb_pos;
    thumb_mcp.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    hand_nodes.push_back(thumb_mcp);
    uint32_t thumb_parent_id = node_id++;

    NodeSetupInfo thumb_ip = {};
    strncpy(thumb_ip.name, "thumb_ip", sizeof(thumb_ip.name) - 1);
    thumb_ip.type = NodeType::NodeType_Joint;
    thumb_ip.parentID = thumb_parent_id;
    thumb_ip.transform.position = { thumb_pos.x, thumb_pos.y + 0.03f, thumb_pos.z };
    thumb_ip.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    hand_nodes.push_back(thumb_ip);

    // Create all nodes in the skeleton
    for (const auto& node : hand_nodes) {
        uint32_t created_node_id;
        SDKReturnCode result = CoreSdk_AddNodeToSkeleton(skeleton_id, &node, &created_node_id);
        if (result != SDKReturnCode::SDKReturnCode_Success) {
            std::cerr << "Failed to add node " << node.name << " to skeleton: "
                << static_cast<int>(result) << std::endl;
            return false;
        }
    }

    return true;
}

ManusVec3 ManusSkeletonSetup::EstimateFingerBasePosition(const std::string& finger, bool is_left_hand) const {
    // Estimate finger base positions relative to wrist
    float hand_width = 0.08f; // 8cm hand width
    float side_multiplier = is_left_hand ? -1.0f : 1.0f;

    if (finger == "index") {
        return { side_multiplier * 0.025f, 0.08f, 0.0f };
    }
    else if (finger == "middle") {
        return { side_multiplier * 0.008f, 0.085f, 0.0f };
    }
    else if (finger == "ring") {
        return { side_multiplier * -0.008f, 0.08f, 0.0f };
    }
    else if (finger == "pinky") {
        return { side_multiplier * -0.025f, 0.07f, 0.0f };
    }
    else if (finger == "thumb") {
        return { side_multiplier * 0.04f, 0.02f, 0.01f };
    }

    return { 0.0f, 0.0f, 0.0f }; // Default
}

bool ManusSkeletonSetup::EnsureFingertipNodes(uint32_t skeleton_id) {
    std::vector<std::string> required_tips = GetRequiredFingertipNodes(false); // Assume right hand for now

    bool all_tips_exist = true;

    for (const auto& tip_name : required_tips) {
        uint32_t tip_node_id = FindNodeByName(skeleton_id, tip_name);

        if (tip_node_id == 0) {
            // Tip node doesn't exist, try to create it
            std::string finger_name = tip_name.substr(0, tip_name.find("_tip"));
            std::string parent_name = finger_name + "_dip"; // Attach to DIP joint

            uint32_t parent_id = FindNodeByName(skeleton_id, parent_name);
            if (parent_id == 0) {
                std::cerr << "Cannot find parent node " << parent_name << " for tip " << tip_name << std::endl;
                all_tips_exist = false;
                continue;
            }

            ManusVec3 tip_offset = EstimateFingertipOffset(finger_name);
            if (!AddFingertipNode(skeleton_id, tip_name, parent_id, tip_offset)) {
                std::cerr << "Failed to add fingertip node " << tip_name << std::endl;
                all_tips_exist = false;
            }
            else {
                std::cout << "✓ Added missing fingertip node: " << tip_name << std::endl;
            }
        }
    }

    return all_tips_exist;
}

bool ManusSkeletonSetup::AddFingertipNode(uint32_t skeleton_id, const std::string& finger_name,
    uint32_t parent_node_id, const ManusVec3& offset) {
    NodeSetupInfo tip_node = {};
    strncpy(tip_node.name, finger_name.c_str(), sizeof(tip_node.name) - 1);
    tip_node.name[sizeof(tip_node.name) - 1] = '\0';
    tip_node.type = NodeType::NodeType_Joint;
    tip_node.parentID = parent_node_id;
    tip_node.transform.position = offset;
    tip_node.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f }; // Identity quaternion

    uint32_t created_node_id;
    SDKReturnCode result = CoreSdk_AddNodeToSkeleton(skeleton_id, &tip_node, &created_node_id);

    return result == SDKReturnCode::SDKReturnCode_Success;
}

uint32_t ManusSkeletonSetup::FindNodeByName(uint32_t skeleton_id, const std::string& node_name) const {
    SkeletonStreamInfo skeleton_info;
    SDKReturnCode result = CoreSdk_GetSkeletonStreamData(skeleton_id, &skeleton_info);

    if (result != SDKReturnCode::SDKReturnCode_Success) {
        return 0;
    }

    for (uint32_t i = 0; i < skeleton_info.nodesCount; ++i) {
        if (std::string(skeleton_info.nodes[i].id.name) == node_name) {
            return skeleton_info.nodes[i].id.id;
        }
    }

    return 0; // Not found
}

std::vector<std::string> ManusSkeletonSetup::GetRequiredFingertipNodes(bool is_left_hand) const {
    return {
        "index_tip",
        "middle_tip",
        "ring_tip",
        "pinky_tip",
        "thumb_tip"
    };
}

ManusVec3 ManusSkeletonSetup::EstimateFingertipOffset(const std::string& finger_name) const {
    // Estimate fingertip offset from DIP joint
    if (finger_name == "thumb") {
        return { 0.0f, 0.025f, 0.0f }; // 2.5cm for thumb
    }
    else {
        return { 0.0f, 0.02f, 0.0f };  // 2cm for fingers
    }
}

bool ManusSkeletonSetup::ValidateSkeletonStructure(uint32_t skeleton_id) const {
    SkeletonStreamInfo skeleton_info;
    SDKReturnCode result = CoreSdk_GetSkeletonStreamData(skeleton_id, &skeleton_info);

    if (result != SDKReturnCode::SDKReturnCode_Success) {
        return false;
    }

    // Check minimum required nodes
    std::vector<std::string> required_nodes = {
        "wrist", "index_mcp", "middle_mcp", "ring_mcp", "pinky_mcp", "thumb_mcp"
    };

    for (const auto& required_node : required_nodes) {
        bool found = false;
        for (uint32_t i = 0; i < skeleton_info.nodesCount; ++i) {
            if (std::string(skeleton_info.nodes[i].id.name) == required_node) {
                found = true;
                break;
            }
        }

        if (!found) {
            std::cerr << "Required node " << required_node << " not found in skeleton " << skeleton_id << std::endl;
            return false;
        }
    }

    return true;
}

bool ManusSkeletonSetup::IsSkeletonValid(uint32_t skeleton_id) const {
    auto it = std::find(skeleton_ids_.begin(), skeleton_ids_.end(), skeleton_id);
    return it != skeleton_ids_.end();
}

void ManusSkeletonSetup::CleanupSkeletons() {
    for (uint32_t skeleton_id : skeleton_ids_) {
        CleanupSkeleton(skeleton_id);
    }
    skeleton_ids_.clear();
}

void ManusSkeletonSetup::CleanupSkeleton(uint32_t skeleton_id) {
    SDKReturnCode result = CoreSdk_DestroyTemplate(skeleton_id);
    if (result != SDKReturnCode::SDKReturnCode_Success) {
        std::cerr << "Warning: Failed to cleanup skeleton " << skeleton_id
            << ": " << static_cast<int>(result) << std::endl;
    }
}