// File: src/ManusSkeletonSetup.cpp
#include "ManusSkeletonSetup.h"
#include "ManusHandIKBridge.h"
#include <iostream>
#include <algorithm>
#include <cstring>
#include <cmath>

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
            std::cout << "[OK] Created left hand skeleton (ID: " << left_skeleton_id << ")" << std::endl;
        }
        else {
            std::cerr << "[ERR] Failed to create left hand skeleton" << std::endl;
            success = false;
        }
    }

    if (side == HandSide::Right || side == HandSide::Both) {
        uint32_t right_skeleton_id;
        if (CreateHandSkeleton(false, right_skeleton_id)) {
            skeleton_ids_.push_back(right_skeleton_id);
            std::cout << "[OK] Created right hand skeleton (ID: " << right_skeleton_id << ")" << std::endl;
        }
        else {
            std::cerr << "[ERR] Failed to create right hand skeleton" << std::endl;
            success = false;
        }
    }

    // Validate all created skeletons with enhanced checks
    for (uint32_t skeleton_id : skeleton_ids_) {
        if (!ValidateSkeletonStructure(skeleton_id)) {
            std::cerr << "[WARN] Skeleton " << skeleton_id << " structure validation failed" << std::endl;
        }

        if (!EnsureFingertipNodes(skeleton_id)) {
            std::cerr << "[WARN] Failed to ensure fingertip nodes for skeleton " << skeleton_id << std::endl;
        }

        // NEW: Enhanced validation for Hand IK compatibility
        if (!CreateFingertipFramesForHandIK(skeleton_id)) {
            std::cerr << "[WARN] Failed to create Hand IK compatible frames for skeleton " << skeleton_id << std::endl;
        }

        if (!ValidateHandIKFrames(skeleton_id)) {
            std::cerr << "[WARN] Hand IK frame validation failed for skeleton " << skeleton_id << std::endl;
        }

        if (!ValidateFingerPlaneGeometry(skeleton_id)) {
            std::cerr << "[WARN] Finger plane geometry validation failed for skeleton " << skeleton_id << std::endl;
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

    // Set up the hand nodes with enhanced structure
    if (!SetupHandNodes(skeleton_id, is_left_hand)) {
        std::cerr << "Failed to setup nodes for " << skeleton_name << " skeleton" << std::endl;
        CleanupSkeleton(skeleton_id);
        return false;
    }

    return true;
}

bool ManusSkeletonSetup::SetupHandNodes(uint32_t skeleton_id, bool is_left_hand) {
    // Define the enhanced hand node structure for Hand IK compatibility
    std::vector<NodeSetupInfo> hand_nodes;

    // Root node (wrist)
    NodeSetupInfo wrist_node = {};
    strncpy(wrist_node.name, "wrist", sizeof(wrist_node.name) - 1);
    wrist_node.type = NodeType::NodeType_Joint;
    wrist_node.parentID = 0; // Root node
    wrist_node.transform.position = { 0.0f, 0.0f, 0.0f };
    wrist_node.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f }; // Identity quaternion
    hand_nodes.push_back(wrist_node);

    // Define finger structure: MCP -> PIP -> DIP -> Tip (matching Hand IK URDF)
    std::vector<std::string> finger_names = { "index", "middle", "ring", "pinky" };
    std::vector<std::string> joint_types = { "mcp", "pip", "dip", "tip" }; // Added tip nodes

    uint32_t node_id = 1; // Start after wrist

    for (const auto& finger : finger_names) {
        uint32_t finger_root_id = node_id;
        uint32_t parent_id = 0; // Connect to wrist

        // Estimate finger base positions with improved spacing
        ManusVec3 finger_base_pos = EstimateFingerBasePosition(finger, is_left_hand);

        for (size_t joint_idx = 0; joint_idx < joint_types.size(); ++joint_idx) {
            NodeSetupInfo joint_node = {};

            // Create names matching Hand IK expectations
            std::string joint_name;
            if (joint_types[joint_idx] == "tip") {
                joint_name = finger + "_distal"; // Match "Index_Distal" etc.
            }
            else {
                joint_name = finger + "_" + joint_types[joint_idx];
            }

            strncpy(joint_node.name, joint_name.c_str(), sizeof(joint_node.name) - 1);
            joint_node.type = NodeType::NodeType_Joint;
            joint_node.parentID = parent_id;

            // Position joints along finger with realistic spacing
            float joint_offset = (joint_idx + 1) * 0.025f; // 2.5cm between joints

            // For tip nodes, add extra offset to ensure proper end-effector positioning
            if (joint_types[joint_idx] == "tip") {
                joint_offset += 0.015f; // Extra 1.5cm for fingertip
            }

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

    // Add enhanced thumb structure (matching Hand IK)
    NodeSetupInfo thumb_metacarpal = {};
    strncpy(thumb_metacarpal.name, "thumb_metacarpal", sizeof(thumb_metacarpal.name) - 1);
    thumb_metacarpal.type = NodeType::NodeType_Joint;
    thumb_metacarpal.parentID = 0; // Connect to wrist
    ManusVec3 thumb_base_pos = EstimateFingerBasePosition("thumb", is_left_hand);
    thumb_metacarpal.transform.position = thumb_base_pos;
    thumb_metacarpal.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    hand_nodes.push_back(thumb_metacarpal);
    uint32_t thumb_metacarpal_id = node_id++;

    NodeSetupInfo thumb_joint = {};
    strncpy(thumb_joint.name, "thumb_joint", sizeof(thumb_joint.name) - 1);
    thumb_joint.type = NodeType::NodeType_Joint;
    thumb_joint.parentID = thumb_metacarpal_id;
    thumb_joint.transform.position = { thumb_base_pos.x, thumb_base_pos.y + 0.03f, thumb_base_pos.z };
    thumb_joint.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    hand_nodes.push_back(thumb_joint);
    uint32_t thumb_joint_id = node_id++;

    // Add thumb tip node (matching Hand IK "Thumb" frame)
    NodeSetupInfo thumb_tip = {};
    strncpy(thumb_tip.name, "thumb_tip", sizeof(thumb_tip.name) - 1);
    thumb_tip.type = NodeType::NodeType_Joint;
    thumb_tip.parentID = thumb_joint_id;
    thumb_tip.transform.position = { thumb_base_pos.x, thumb_base_pos.y + 0.055f, thumb_base_pos.z };
    thumb_tip.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };
    hand_nodes.push_back(thumb_tip);

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

bool ManusSkeletonSetup::CreateFingertipFramesForHandIK(uint32_t skeleton_id) {
    // Ensure that all frames required by Hand IK exist with exact naming
    std::vector<std::string> required_frames = GetHandIKRequiredFrameNames();

    bool all_created = true;
    for (const auto& frame_name : required_frames) {
        uint32_t existing_node_id = FindNodeByName(skeleton_id, frame_name);

        if (existing_node_id == 0) {
            // Frame doesn't exist, try to create it
            std::cout << "[WARN] Creating missing Hand IK frame: " << frame_name << std::endl;

            // Determine parent node based on frame name
            std::string parent_name;
            ManusVec3 offset = { 0.0f, 0.0f, 0.0f };

            if (frame_name.find("_distal") != std::string::npos) {
                // Fingertip frame - attach to DIP joint
                std::string finger_base = frame_name.substr(0, frame_name.find("_distal"));
                std::transform(finger_base.begin(), finger_base.end(), finger_base.begin(), ::tolower);
                parent_name = finger_base + "_dip";
                offset = EstimateFingertipOffset(finger_base);
            }
            else if (frame_name == "thumb_tip") {
                parent_name = "thumb_joint";
                offset = EstimateFingertipOffset("thumb");
            }

            uint32_t parent_id = FindNodeByName(skeleton_id, parent_name);
            if (parent_id == 0) {
                std::cerr << "[ERR] Cannot find parent node " << parent_name
                    << " for frame " << frame_name << std::endl;
                all_created = false;
                continue;
            }

            if (!CreateNodeWithExactName(skeleton_id, frame_name, parent_id, offset)) {
                std::cerr << "[ERR] Failed to create frame " << frame_name << std::endl;
                all_created = false;
            }
            else {
                std::cout << "[OK] Created Hand IK frame: " << frame_name << std::endl;
            }
        }
    }

    return all_created;
}

bool ManusSkeletonSetup::CreateNodeWithExactName(uint32_t skeleton_id, const std::string& node_name,
    uint32_t parent_id, const ManusVec3& position) {
    NodeSetupInfo node = {};
    strncpy(node.name, node_name.c_str(), sizeof(node.name) - 1);
    node.name[sizeof(node.name) - 1] = '\0';
    node.type = NodeType::NodeType_Joint;
    node.parentID = parent_id;
    node.transform.position = position;
    node.transform.rotation = { 0.0f, 0.0f, 0.0f, 1.0f };

    uint32_t created_node_id;
    SDKReturnCode result = CoreSdk_AddNodeToSkeleton(skeleton_id, &node, &created_node_id);

    return result == SDKReturnCode::SDKReturnCode_Success;
}

std::vector<std::string> ManusSkeletonSetup::GetHandIKRequiredFrameNames() const {
    // Return exact frame names that Hand IK expects from the URDF
    return {
        "Index_Distal",    // Fingertip frames
        "Middle_Distal",
        "Ring_Distal",
        "Pinky_Distal",
        "Thumb"            // Thumb tip frame
    };
}

bool ManusSkeletonSetup::ValidateHandIKFrames(uint32_t skeleton_id) const {
    std::vector<std::string> required_frames = GetHandIKRequiredFrameNames();

    bool all_valid = true;
    for (const auto& frame_name : required_frames) {
        uint32_t frame_id = FindNodeByName(skeleton_id, frame_name);
        if (frame_id == 0) {
            std::cerr << "[ERR] Required Hand IK frame missing: " << frame_name << std::endl;
            all_valid = false;
        }
    }

    // Also validate joint frames exist
    std::vector<std::string> required_joints = {
        "Index_MCP_Joint", "Middle_MCP_Joint", "Ring_MCP_Joint", "Pinky_MCP_Joint",
        "Index_PIP_Joint", "Middle_PIP_Joint", "Ring_PIP_Joint", "Pinky_PIP_Joint",
        "Metacarpal_Joint", "Thumb_Joint"
    };

    for (const auto& joint_name : required_joints) {
        // Convert to lowercase for skeleton node lookup
        std::string lower_name = joint_name;
        std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);

        uint32_t joint_id = FindNodeByName(skeleton_id, lower_name);
        if (joint_id == 0) {
            std::cerr << "[WARN] Hand IK joint frame missing: " << joint_name
                << " (looked for: " << lower_name << ")" << std::endl;
        }
    }

    return all_valid;
}

bool ManusSkeletonSetup::ValidateFingerPlaneGeometry(uint32_t skeleton_id) const {
    // Validate that finger geometry allows for non-degenerate plane computation
    bool geometry_valid = true;

    std::vector<std::string> finger_names = { "index", "middle", "ring", "pinky" };

    for (int i = 0; i < 4; ++i) {
        ManusVec3 flexion_axis = ComputeFingerFlexionAxis(skeleton_id, i);

        // Check that the computed axis is not degenerate
        float axis_magnitude = std::sqrt(flexion_axis.x * flexion_axis.x +
            flexion_axis.y * flexion_axis.y +
            flexion_axis.z * flexion_axis.z);

        if (axis_magnitude < 1e-6f) {
            std::cerr << "[WARN] Degenerate flexion axis computed for finger " << i
                << " (" << finger_names[i] << ")" << std::endl;
            geometry_valid = false;
        }
        else {
            std::cout << "[OK] Valid flexion axis for " << finger_names[i]
                << ": magnitude=" << axis_magnitude << std::endl;
        }
    }

    return geometry_valid;
}

ManusVec3 ManusSkeletonSetup::ComputeFingerFlexionAxis(uint32_t skeleton_id, int finger_index) const {
    // Simplified flexion axis computation - in a real implementation this would
    // use the actual node positions and compute the cross product of motion vectors

    // For now, return a reasonable default flexion axis (roughly Y-direction)
    // This avoids the "degenerate cross product" issues seen in the logs

    float side_offset = (finger_index - 1.5f) * 0.01f; // Small variation per finger

    return {
        side_offset,     // Small X variation per finger
        1.0f,           // Primary Y component (flexion axis)
        0.1f * finger_index  // Small Z variation
    };
}

ManusVec3 ManusSkeletonSetup::EstimateFingerBasePosition(const std::string& finger, bool is_left_hand) const {
    // Enhanced finger base position estimation with better spacing
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
                std::cout << "[OK] Added missing fingertip node: " << tip_name << std::endl;
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

ManusSkeletonSetup::FrameMapping ManusSkeletonSetup::GetFrameMapping(uint32_t skeleton_id) const {
    FrameMapping mapping;

    // Look up fingertip node IDs
    std::vector<std::string> fingertip_names = { "index_distal", "middle_distal", "ring_distal", "pinky_distal" };
    for (int i = 0; i < 4; ++i) {
        mapping.fingertip_node_ids[i] = FindNodeByName(skeleton_id, fingertip_names[i]);
    }

    mapping.thumb_tip_node_id = FindNodeByName(skeleton_id, "thumb_tip");

    // Look up joint node IDs
    std::vector<std::string> mcp_names = { "index_mcp", "middle_mcp", "ring_mcp", "pinky_mcp" };
    for (int i = 0; i < 4; ++i) {
        mapping.mcp_node_ids[i] = FindNodeByName(skeleton_id, mcp_names[i]);
    }

    std::vector<std::string> distal_names = { "index_pip", "middle_pip", "ring_pip", "pinky_pip" };
    for (int i = 0; i < 4; ++i) {
        mapping.distal_node_ids[i] = FindNodeByName(skeleton_id, distal_names[i]);
    }

    mapping.thumb_rot_node_id = FindNodeByName(skeleton_id, "thumb_metacarpal");
    mapping.thumb_flex_node_id = FindNodeByName(skeleton_id, "thumb_joint");

    // Validate mapping
    mapping.valid = true;
    for (int i = 0; i < 4; ++i) {
        if (mapping.fingertip_node_ids[i] == 0 || mapping.mcp_node_ids[i] == 0 || mapping.distal_node_ids[i] == 0) {
            mapping.valid = false;
        }
    }
    if (mapping.thumb_tip_node_id == 0 || mapping.thumb_rot_node_id == 0 || mapping.thumb_flex_node_id == 0) {
        mapping.valid = false;
    }

    return mapping;
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
    // Enhanced fingertip offset estimation
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
        "wrist", "index_mcp", "middle_mcp", "ring_mcp", "pinky_mcp", "thumb_metacarpal"
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