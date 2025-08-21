// SDKClientIntegration.cpp - Complete Manus Core + Hand IK integration
#include "ManusHandIKBridge.h"
#include "ManusSkeletonSetup.h"
#include <ManusSDK.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <filesystem>

// Global state for integration
class ManusIntegration {
private:
    std::unique_ptr<ManusHandIKBridge> ik_bridge_;
    std::unique_ptr<ManusSkeletonSetup> skeleton_setup_;
    std::atomic<bool> running_{ false };
    std::atomic<bool> connected_{ false };

    // Configuration
    std::string manus_host_ = "127.0.0.1";
    uint16_t manus_port_ = 9004;
    bool simulate_mode_ = false;
    bool debug_logging_ = false;
    HandSide active_side_ = HandSide::Both;

    // Performance tracking
    std::chrono::steady_clock::time_point last_stats_time_;
    uint64_t frame_count_ = 0;
    uint64_t ik_solve_count_ = 0;
    double total_solve_time_ms_ = 0.0;

public:
    ManusIntegration() {
        last_stats_time_ = std::chrono::steady_clock::now();
    }

    bool Initialize(int argc, char* argv[]) {
        // Parse command line arguments
        ParseCommandLine(argc, argv);

        // Print startup banner
        PrintStartupBanner();

        // Initialize Hand IK bridge
        try {
            std::string urdf_path = ResolveUrdfPath(argc, argv);
            ik_bridge_ = std::make_unique<ManusHandIKBridge>(urdf_path);
            std::cout << "✓ Hand IK engine initialized with URDF: " << urdf_path << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "❌ Failed to initialize Hand IK: " << e.what() << std::endl;
            return false;
        }

        // Initialize Manus SDK
        if (!InitializeManusSDK()) {
            return false;
        }

        // Set up skeletons
        try {
            skeleton_setup_ = std::make_unique<ManusSkeletonSetup>();
            if (!skeleton_setup_->CreateSkeletons(active_side_)) {
                std::cerr << "❌ Failed to create Manus skeletons" << std::endl;
                return false;
            }
            std::cout << "✓ Manus skeletons created successfully" << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "❌ Skeleton setup failed: " << e.what() << std::endl;
            return false;
        }

        return true;
    }

    void Run() {
        running_ = true;
        std::cout << "\n🚀 Starting real-time Hand IK streaming..." << std::endl;
        std::cout << "Press Ctrl+C to stop.\n" << std::endl;

        // Main processing loop
        while (running_) {
            ProcessFrame();

            // Print performance stats every 5 seconds
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_stats_time_);
            if (elapsed.count() >= 5) {
                PrintPerformanceStats();
                last_stats_time_ = now;
            }

            // Small sleep to prevent CPU hammering
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        std::cout << "\n🛑 Stopping Hand IK streaming..." << std::endl;
    }

    void Shutdown() {
        running_ = false;

        if (skeleton_setup_) {
            skeleton_setup_->CleanupSkeletons();
        }

        if (connected_) {
            SDKReturnCode result = CoreSdk_ShutDown();
            if (result == SDKReturnCode::SDKReturnCode_Success) {
                std::cout << "✓ Disconnected from Manus Core" << std::endl;
            }
        }

        std::cout << "✓ Integration shutdown complete" << std::endl;
    }

private:
    void ParseCommandLine(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            std::string arg(argv[i]);

            if (arg == "--simulate") {
                simulate_mode_ = true;
            }
            else if (arg == "--debug" || arg == "--log=debug") {
                debug_logging_ = true;
            }
            else if (arg == "--side=left") {
                active_side_ = HandSide::Left;
            }
            else if (arg == "--side=right") {
                active_side_ = HandSide::Right;
            }
            else if (arg == "--side=both") {
                active_side_ = HandSide::Both;
            }
            else if (arg.find("--host=") == 0) {
                manus_host_ = arg.substr(7);
            }
            else if (arg.find("--port=") == 0) {
                manus_port_ = static_cast<uint16_t>(std::stoi(arg.substr(7)));
            }
        }
    }

    void PrintStartupBanner() {
        std::cout << "\n";
        std::cout << "================================================================\n";
        std::cout << "     Manus Core + Hand IK Integration v1.0\n";
        std::cout << "================================================================\n";
        std::cout << "Configuration:\n";
        std::cout << "  • Manus Core: " << manus_host_ << ":" << manus_port_ << "\n";
        std::cout << "  • Hand Side: " << HandSideToString(active_side_) << "\n";
        std::cout << "  • Simulate Mode: " << (simulate_mode_ ? "ON" : "OFF") << "\n";
        std::cout << "  • Debug Logging: " << (debug_logging_ ? "ON" : "OFF") << "\n";
        std::cout << "================================================================\n\n";
    }

    std::string ResolveUrdfPath(int argc, char* argv[]) {
        // Check for --urdf argument
        for (int i = 1; i < argc - 1; ++i) {
            if (std::string(argv[i]) == "--urdf") {
                return argv[i + 1];
            }
        }

        // Check environment variable
        const char* env_urdf = std::getenv("HAND_IK_URDF");
        if (env_urdf) {
            return env_urdf;
        }

        // Default: look next to executable
        std::filesystem::path exe_dir = std::filesystem::path(argv[0]).parent_path();
        std::filesystem::path urdf_path = exe_dir / "surge_v13_hand_right_pybullet.urdf";

        if (std::filesystem::exists(urdf_path)) {
            return urdf_path.string();
        }

        throw std::runtime_error("URDF file not found. Use --urdf <path> or place surge_v13_hand_right_pybullet.urdf next to executable.");
    }

    bool InitializeManusSDK() {
        std::cout << "🔗 Connecting to Manus Core at " << manus_host_ << ":" << manus_port_ << "..." << std::endl;

        // Initialize SDK
        SDKReturnCode result = CoreSdk_Initialize(SessionType::SessionType_CoreSDK);
        if (result != SDKReturnCode::SDKReturnCode_Success) {
            std::cerr << "❌ Failed to initialize Manus SDK: " << static_cast<int>(result) << std::endl;
            return false;
        }

        // Connect to Manus Core
        result = CoreSdk_ConnectGRPC(manus_host_.c_str(), manus_port_);
        if (result != SDKReturnCode::SDKReturnCode_Success) {
            std::cerr << "❌ Failed to connect to Manus Core: " << static_cast<int>(result) << std::endl;
            std::cerr << "   Make sure Manus Core is running and accessible at " << manus_host_ << ":" << manus_port_ << std::endl;
            return false;
        }

        connected_ = true;
        std::cout << "✓ Connected to Manus Core successfully" << std::endl;

        // Set coordinate system to right-handed, Z-up, meters
        CoordinateSystemVUH coordinate_system;
        coordinate_system.handedness = Side::Side_Right;  // Right-handed
        coordinate_system.up = AxisPolarity::AxisPolarity_PositiveZ;  // Z-up
        coordinate_system.view = AxisView::AxisView_ZFromViewer;  // Standard view
        coordinate_system.unitType = MeasurementType::MeasurementType_Meters;  // Meters

        result = CoreSdk_SetCoordinateSystemVUH(coordinate_system, true);
        if (result != SDKReturnCode::SDKReturnCode_Success) {
            std::cerr << "⚠️  Warning: Failed to set coordinate system: " << static_cast<int>(result) << std::endl;
        }
        else {
            std::cout << "✓ Coordinate system set: Right-handed, Z-up, Meters" << std::endl;
        }

        return true;
    }

    void ProcessFrame() {
        frame_count_++;

        if (simulate_mode_) {
            ProcessSimulatedFrame();
        }
        else {
            ProcessRealFrame();
        }
    }

    void ProcessRealFrame() {
        // Get skeleton data from Manus Core
        std::vector<uint32_t> skeleton_ids = skeleton_setup_->GetSkeletonIds();

        for (uint32_t skeleton_id : skeleton_ids) {
            // Get skeleton stream data
            SkeletonStreamInfo skeleton_info;
            SDKReturnCode result = CoreSdk_GetSkeletonStreamData(skeleton_id, &skeleton_info);

            if (result != SDKReturnCode::SDKReturnCode_Success) {
                if (debug_logging_) {
                    std::cerr << "Warning: Failed to get skeleton data for ID " << skeleton_id
                        << ": " << static_cast<int>(result) << std::endl;
                }
                continue;
            }

            // Process the skeleton data
            ProcessSkeletonData(skeleton_id, skeleton_info);
        }
    }

    void ProcessSimulatedFrame() {
        // Generate simple canned trajectory for testing
        static double time = 0.0;
        time += 0.016; // ~60 FPS

        // Simple sinusoidal finger motion
        FingerTargets targets;
        for (int i = 0; i < 4; ++i) {
            double phase = time + i * 0.5; // Phase offset per finger
            double flex = 0.3 + 0.2 * std::sin(phase); // Flex between 0.1 and 0.5 radians

            // Create target positions (simplified)
            targets.finger_positions[i] = Eigen::Vector3d(
                0.12 + i * 0.02,  // X: spread fingers
                0.0,              // Y: centerline
                0.10 + flex * 0.05 // Z: height varies with flexion
            );
        }

        // Thumb target
        targets.thumb_position = Eigen::Vector3d(0.08, 0.06, 0.08);
        targets.thumb_rotation = Eigen::Matrix3d::Identity();

        // Solve IK and update skeleton
        ProcessIKSolve(0, targets); // Use skeleton ID 0 for simulation
    }

    void ProcessSkeletonData(uint32_t skeleton_id, const SkeletonStreamInfo& skeleton_info) {
        // Extract finger targets from skeleton data
        FingerTargets targets;

        if (!ExtractFingerTargets(skeleton_info, targets)) {
            return; // Skip if we can't extract valid targets
        }

        // Solve IK and update skeleton
        ProcessIKSolve(skeleton_id, targets);
    }

    bool ExtractFingerTargets(const SkeletonStreamInfo& skeleton_info, FingerTargets& targets) {
        // Map from Manus skeleton data to our finger targets
        // This requires knowing the Manus skeleton structure and node IDs

        if (skeleton_info.nodesCount == 0) {
            return false;
        }

        // Find fingertip nodes by name matching
        std::vector<std::string> fingertip_names = {
            "index_tip", "middle_tip", "ring_tip", "little_tip"
        };

        bool found_all_fingers = true;

        for (int finger = 0; finger < 4; ++finger) {
            bool found_finger = false;

            for (uint32_t node_idx = 0; node_idx < skeleton_info.nodesCount; ++node_idx) {
                const NodeStreamInfo& node = skeleton_info.nodes[node_idx];

                // Check if this node matches our fingertip pattern
                std::string node_name(node.id.name);
                std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);

                if (node_name.find(fingertip_names[finger]) != std::string::npos) {
                    // Found fingertip node - extract position
                    targets.finger_positions[finger] = Eigen::Vector3d(
                        node.transform.position.x,
                        node.transform.position.y,
                        node.transform.position.z
                    );
                    found_finger = true;
                    break;
                }
            }

            if (!found_finger) {
                found_all_fingers = false;
                if (debug_logging_) {
                    std::cerr << "Warning: Could not find " << fingertip_names[finger] << " node" << std::endl;
                }
            }
        }

        // Find thumb tip
        bool found_thumb = false;
        for (uint32_t node_idx = 0; node_idx < skeleton_info.nodesCount; ++node_idx) {
            const NodeStreamInfo& node = skeleton_info.nodes[node_idx];
            std::string node_name(node.id.name);
            std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);

            if (node_name.find("thumb_tip") != std::string::npos) {
                targets.thumb_position = Eigen::Vector3d(
                    node.transform.position.x,
                    node.transform.position.y,
                    node.transform.position.z
                );

                // Extract thumb orientation
                const auto& q = node.transform.rotation;
                Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
                targets.thumb_rotation = quat.toRotationMatrix();
                found_thumb = true;
                break;
            }
        }

        if (!found_thumb && debug_logging_) {
            std::cerr << "Warning: Could not find thumb_tip node" << std::endl;
        }

        return found_all_fingers && found_thumb;
    }

    void ProcessIKSolve(uint32_t skeleton_id, const FingerTargets& targets) {
        auto start_time = std::chrono::high_resolution_clock::now();

        // Solve IK using our Hand IK bridge
        JointConfiguration joint_config;
        bool ik_success = ik_bridge_->Solve(targets, joint_config);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto solve_time = std::chrono::duration<double, std::milli>(end_time - start_time);

        // Update performance tracking
        ik_solve_count_++;
        total_solve_time_ms_ += solve_time.count();

        if (!ik_success) {
            if (debug_logging_) {
                std::cerr << "IK solve failed for skeleton " << skeleton_id << std::endl;
            }
            return;
        }

        // Update Manus skeleton with solved joint configuration
        UpdateManusSkeletonPose(skeleton_id, joint_config);

        if (debug_logging_) {
            std::cout << "IK solved in " << solve_time.count() << "ms for skeleton " << skeleton_id << std::endl;
        }
    }

    void UpdateManusSkeletonPose(uint32_t skeleton_id, const JointConfiguration& joint_config) {
        // Convert our joint configuration back to Manus skeleton pose
        // This involves mapping our 6 active joints to the appropriate Manus skeleton nodes

        SkeletonStreamInfo skeleton_info;
        SDKReturnCode result = CoreSdk_GetSkeletonStreamData(skeleton_id, &skeleton_info);
        if (result != SDKReturnCode::SDKReturnCode_Success) {
            return;
        }

        // Update joint values in the skeleton
        // Map our solved joint angles to the Manus skeleton structure

        std::vector<std::string> joint_names = {
            "index_mcp", "middle_mcp", "ring_mcp", "pinky_mcp",
            "thumb_rotation", "thumb_flexion"
        };

        for (uint32_t node_idx = 0; node_idx < skeleton_info.nodesCount; ++node_idx) {
            NodeStreamInfo& node = skeleton_info.nodes[node_idx];
            std::string node_name(node.id.name);
            std::transform(node_name.begin(), node_name.end(), node_name.begin(), ::tolower);

            // Map joint angles to corresponding nodes
            for (int joint_idx = 0; joint_idx < 6; ++joint_idx) {
                if (node_name.find(joint_names[joint_idx]) != std::string::npos) {
                    // Update node rotation based on solved joint angle
                    double angle = joint_config.joint_angles[joint_idx];

                    // Convert angle to quaternion rotation (assuming rotation about Y-axis for MCP joints)
                    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitY());
                    Eigen::Quaterniond quat(rotation);

                    node.transform.rotation.x = quat.x();
                    node.transform.rotation.y = quat.y();
                    node.transform.rotation.z = quat.z();
                    node.transform.rotation.w = quat.w();
                    break;
                }
            }
        }

        // Send updated skeleton back to Manus Core
        result = CoreSdk_SendSkeletonStreamData(skeleton_id, &skeleton_info);
        if (result != SDKReturnCode::SDKReturnCode_Success && debug_logging_) {
            std::cerr << "Warning: Failed to send skeleton data for ID " << skeleton_id
                << ": " << static_cast<int>(result) << std::endl;
        }
    }

    void PrintPerformanceStats() {
        double fps = frame_count_ / 5.0;
        double avg_solve_time = ik_solve_count_ > 0 ? total_solve_time_ms_ / ik_solve_count_ : 0.0;

        std::cout << "📊 Performance [5s]: "
            << "FPS=" << std::fixed << std::setprecision(1) << fps
            << ", IK solves=" << ik_solve_count_
            << ", Avg solve time=" << std::setprecision(3) << avg_solve_time << "ms"
            << std::endl;

        // Reset counters
        frame_count_ = 0;
        ik_solve_count_ = 0;
        total_solve_time_ms_ = 0.0;
    }

    std::string HandSideToString(HandSide side) {
        switch (side) {
        case HandSide::Left: return "Left";
        case HandSide::Right: return "Right";
        case HandSide::Both: return "Both";
        default: return "Unknown";
        }
    }
};

// Global integration instance
std::unique_ptr<ManusIntegration> g_integration;

// Signal handler for graceful shutdown
void SignalHandler(int signal) {
    std::cout << "\n🛑 Received shutdown signal (" << signal << ")" << std::endl;
    if (g_integration) {
        g_integration->Shutdown();
    }
    exit(0);
}

int main(int argc, char* argv[]) {
    // Set up signal handling for graceful shutdown
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    try {
        // Create and initialize integration
        g_integration = std::make_unique<ManusIntegration>();

        if (!g_integration->Initialize(argc, argv)) {
            std::cerr << "❌ Integration initialization failed" << std::endl;
            return 1;
        }

        // Run main processing loop
        g_integration->Run();

    }
    catch (const std::exception& e) {
        std::cerr << "❌ Integration error: " << e.what() << std::endl;
        return 1;
    }

    // Clean shutdown
    if (g_integration) {
        g_integration->Shutdown();
    }

    return 0;
}