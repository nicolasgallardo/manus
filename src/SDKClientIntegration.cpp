#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <random>
#include <iomanip>

// Manus SDK headers
#ifdef MANUS_SDK_AVAILABLE
#include "ManusSDK.h"
#endif

// Hand IK integration
#include "ManusHandIKBridge.h"

// UTF-8 safe emoji definitions (Fix 1)
#ifdef _WIN32
#define ICON_OK   u8"✓"
#define ICON_BAD  u8"❌" 
#define ICON_WARN u8"⚠️"
#define ICON_ROCKET u8"🚀"
#define ICON_LINK u8"🔗"
#define ICON_STOP u8"🛑"
#define ICON_DATA u8"📊"
#else
#define ICON_OK   "✓"
#define ICON_BAD  "❌"
#define ICON_WARN "⚠️"
#define ICON_ROCKET "🚀"
#define ICON_LINK "🔗"
#define ICON_STOP "🛑"
#define ICON_DATA "📊"
#endif

// Global shutdown flag
std::atomic<bool> g_shutdown{ false };

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    std::cout << "\n[SIGNAL] Received signal " << signal << ", shutting down..." << std::endl;
    g_shutdown = true;
}

#ifdef MANUS_SDK_AVAILABLE

class ManusSDKClient {
private:
    uint32_t session_id_;
    bool connected_;
    std::unique_ptr<ManusHandIKBridge> ik_bridge_;
    ManusIntegrationConfig config_;

public:
    ManusSDKClient() : session_id_(0), connected_(false) {}

    ~ManusSDKClient() {
        shutdown();
    }

    bool initialize(const std::string& config_path) {
        std::cout << ICON_ROCKET << " Initializing Manus SDK Client with configuration..." << std::endl;

        try {
            // Load configuration first
            if (!config_path.empty() && std::filesystem::exists(config_path)) {
                config_ = ManusIntegrationConfig::LoadFromFile(config_path);
                std::cout << ICON_OK << " Configuration loaded from: " << config_path << std::endl;
            }
            else {
                std::cout << ICON_WARN << " Config file not found, using defaults" << std::endl;
                config_ = ManusIntegrationConfig{};
            }

            // Step 1: Initialize Manus Core SDK
            std::cout << ICON_WARN << " Initializing Manus Core SDK..." << std::endl;

            SDKReturnCode initResult = CoreSdk_InitializeCore();
            if (initResult != SDKReturnCode_Success) {
                std::cout << ICON_BAD << " Failed to initialize Manus Core SDK (status: " << initResult << ")" << std::endl;
                return false;
            }
            std::cout << ICON_OK << " Manus Core SDK initialized" << std::endl;

            // Step 2: Set coordinate system based on config
            std::cout << ICON_WARN << " Setting up coordinate system..." << std::endl;

            CoordinateSystemVUH coordSystem{};

            // Use config settings or defaults
            if (config_.coordinate_system.handedness == "right") {
                coordSystem.handedness = Side::Side_Right;
            }
            else {
                coordSystem.handedness = Side::Side_Left;
            }

            // Map up axis from config
            if (config_.coordinate_system.up_axis == "positive_y") {
                coordSystem.up = AxisPolarity::AxisPolarity_PositiveY;
            }
            else if (config_.coordinate_system.up_axis == "positive_z") {
                coordSystem.up = AxisPolarity::AxisPolarity_PositiveZ;
            }
            else {
                coordSystem.up = AxisPolarity::AxisPolarity_PositiveY; // Default
            }

            // Map view axis from config
            if (config_.coordinate_system.view_axis == "x_from_viewer") {
                coordSystem.view = AxisView::AxisView_XFromViewer;
            }
            else if (config_.coordinate_system.view_axis == "z_from_viewer") {
                coordSystem.view = AxisView::AxisView_ZFromViewer;
            }
            else {
                coordSystem.view = AxisView::AxisView_XFromViewer; // Default
            }

            coordSystem.unitScale = 1.0f; // Always use meters

            SDKReturnCode coordResult = CoreSdk_InitializeCoordinateSystemWithVUH(coordSystem, true);
            if (coordResult != SDKReturnCode_Success) {
                std::cout << ICON_BAD << " Failed to set coordinate system (status: " << coordResult << ")" << std::endl;
                return false;
            }

            std::cout << ICON_OK << " Coordinate system configured:" << std::endl;
            std::cout << "  Handedness: " << config_.coordinate_system.handedness << std::endl;
            std::cout << "  Up axis: " << config_.coordinate_system.up_axis << std::endl;
            std::cout << "  View axis: " << config_.coordinate_system.view_axis << std::endl;

            // Step 3: Initialize Hand IK bridge with config
            std::cout << ICON_WARN << " Initializing Hand IK bridge with configuration..." << std::endl;

            try {
                ik_bridge_ = std::make_unique<ManusHandIKBridge>(config_.urdf_path, config_path);

                if (!ik_bridge_->IsInitialized()) {
                    std::cout << ICON_BAD << " Hand IK bridge failed to initialize" << std::endl;
                    return false;
                }
                std::cout << ICON_OK << " Hand IK bridge initialized with configuration" << std::endl;
            }
            catch (const std::exception& e) {
                std::cout << ICON_BAD << " Hand IK bridge initialization failed: " << e.what() << std::endl;
                return false;
            }

            return true;
        }
        catch (const std::exception& e) {
            std::cout << ICON_BAD << " Exception during initialization: " << e.what() << std::endl;
            return false;
        }
    }

    bool connectToCore() {
        std::cout << ICON_LINK << " Attempting to connect to Manus Core..." << std::endl;

        // Add pre-connection diagnostics
        std::cout << ICON_WARN << " Connection diagnostics:" << std::endl;
        std::cout << "  - Host: " << config_.connection.host << std::endl;
        std::cout << "  - Port: " << config_.connection.port << std::endl;
        std::cout << "  - Timeout: " << config_.connection.timeout_ms << "ms" << std::endl;
        std::cout << "  - Retry attempts: " << config_.connection.retry_attempts << std::endl;
        std::cout << "  - Protocol: gRPC" << std::endl;

        // Try to get more info about the connection attempt
        std::cout << ICON_WARN << " Attempting gRPC connection..." << std::endl;

        SDKReturnCode result = CoreSdk_ConnectGRPC();

        // Provide detailed error information
        switch (result) {
        case SDKReturnCode_Success:
            connected_ = true;
            std::cout << ICON_OK << " Connected to Manus Core successfully!" << std::endl;

            // Get session ID for reference
            CoreSdk_GetSessionId(&session_id_);
            std::cout << ICON_OK << " Session ID: " << session_id_ << std::endl;

            std::cout << ICON_OK << " Connection established, ready for data streaming" << std::endl;
            return true;

        case SDKReturnCode_Error:
            std::cout << ICON_BAD << " Generic error connecting to Manus Core" << std::endl;
            break;

        case SDKReturnCode_InvalidArgument:
            std::cout << ICON_BAD << " Invalid argument error" << std::endl;
            break;

        case SDKReturnCode_NotConnected:
            std::cout << ICON_BAD << " Not connected - Manus Core may not be running" << std::endl;
            break;

        default:
            std::cout << ICON_BAD << " Unknown connection error (status code: " << result << ")" << std::endl;
            break;
        }

        // Provide troubleshooting guidance
        std::cout << std::endl;
        std::cout << ICON_WARN << " Connection failed. Troubleshooting steps:" << std::endl;
        std::cout << "  1. Ensure Manus Dashboard is running and shows 'Connected'" << std::endl;
        std::cout << "  2. Check if Manus Core service is running in background" << std::endl;
        std::cout << "  3. Verify no firewall is blocking port " << config_.connection.port << std::endl;
        std::cout << "  4. Try restarting Manus Dashboard" << std::endl;
        std::cout << "  5. Check Manus logs for detailed error information" << std::endl;

        return false;
    }

    void registerCallbacks() {
        if (!connected_) return;

        std::cout << ICON_WARN << " Registering SDK callbacks..." << std::endl;

        // Register skeleton stream callback
        auto skeletonCallback = [](const SkeletonStreamInfo* skeletonInfo) {
            // Process skeleton data here
            if (skeletonInfo && skeletonInfo->skeletonsCount > 0) {
                std::cout << ICON_DATA << " Received skeleton data for " << skeletonInfo->skeletonsCount << " skeletons" << std::endl;
            }
            };

        SDKReturnCode callbackResult = CoreSdk_RegisterCallbackForSkeletonStream(skeletonCallback);
        if (callbackResult == SDKReturnCode_Success) {
            std::cout << ICON_OK << " Skeleton stream callback registered" << std::endl;
        }
        else {
            std::cout << ICON_BAD << " Failed to register skeleton callback (status: " << callbackResult << ")" << std::endl;
        }
    }

    void processHandIKData() {
        std::cout << ICON_DATA << " Starting Hand IK processing loop..." << std::endl;
        std::cout << "  Target FPS: " << config_.performance.target_fps << std::endl;
        std::cout << "  Max solve time: " << config_.performance.max_solve_time_ms << "ms" << std::endl;

        auto target_frame_time = std::chrono::milliseconds(1000 / config_.performance.target_fps);

        while (!g_shutdown && connected_) {
            auto frame_start = std::chrono::high_resolution_clock::now();

            try {
                if (ik_bridge_) {
                    // Example: Create test finger targets
                    FingerTargets test_targets;

                    // Set some realistic finger positions (in meters, hand coordinate frame)
                    test_targets.finger_positions[0] = Eigen::Vector3d(0.15, 0.05, 0.12);  // Index
                    test_targets.finger_positions[1] = Eigen::Vector3d(0.16, 0.02, 0.14);  // Middle  
                    test_targets.finger_positions[2] = Eigen::Vector3d(0.15, -0.02, 0.13); // Ring
                    test_targets.finger_positions[3] = Eigen::Vector3d(0.13, -0.05, 0.11); // Pinky

                    test_targets.thumb_position = Eigen::Vector3d(0.08, 0.08, 0.10);
                    test_targets.thumb_rotation = Eigen::Matrix3d::Identity();

                    // Solve Hand IK
                    JointConfiguration joint_config;
                    bool solved = ik_bridge_->Solve(test_targets, joint_config);

                    if (solved && joint_config.valid) {
                        // Success - the bridge handles performance logging automatically
                    }
                    else {
                        static int fail_count = 0;
                        fail_count++;
                        if (fail_count % 60 == 1) { // Throttle failure messages
                            std::cout << ICON_WARN << " Hand IK solve failed (count: " << fail_count << ")" << std::endl;
                        }
                    }
                }

                // Frame rate control
                auto frame_end = std::chrono::high_resolution_clock::now();
                auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(frame_end - frame_start);

                if (frame_duration < target_frame_time) {
                    std::this_thread::sleep_for(target_frame_time - frame_duration);
                }

            }
            catch (const std::exception& e) {
                std::cout << ICON_BAD << " Error in Hand IK processing: " << e.what() << std::endl;
                break;
            }
        }
    }

    // ONLY REPLACE THE runStandaloneIKTest METHOD in your existing SDKClientIntegration.cpp
    // Find this method and replace it with the version below:

    void runStandaloneIKTest() {
        if (!ik_bridge_) {
            std::cout << ICON_BAD << " No Hand IK bridge for testing" << std::endl;
            return;
        }

        std::cout << ICON_WARN << " Running standalone Hand IK test with PROPER analytical Jacobian..." << std::endl;

        // Test 1: Run built-in diagnostics
        std::cout << ICON_WARN << " Running Hand IK diagnostics..." << std::endl;
        bool diagnostics_ok = ik_bridge_->RunDiagnostics();

        // Test 2: Use KNOWN working targets from test suite
        std::cout << ICON_WARN << " Testing with KNOWN working targets from test suite..." << std::endl;

        // These targets are based on the working test configurations (5-9cm range)
        FingerTargets working_targets;
        working_targets.finger_positions[0] = Eigen::Vector3d(0.06, 0.01, 0.04);   // Index
        working_targets.finger_positions[1] = Eigen::Vector3d(0.065, 0.005, 0.045); // Middle  
        working_targets.finger_positions[2] = Eigen::Vector3d(0.06, -0.01, 0.04);   // Ring
        working_targets.finger_positions[3] = Eigen::Vector3d(0.055, -0.02, 0.035); // Pinky
        working_targets.thumb_position = Eigen::Vector3d(0.04, 0.04, 0.04);         // Thumb
        working_targets.thumb_rotation = Eigen::Matrix3d::Identity();

        std::cout << "Working target distances from origin:" << std::endl;
        for (int i = 0; i < 4; ++i) {
            double dist = working_targets.finger_positions[i].norm();
            std::cout << "  Finger " << i << ": " << std::fixed << std::setprecision(3) << dist << "m" << std::endl;
        }
        std::cout << "  Thumb: " << working_targets.thumb_position.norm() << "m" << std::endl;

        // Enable verbose mode for detailed diagnostics
        ik_bridge_->SetVerbose(true);

        JointConfiguration joint_config;
        bool solved = ik_bridge_->Solve(working_targets, joint_config);

        if (solved && joint_config.valid) {
            std::cout << ICON_OK << " Working targets: SUCCESS ("
                << joint_config.solve_time_ms << "ms, "
                << joint_config.iterations << " iterations)" << std::endl;

            std::cout << "    Joint angles (radians): ";
            for (int k = 0; k < 6; ++k) {
                std::cout << std::fixed << std::setprecision(3) << joint_config.joint_angles[k] << " ";
            }
            std::cout << std::endl;

            // Check thumb flexion is working
            if (joint_config.joint_angles[5] != 0.0) {
                std::cout << "    " << ICON_OK << " Thumb flexion working: " << joint_config.joint_angles[5]
                    << " rad (" << (joint_config.joint_angles[5] * 180.0 / 3.14159) << "°)" << std::endl;
            }
            else {
                std::cout << "    " << ICON_WARN << " Thumb flexion still at zero - check joint limits" << std::endl;
            }
        }
        else {
            std::cout << ICON_BAD << " Working targets: FAILED" << std::endl;
            std::cout << "This indicates the analytical Jacobian fix may not be complete!" << std::endl;
        }

        // Disable verbose mode
        ik_bridge_->SetVerbose(false);

        // Test 3: Test multiple working targets with small variations
        std::cout << ICON_WARN << " Testing multiple working targets with variations..." << std::endl;

        int successful_solves = 0;
        const int num_tests = 10;

        for (int i = 0; i < num_tests; ++i) {
            // Start with working base and add small variations
            FingerTargets varied_targets = working_targets;

            // Add ±2mm random variation (very small)
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> noise(-0.002, 0.002);

            for (int j = 0; j < 4; ++j) {
                varied_targets.finger_positions[j] += Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
            }
            varied_targets.thumb_position += Eigen::Vector3d(noise(gen), noise(gen), noise(gen));

            // Solve IK
            bool test_solved = ik_bridge_->Solve(varied_targets, joint_config);

            if (test_solved && joint_config.valid) {
                successful_solves++;

                if (i == 0) {  // Print details for first solve
                    std::cout << ICON_OK << " IK solve " << (i + 1) << ": SUCCESS ("
                        << joint_config.solve_time_ms << "ms, "
                        << joint_config.iterations << " iterations)" << std::endl;
                }
            }
            else {
                if (i < 3) { // Only print first few failures
                    std::cout << ICON_BAD << " IK solve " << (i + 1) << ": FAILED" << std::endl;
                }
            }
        }

        double success_rate = (double)successful_solves / num_tests * 100.0;
        std::cout << ICON_DATA << " Working targets test results: " << successful_solves << "/" << num_tests
            << " (" << std::fixed << std::setprecision(1) << success_rate << "%) successful" << std::endl;

        if (success_rate >= 80.0) {
            std::cout << ICON_OK << " Hand IK integration is working with PROPER analytical Jacobian!" << std::endl;
        }
        else if (success_rate > 0.0) {
            std::cout << ICON_WARN << " Hand IK works but may need further tuning" << std::endl;
        }
        else {
            std::cout << ICON_BAD << " Hand IK solver still has issues - check the fixes were applied correctly" << std::endl;
        }

        std::cout << std::endl;
        std::cout << ICON_OK << " Standalone test complete. Expected improvements:" << std::endl;
        std::cout << "  - Jacobian FD errors should be ~1e-6 instead of ~1e-1" << std::endl;
        std::cout << "  - Solve times should be ~1-5ms instead of 7000ms" << std::endl;
        std::cout << "  - Success rates should be ≥80% instead of 0%" << std::endl;
        std::cout << "  - Thumb flexion should show non-zero angles" << std::endl;
    }

    void runDiagnostics() {
        if (!ik_bridge_) {
            std::cout << ICON_BAD << " No Hand IK bridge for diagnostics" << std::endl;
            return;
        }

        std::cout << ICON_WARN << " Running Hand IK diagnostics..." << std::endl;

        bool diagnostics_ok = ik_bridge_->RunDiagnostics();
        if (diagnostics_ok) {
            std::cout << ICON_OK << " Hand IK diagnostics passed" << std::endl;
        }
        else {
            std::cout << ICON_BAD << " Hand IK diagnostics failed" << std::endl;
        }
    }

    void shutdown() {
        std::cout << ICON_STOP << " Shutting down Manus SDK Client..." << std::endl;

        if (connected_) {
            // Disconnect from Core
            SDKReturnCode disconnectResult = CoreSdk_Disconnect();
            if (disconnectResult == SDKReturnCode_Success) {
                std::cout << ICON_OK << " Disconnected from Manus Core" << std::endl;
            }
            connected_ = false;
        }

        // Clean up Hand IK bridge
        ik_bridge_.reset();
        std::cout << ICON_OK << " Hand IK bridge cleaned up" << std::endl;

        // Shutdown SDK
        SDKReturnCode shutdownResult = CoreSdk_ShutDown();
        if (shutdownResult == SDKReturnCode_Success) {
            std::cout << ICON_OK << " Manus SDK shutdown complete" << std::endl;
        }
    }
};

#else

// Fallback implementation when Manus SDK is not available
class ManusSDKClient {
public:
    bool initialize(const std::string& config_path) {
        std::cout << ICON_BAD << " Manus SDK not available - running in simulation mode" << std::endl;
        return false;
    }

    bool connectToCore() {
        std::cout << ICON_BAD << " Manus SDK not available" << std::endl;
        return false;
    }

    void registerCallbacks() {}
    void processHandIKData() {
        std::cout << "Simulation mode - no real data processing" << std::endl;
        while (!g_shutdown) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    void runStandaloneIKTest() {}
    void runDiagnostics() {}
    void shutdown() {}
};

#endif

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "Manus SDK Integration with Hand IK" << std::endl;
    std::cout << "========================================" << std::endl;

    // Set up signal handling for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Load configuration
    std::string config_path;

    // Check for config file in multiple locations
    std::vector<std::string> config_candidates = {
        "config/manus_integration.json",
        "../config/manus_integration.json",
        "manus_integration.json",
        "C:/Users/nicol/Alt_Bionics/GitHub/manus/config/manus_integration.json"
    };

    bool config_found = false;
    for (const auto& candidate : config_candidates) {
        if (std::filesystem::exists(candidate)) {
            config_path = candidate;
            config_found = true;
            break;
        }
    }

    if (config_found) {
        std::cout << ICON_OK << " Found config file: " << config_path << std::endl;
    }
    else {
        std::cout << ICON_WARN << " Config file not found, using defaults" << std::endl;
        std::cout << "Checked locations:" << std::endl;
        for (const auto& candidate : config_candidates) {
            std::cout << "  - " << candidate << std::endl;
        }
        config_path = ""; // Use empty path for defaults
    }

    ManusSDKClient client;

    // Initialize the client with config
    if (!client.initialize(config_path)) {
        std::cout << ICON_BAD << " Failed to initialize SDK client" << std::endl;
        return 1;
    }

    // Try to connect to Manus Core
    if (client.connectToCore()) {
        client.registerCallbacks();
        client.runDiagnostics();
        client.processHandIKData();
    }
    else {
        std::cout << ICON_BAD << " Could not connect to Manus Core" << std::endl;
        std::cout << ICON_WARN << " Running standalone Hand IK test instead..." << std::endl;
        client.runStandaloneIKTest();
    }

    client.shutdown();

    std::cout << "Application terminated." << std::endl;
    return 0;
}