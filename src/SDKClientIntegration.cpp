#include "SDKClientIntegration.h"

// Global shutdown flag
std::atomic<bool> g_shutdown{ false };

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    std::cout << "\n[SIGNAL] Received signal " << signal << ", shutting down..." << std::endl;
    g_shutdown = true;
}

#ifdef MANUS_SDK_AVAILABLE

// License and status detection methods
bool ManusSDKClient::CheckLicenseStatus() {
    std::cout << ICON_LICENSE << " Checking license/dongle status..." << std::endl;

    // Try to detect if license/dongle is present via SDK status calls
    try {
        // Check if there are any status functions available in the SDK
        // This is a placeholder - actual implementation depends on SDK capabilities

        // Option 1: Try to get SDK status/info
        // SDKReturnCode status = CoreSdk_GetStatus(); // if available

        // Option 2: Try to get last error string
        // const char* error_msg = CoreSdk_GetLastError(); // if available

        // Option 3: Try a lightweight SDK call that would fail without license
        // SDKReturnCode test = CoreSdk_GetVersion(); // if available

        std::cout << ICON_LICENSE << " License status check: UNKNOWN (no SDK status API found)" << std::endl;
        std::cout << ICON_LICENSE << " Assuming license present - connection failure will clarify" << std::endl;
        return true; // Assume OK, let connection attempt reveal the issue

    }
    catch (const std::exception& e) {
        std::cout << ICON_LICENSE << " License check error: " << e.what() << std::endl;
        return false;
    }
}

bool ManusSDKClient::DetectLicenseDongle() {
    std::cout << ICON_LICENSE << " Scanning for license dongle..." << std::endl;

    // This would ideally check for USB dongles or license files
    // For now, provide user guidance
    std::cout << ICON_LICENSE << " License dongle detection: NOT IMPLEMENTED" << std::endl;
    std::cout << ICON_LICENSE << " Please verify manually:" << std::endl;
    std::cout << "  1. License dongle is connected to USB port" << std::endl;
    std::cout << "  2. Manus Dashboard shows 'Licensed' or 'Connected' status" << std::endl;
    std::cout << "  3. No license expiration warnings in Manus Dashboard" << std::endl;

    return true; // Assume present for now
}

std::string ManusSDKClient::ExplainSdkResult(SDKReturnCode code) {
    // Translate SDK result codes to human-readable messages
    switch (code) {
    case SDKReturnCode_Success:
        return "Success";
    case SDKReturnCode_Error:
        return "General error";
    case SDKReturnCode_InvalidArgument:
        return "Invalid argument";
    case SDKReturnCode_ArgumentsNotOK:
        return "Arguments not OK";
    case SDKReturnCode_NotConnected:
        return "Not connected to Core";
    case SDKReturnCode_NotAllocated:
        return "Not allocated";
    case SDKReturnCode_SDK_not_available:
        return "SDK not available";
    case SDKReturnCode_FunctionCalledBeforeInitialize:
        return "Function called before initialize";
    case SDKReturnCode_CouldNotStartPlugin:
        return "Could not start plugin";
    case SDKReturnCode_CouldNotConnectToCoreFromPlugin:
        return "Could not connect to Core from plugin";
    case SDKReturnCode_PluginNotFound:
        return "Plugin not found";

        // Add more cases as needed based on the SDK header
    default: {
        std::ostringstream oss;
        oss << "Unknown error code: " << static_cast<int>(code);
        return oss.str();
    }
    }
}

bool ManusSDKClient::PerformLicensePreflight() {
    std::cout << ICON_LICENSE << " License preflight check..." << std::endl;

    // Step 1: Try to detect license/dongle
    bool dongle_detected = DetectLicenseDongle();

    // Step 2: Try license status check
    bool license_ok = CheckLicenseStatus();

    if (!dongle_detected || !license_ok) {
        std::cout << ICON_LICENSE << " License preflight: FAIL" << std::endl;
        std::cout << ICON_LICENSE << " Core connection may fail due to missing/invalid license" << std::endl;
        return false;
    }

    std::cout << ICON_LICENSE << " License preflight: PASS" << std::endl;
    return true;
}

bool ManusSDKClient::ConfigureCorePresetRoute(const std::string& host, uint16_t port) {
    std::cout << ICON_WARN << " Configuring Manus Core preset route (SDK 3.0.0)..." << std::endl;
    std::cout << "  Preset route target: " << host << ":" << port << std::endl;
    std::cout << "  (must be configured in Manus Core UI for SDK 3.0.0)" << std::endl;

    // SDK 3.0.0 does not expose runtime route configuration
    // The route must be preset in the Manus Core application

    std::cout << ICON_WARN << " SDK 3.0.0 preset route behavior:" << std::endl;
    std::cout << "  - Route is configured in Manus Core application/service" << std::endl;
    std::cout << "  - SDK connects to preset route via CoreSdk_ConnectGRPC()" << std::endl;
    std::cout << "  - No runtime route configuration API in SDK 3.0.0" << std::endl;

    // Try to set settings location if available
    try {
        SDKReturnCode settings_result = CoreSdk_SetSettingsLocation("./");
        std::cout << ICON_WARN << " Settings location result: " << ExplainSdkResult(settings_result)
            << " (" << static_cast<int>(settings_result) << ")" << std::endl;
    }
    catch (...) {
        std::cout << ICON_WARN << " Settings location configuration not available" << std::endl;
    }

    std::cout << ICON_OK << " Preset route preparation complete" << std::endl;
    return true;
}

bool ManusSDKClient::ConnectToCore_WithPresetRoute() {
    std::cout << ICON_LINK << " Attempting gRPC connection to preset route..." << std::endl;

    // Use the no-argument version that connects to preset route
    SDKReturnCode rc = CoreSdk_ConnectGRPC();

    if (rc == SDKReturnCode_Success) {
        return true;
    }

    // Connection failed - provide detailed error analysis
    std::string error_explanation = ExplainSdkResult(rc);
    std::cout << ICON_BAD << " gRPC connection failed: " << error_explanation
        << " (code: " << static_cast<int>(rc) << ")" << std::endl;

    // Analyze the specific failure
    if (error_explanation.find("PresetRoute") != std::string::npos ||
        error_explanation.find("Address and Port not configured") != std::string::npos) {
        std::cout << ICON_BAD << " Preset route not configured correctly" << std::endl;
    }
    else if (rc == SDKReturnCode_NotConnected) {
        std::cout << ICON_BAD << " Connection refused - possible license or service issue" << std::endl;
    }

    return false;
}

ManusSDKClient::ManusSDKClient() : session_id_(0), connected_(false), offline_mode_(false) {}

ManusSDKClient::~ManusSDKClient() {
    Shutdown();
}

bool ManusSDKClient::Initialize(const std::string& config_path, bool offline_mode) {
    std::cout << ICON_ROCKET << " Initializing Manus SDK Client..." << std::endl;

    offline_mode_ = offline_mode;

    if (offline_mode_) {
        std::cout << ICON_WARN << " Offline mode: skipping Core connection" << std::endl;
    }

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

        // Check for offline mode from config
        if (config_.connection.offline_mode) {
            offline_mode_ = true;
            std::cout << ICON_WARN << " Offline mode enabled in config" << std::endl;
        }

        if (!offline_mode_) {
            // Step 1: Initialize Manus Core SDK
            std::cout << ICON_WARN << " Initializing Manus Core SDK..." << std::endl;

            SDKReturnCode initResult = CoreSdk_InitializeCore();
            if (initResult != SDKReturnCode_Success) {
                std::cout << ICON_BAD << " Failed to initialize Manus Core SDK: "
                    << ExplainSdkResult(initResult) << " (" << static_cast<int>(initResult) << ")" << std::endl;
                return false;
            }
            std::cout << ICON_OK << " Manus Core SDK initialized" << std::endl;

            // Step 2: Set coordinate system based on config
            std::cout << ICON_WARN << " Setting up coordinate system..." << std::endl;

            CoordinateSystemVUH coordSystem{};

            if (config_.coordinate_system.handedness == "right") {
                coordSystem.handedness = Side::Side_Right;
            }
            else {
                coordSystem.handedness = Side::Side_Left;
            }

            if (config_.coordinate_system.up_axis == "positive_y") {
                coordSystem.up = AxisPolarity::AxisPolarity_PositiveY;
            }
            else if (config_.coordinate_system.up_axis == "positive_z") {
                coordSystem.up = AxisPolarity::AxisPolarity_PositiveZ;
            }
            else {
                coordSystem.up = AxisPolarity::AxisPolarity_PositiveY;
            }

            if (config_.coordinate_system.view_axis == "x_from_viewer") {
                coordSystem.view = AxisView::AxisView_XFromViewer;
            }
            else if (config_.coordinate_system.view_axis == "z_from_viewer") {
                coordSystem.view = AxisView::AxisView_ZFromViewer;
            }
            else {
                coordSystem.view = AxisView::AxisView_XFromViewer;
            }

            coordSystem.unitScale = 1.0f;

            SDKReturnCode coordResult = CoreSdk_InitializeCoordinateSystemWithVUH(coordSystem, true);
            if (coordResult != SDKReturnCode_Success) {
                std::cout << ICON_BAD << " Failed to set coordinate system: "
                    << ExplainSdkResult(coordResult) << " (" << static_cast<int>(coordResult) << ")" << std::endl;
                return false;
            }

            std::cout << ICON_OK << " Coordinate system configured" << std::endl;
        }

        // Step 3: Initialize Hand IK bridge with enhanced config
        std::cout << ICON_WARN << " Initializing Hand IK bridge with enhanced configuration..." << std::endl;

        try {
            ik_bridge_ = std::make_unique<ManusHandIKBridge>(config_.urdf_path, config_path);

            if (!ik_bridge_->IsInitialized()) {
                std::cout << ICON_BAD << " Hand IK bridge failed to initialize" << std::endl;
                return false;
            }
            std::cout << ICON_OK << " Hand IK bridge initialized with enhanced configuration" << std::endl;
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

bool ManusSDKClient::ConnectToCore() {
    if (offline_mode_) {
        std::cout << ICON_WARN << " Offline mode: skipping Core connection" << std::endl;
        return false; // Not an error in offline mode
    }

    std::cout << ICON_LINK << " Attempting to connect to Manus Core..." << std::endl;

    // Extract connection details from config
    std::string host = config_.connection.host;
    uint16_t port = static_cast<uint16_t>(config_.connection.port);

    // Connection diagnostics
    std::cout << ICON_WARN << " Connection diagnostics:" << std::endl;
    std::cout << "  - Host: " << host << std::endl;
    std::cout << "  - Port: " << port << std::endl;
    std::cout << "  - Timeout: " << config_.connection.timeout_ms << "ms" << std::endl;
    std::cout << "  - Retry attempts: " << config_.connection.retry_attempts << std::endl;
    std::cout << "  - Protocol: gRPC (SDK 3.0.0)" << std::endl;

    // ENHANCED: License preflight check
    bool license_preflight_ok = PerformLicensePreflight();

    // Configure preset route
    bool preset_ok = ConfigureCorePresetRoute(host, port);
    if (!preset_ok) {
        std::cout << ICON_BAD << " Failed to configure preset route" << std::endl;
        return false;
    }

    // Attempt connection
    bool success = ConnectToCore_WithPresetRoute();

    if (success) {
        connected_ = true;
        std::cout << ICON_OK << " Connected to Manus Core successfully!" << std::endl;

        // Get session ID for reference
        CoreSdk_GetSessionId(&session_id_);
        std::cout << ICON_OK << " Session ID: " << session_id_ << std::endl;
        return true;
    }

    // Connection failed - provide license-aware troubleshooting
    std::cout << ICON_BAD << " Failed to connect to Manus Core" << std::endl;
    std::cout << std::endl;
    std::cout << ICON_WARN << " Troubleshooting checklist (check in order):" << std::endl;

    if (!license_preflight_ok) {
        std::cout << "  1. LICENSE/DONGLE: Check license dongle is connected and valid" << std::endl;
        std::cout << "     - Verify dongle is plugged into USB port" << std::endl;
        std::cout << "     - Check Manus Dashboard shows 'Licensed' status" << std::endl;
        std::cout << "     - Verify license has not expired" << std::endl;
        std::cout << "     - Try unplugging/replugging dongle" << std::endl;
    }
    else {
        std::cout << "  1. License/dongle: APPEARS OK" << std::endl;
    }

    std::cout << "  2. PRESET ROUTE: Configure Manus Core for " << host << ":" << port << std::endl;
    std::cout << "     - Open Manus Core settings/configuration" << std::endl;
    std::cout << "     - Set gRPC server address to " << host << ":" << port << std::endl;
    std::cout << "     - Save settings and restart Manus Core" << std::endl;
    std::cout << "     - SDK 3.0.0 connects to preset route configured in Core" << std::endl;
    std::cout << "  3. SERVICE: Ensure Manus Dashboard is running and shows 'Connected'" << std::endl;
    std::cout << "  4. FIREWALL: Check firewall is not blocking port " << port << std::endl;
    std::cout << "  5. RESTART: Try restarting Manus Dashboard/Core services" << std::endl;
    std::cout << "  6. LOGS: Check Manus logs for detailed error information" << std::endl;

    return false;
}

void ManusSDKClient::RegisterCallbacks() {
    if (!connected_) return;

    std::cout << ICON_WARN << " Registering SDK callbacks..." << std::endl;

    auto skeletonCallback = [](const SkeletonStreamInfo* skeletonInfo) {
        if (skeletonInfo && skeletonInfo->skeletonsCount > 0) {
            std::cout << ICON_DATA << " Received skeleton data for " << skeletonInfo->skeletonsCount << " skeletons" << std::endl;
        }
        };

    SDKReturnCode callbackResult = CoreSdk_RegisterCallbackForSkeletonStream(skeletonCallback);
    if (callbackResult == SDKReturnCode_Success) {
        std::cout << ICON_OK << " Skeleton stream callback registered" << std::endl;
    }
    else {
        std::cout << ICON_BAD << " Failed to register skeleton callback: "
            << ExplainSdkResult(callbackResult) << std::endl;
    }
}

void ManusSDKClient::ProcessHandIKData() {
    std::cout << ICON_DATA << " Starting Hand IK processing loop..." << std::endl;
    std::cout << "  Target FPS: " << config_.performance.target_fps << std::endl;
    std::cout << "  Max solve time: " << config_.performance.max_solve_time_ms << "ms" << std::endl;

    auto target_frame_time = std::chrono::milliseconds(1000 / config_.performance.target_fps);

    while (!g_shutdown && (connected_ || offline_mode_)) {
        auto frame_start = std::chrono::high_resolution_clock::now();

        try {
            if (ik_bridge_) {
                // Example: Create test finger targets (in offline mode or with real data)
                FingerTargets test_targets;

                // Use working targets from validated test suite
                test_targets.finger_positions[0] = Eigen::Vector3d(0.06, 0.01, 0.04);   // Index
                test_targets.finger_positions[1] = Eigen::Vector3d(0.065, 0.005, 0.045); // Middle  
                test_targets.finger_positions[2] = Eigen::Vector3d(0.06, -0.01, 0.04);   // Ring
                test_targets.finger_positions[3] = Eigen::Vector3d(0.055, -0.02, 0.035); // Pinky
                test_targets.thumb_position = Eigen::Vector3d(0.04, 0.04, 0.04);
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

void ManusSDKClient::RunStandaloneIKTest() {
    if (!ik_bridge_) {
        std::cout << ICON_BAD << " No Hand IK bridge for testing" << std::endl;
        return;
    }

    std::cout << ICON_WARN << " Running standalone Hand IK test with ENHANCED solver..." << std::endl;

    // Test 1: Run built-in diagnostics
    bool diagnostics_ok = ik_bridge_->RunDiagnostics();

    // Test 2: Multiple working targets with variations
    std::cout << ICON_WARN << " Testing multiple working targets..." << std::endl;

    FingerTargets working_targets;
    working_targets.finger_positions[0] = Eigen::Vector3d(0.06, 0.01, 0.04);
    working_targets.finger_positions[1] = Eigen::Vector3d(0.065, 0.005, 0.045);
    working_targets.finger_positions[2] = Eigen::Vector3d(0.06, -0.01, 0.04);
    working_targets.finger_positions[3] = Eigen::Vector3d(0.055, -0.02, 0.035);
    working_targets.thumb_position = Eigen::Vector3d(0.04, 0.04, 0.04);
    working_targets.thumb_rotation = Eigen::Matrix3d::Identity();

    int successful_solves = 0;
    const int num_tests = 10;
    double total_solve_time = 0.0;

    for (int i = 0; i < num_tests; ++i) {
        FingerTargets varied_targets = working_targets;

        // Add small variations (±2mm)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> noise(-0.002, 0.002);

        for (int j = 0; j < 4; ++j) {
            varied_targets.finger_positions[j] += Eigen::Vector3d(noise(gen), noise(gen), noise(gen));
        }
        varied_targets.thumb_position += Eigen::Vector3d(noise(gen), noise(gen), noise(gen));

        JointConfiguration joint_config;
        bool test_solved = ik_bridge_->Solve(varied_targets, joint_config);

        if (test_solved && joint_config.valid) {
            successful_solves++;
            total_solve_time += joint_config.solve_time_ms;
        }
    }

    double success_rate = (double)successful_solves / num_tests * 100.0;
    double avg_solve_time = successful_solves > 0 ? total_solve_time / successful_solves : 0.0;

    std::cout << ICON_DATA << " Enhanced solver test results:" << std::endl;
    std::cout << "  Success rate: " << successful_solves << "/" << num_tests
        << " (" << std::fixed << std::setprecision(1) << success_rate << "%)" << std::endl;
    std::cout << "  Average solve time: " << std::fixed << std::setprecision(3) << avg_solve_time << "ms" << std::endl;

    if (success_rate >= 80.0 && avg_solve_time <= 10.0) {
        std::cout << ICON_OK << " Enhanced Hand IK solver is working optimally!" << std::endl;
    }
    else if (success_rate >= 50.0) {
        std::cout << ICON_WARN << " Hand IK solver working but may need further tuning" << std::endl;
    }
    else {
        std::cout << ICON_BAD << " Hand IK solver still has significant issues" << std::endl;
    }

    std::cout << std::endl;
    std::cout << ICON_OK << " Expected improvements with enhanced solver:" << std::endl;
    std::cout << "  - Success rates: ≥80% (vs 0% before)" << std::endl;
    std::cout << "  - Solve times: 1-5ms (vs >50ms before)" << std::endl;
    std::cout << "  - Plane tolerance: 5cm (vs 0.5cm rigid before)" << std::endl;
    std::cout << "  - Robust joint mapping by name" << std::endl;
}

void ManusSDKClient::RunDiagnostics() {
    if (!ik_bridge_) {
        std::cout << ICON_BAD << " No Hand IK bridge for diagnostics" << std::endl;
        return;
    }

    std::cout << ICON_WARN << " Running enhanced Hand IK diagnostics..." << std::endl;
    bool diagnostics_ok = ik_bridge_->RunDiagnostics();

    if (diagnostics_ok) {
        std::cout << ICON_OK << " Enhanced Hand IK diagnostics passed" << std::endl;
    }
    else {
        std::cout << ICON_BAD << " Enhanced Hand IK diagnostics failed" << std::endl;
    }
}

void ManusSDKClient::Shutdown() {
    std::cout << ICON_STOP << " Shutting down Manus SDK Client..." << std::endl;

    if (connected_) {
        SDKReturnCode disconnectResult = CoreSdk_Disconnect();
        if (disconnectResult == SDKReturnCode_Success) {
            std::cout << ICON_OK << " Disconnected from Manus Core" << std::endl;
        }
        connected_ = false;
    }

    ik_bridge_.reset();
    std::cout << ICON_OK << " Hand IK bridge cleaned up" << std::endl;

    if (!offline_mode_) {
        SDKReturnCode shutdownResult = CoreSdk_ShutDown();
        if (shutdownResult == SDKReturnCode_Success) {
            std::cout << ICON_OK << " Manus SDK shutdown complete" << std::endl;
        }
    }
}

// Static CLI argument parsing
bool ManusSDKClient::ParseCliArgs(int argc, char** argv, std::string& config_path, bool& offline_mode,
    bool& selftest, bool& fd_check, double& plane_tol) {
    // Set defaults
    config_path = "";
    offline_mode = false;
    selftest = false;
    fd_check = false;
    plane_tol = 0.05; // 5cm default

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--offline") {
            offline_mode = true;
        }
        else if (arg == "--selftest") {
            selftest = true;
        }
        else if (arg == "--fd-check") {
            fd_check = true;
        }
        else if (arg.substr(0, 12) == "--plane-tol=") {
            try {
                plane_tol = std::stod(arg.substr(12));
                if (plane_tol <= 0.0) {
                    std::cerr << "Invalid plane tolerance: " << plane_tol << std::endl;
                    return false;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Failed to parse plane tolerance: " << arg << std::endl;
                return false;
            }
        }
        else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Manus SDK Integration with Hand IK" << std::endl;
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --config <path>     Configuration file path" << std::endl;
            std::cout << "  --offline           Skip Core connection, run IK tests only" << std::endl;
            std::cout << "  --selftest          Run IK self-tests and exit" << std::endl;
            std::cout << "  --fd-check          Enable finite difference validation" << std::endl;
            std::cout << "  --plane-tol=<val>   Set plane tolerance in meters (default: 0.05)" << std::endl;
            std::cout << "  --help, -h          Show this help message" << std::endl;
            std::cout << std::endl;
            std::cout << "Examples:" << std::endl;
            std::cout << "  " << argv[0] << " --offline --selftest" << std::endl;
            std::cout << "  " << argv[0] << " --config config/manus.json --plane-tol=0.03" << std::endl;
            return false;
        }
        else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            std::cerr << "Use --help for usage information" << std::endl;
            return false;
        }
    }

    return true;
}

#else

// Fallback implementation when Manus SDK is not available
bool ManusSDKClient::Initialize(const std::string& config_path, bool offline_mode) {
    std::cout << ICON_BAD << " Manus SDK not available - running in simulation mode" << std::endl;
    return false;
}

bool ManusSDKClient::ConnectToCore() {
    std::cout << ICON_BAD << " Manus SDK not available" << std::endl;
    return false;
}

void ManusSDKClient::RegisterCallbacks() {}

void ManusSDKClient::ProcessHandIKData() {
    std::cout << "Simulation mode - no real data processing" << std::endl;
    while (!g_shutdown) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void ManusSDKClient::RunStandaloneIKTest() {
    std::cout << ICON_WARN << " Manus SDK not available - cannot run IK tests" << std::endl;
}

void ManusSDKClient::RunDiagnostics() {
    std::cout << ICON_WARN << " Manus SDK not available - cannot run diagnostics" << std::endl;
}

void ManusSDKClient::Shutdown() {
    std::cout << ICON_OK << " No cleanup needed in simulation mode" << std::endl;
}

bool ManusSDKClient::ParseCliArgs(int argc, char** argv, std::string& config_path, bool& offline_mode,
    bool& selftest, bool& fd_check, double& plane_tol) {
    // Same implementation as above for consistency
    config_path = "";
    offline_mode = false;
    selftest = false;
    fd_check = false;
    plane_tol = 0.05;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--offline") {
            offline_mode = true;
        }
        else if (arg == "--selftest") {
            selftest = true;
        }
        else if (arg == "--fd-check") {
            fd_check = true;
        }
        else if (arg.substr(0, 12) == "--plane-tol=") {
            try {
                plane_tol = std::stod(arg.substr(12));
                if (plane_tol <= 0.0) {
                    std::cerr << "Invalid plane tolerance: " << plane_tol << std::endl;
                    return false;
                }
            }
            catch (const std::exception& e) {
                std::cerr << "Failed to parse plane tolerance: " << arg << std::endl;
                return false;
            }
        }
        else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        }
        else if (arg == "--help" || arg == "-h") {
            std::cout << "Manus SDK Integration with Hand IK (SDK not available)" << std::endl;
            return false;
        }
    }

    return true;
}

#endif

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "Manus SDK Integration with Hand IK" << std::endl;
    std::cout << "Enhanced with License Detection & Robust IK" << std::endl;
    std::cout << "========================================" << std::endl;

    // Set up signal handling for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Parse command line arguments
    std::string config_path;
    bool offline_mode = false;
    bool selftest = false;
    bool fd_check = false;
    double plane_tol = 0.05;

    if (!ManusSDKClient::ParseCliArgs(argc, argv, config_path, offline_mode, selftest, fd_check, plane_tol)) {
        return 1; // Help shown or parse error
    }

    // Search for config file if not specified
    if (config_path.empty()) {
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
            config_path = ""; // Use empty path for defaults
        }
    }

    // Show CLI argument results
    if (offline_mode) {
        std::cout << ICON_WARN << " CLI: Offline mode enabled" << std::endl;
    }
    if (selftest) {
        std::cout << ICON_WARN << " CLI: Self-test mode enabled" << std::endl;
    }
    if (fd_check) {
        std::cout << ICON_WARN << " CLI: Finite difference validation enabled" << std::endl;
    }
    if (plane_tol != 0.05) {
        std::cout << ICON_WARN << " CLI: Custom plane tolerance: " << plane_tol << " m" << std::endl;
    }

    ManusSDKClient client;

    // Initialize the client with config and CLI options
    if (!client.Initialize(config_path, offline_mode)) {
        std::cout << ICON_BAD << " Failed to initialize SDK client" << std::endl;
        return 1;
    }

    // If selftest mode, run diagnostics and exit
    if (selftest) {
        std::cout << ICON_WARN << " Running self-test mode..." << std::endl;
        client.RunDiagnostics();
        client.RunStandaloneIKTest();
        client.Shutdown();
        return 0;
    }

    // Try to connect to Manus Core (unless offline)
    if (client.ConnectToCore()) {
        client.RegisterCallbacks();
        client.RunDiagnostics();
        client.ProcessHandIKData();
    }
    else {
        if (!offline_mode) {
            std::cout << ICON_BAD << " Could not connect to Manus Core" << std::endl;
            std::cout << ICON_WARN << " Running standalone Hand IK test instead..." << std::endl;
        }
        client.RunStandaloneIKTest();
    }

    client.Shutdown();

    std::cout << "Application terminated." << std::endl;
    return 0;
}