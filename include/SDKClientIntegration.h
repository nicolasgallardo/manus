#pragma once

#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <random>
#include <iomanip>
#include <sstream>

// Manus SDK headers
#ifdef MANUS_SDK_AVAILABLE
#include "ManusSDK.h"
#endif

// Hand IK integration
#include "ManusHandIKBridge.h"

// ASCII replacements for emoji to avoid C4566 codepage warnings
#define ICON_OK   "[OK]"
#define ICON_BAD  "[FAIL]" 
#define ICON_WARN "[WARN]"
#define ICON_ROCKET "[START]"
#define ICON_LINK "[CONN]"
#define ICON_STOP "[STOP]"
#define ICON_DATA "[STATS]"
#define ICON_LICENSE "[LIC]"

// Global shutdown flag
extern std::atomic<bool> g_shutdown;

// Signal handler for graceful shutdown
void signalHandler(int signal);

#ifdef MANUS_SDK_AVAILABLE

class ManusSDKClient {
private:
    uint32_t session_id_;
    bool connected_;
    bool offline_mode_;
    std::unique_ptr<ManusHandIKBridge> ik_bridge_;
    ManusIntegrationConfig config_;

    // License and status helpers
    bool CheckLicenseStatus();
    std::string ExplainSdkResult(SDKReturnCode code);
    bool DetectLicenseDongle();
    
    // Connection helpers with license awareness
    bool ConfigureCorePresetRoute(const std::string& host, uint16_t port);
    bool ConnectToCore_WithPresetRoute();
    bool PerformLicensePreflight();

public:
    ManusSDKClient();
    ~ManusSDKClient();

    bool Initialize(const std::string& config_path, bool offline_mode = false);
    bool ConnectToCore();
    void RegisterCallbacks();
    void ProcessHandIKData();
    void RunStandaloneIKTest();
    void RunDiagnostics();
    void Shutdown();

    // CLI argument processing
    static bool ParseCliArgs(int argc, char** argv, std::string& config_path, bool& offline_mode, 
                           bool& selftest, bool& fd_check, double& plane_tol);
};

#else

// Fallback implementation when Manus SDK is not available
class ManusSDKClient {
public:
    bool Initialize(const std::string& config_path, bool offline_mode = false);
    bool ConnectToCore();
    void RegisterCallbacks();
    void ProcessHandIKData();
    void RunStandaloneIKTest();
    void RunDiagnostics();
    void Shutdown();

    static bool ParseCliArgs(int argc, char** argv, std::string& config_path, bool& offline_mode, 
                           bool& selftest, bool& fd_check, double& plane_tol);
};

#endif