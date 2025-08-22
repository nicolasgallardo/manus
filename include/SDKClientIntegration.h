#pragma once
#include <string>

struct RunnerOptions {
    std::string config_path = "config/manus_integration.json";
    bool offline_mode = false;
    bool selftest = false;
    bool fd_check = false;
    double plane_tol = 0.005; // meters
};

// Runs the app with the given options (no CLI here)
int RunWithOptions(const RunnerOptions& opt);

// Map SDK numeric result to a short string; header-agnostic
std::string SdkResultToString(int code);