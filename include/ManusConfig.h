// File: include/ManusConfig.h
#pragma once

#include "hand_ik.hpp"
#include <string>
#include <vector>
#include <array>

struct ManusConnectionConfig {
    std::string host = "127.0.0.1";
    int port = 9004;
    int timeout_ms = 5000;
    int retry_attempts = 3;
    bool offline_mode = false;  // NEW: Skip Core connection
};

struct ManusCoordinateConfig {
    std::string handedness = "right";
    std::string up_axis = "positive_y";
    std::string view_axis = "x_from_viewer";
    std::string units = "meters";
};

struct ManusHandConfig {
    bool left_enabled = false;
    bool right_enabled = true;
    std::string left_skeleton_name = "left_hand_ik";
    std::string right_skeleton_name = "right_hand_ik";
};

struct ManusIKSolverConfig {
    int max_iterations = 100;           // WORKING value from test suite
    double residual_tolerance = 1e-6;   // WORKING value from test suite
    double step_tolerance = 1e-8;       // WORKING value from test suite
    double damping_init = 1e-3;         // WORKING value from test suite
    double damping_factor = 10.0;       // WORKING value from test suite
    double line_search_factor = 0.8;    // WORKING value from test suite
    int max_line_search_steps = 10;     // WORKING value from test suite
    double plane_tolerance = 0.05;      // NEW: Relaxed default (5cm instead of 0.5cm)
    bool use_moving_planes = true;      // NEW: Recompute planes each iteration
    bool fd_check = false;              // NEW: Enable finite difference validation
    bool verbose = false;
};

struct ManusIKWeights {
    double thumb_position = 1.0;
    double thumb_orientation = 0.2;     // WORKING value from test suite
    std::array<double, 4> finger_weights = { 1.0, 1.0, 1.0, 1.0 };
};

struct ManusPassiveCoupling {
    double a = 0.0;          // VALIDATED coefficients
    double b = 0.137056;     // VALIDATED coefficients
    double c = 0.972037;     // VALIDATED coefficients
    double d = 0.0129125;    // VALIDATED coefficients
};

struct ManusPerformanceConfig {
    int target_fps = 30;                // Reduced from 60 for realistic performance
    int stats_interval_seconds = 5;
    double max_solve_time_ms = 10.0;    // Increased from 5.0 for analytical Jacobian
    int queue_size = 10;
};

struct ManusLoggingConfig {
    std::string level = "info";
    bool verbose_ik = false;
    bool performance_stats = true;
    bool skeleton_debug = false;
    bool coordinate_debug = false;
    bool license_debug = true;          // NEW: License/dongle status logging
};

struct ManusIntegrationConfig {
    ManusConnectionConfig connection;
    ManusCoordinateConfig coordinate_system;
    ManusHandConfig hands;
    std::string urdf_path = "surge_v13_hand_right_pybullet.urdf";
    ManusIKSolverConfig solver_params;
    ManusIKWeights weights;
    ManusPassiveCoupling passive_coupling;
    ManusPerformanceConfig performance;
    ManusLoggingConfig logging;

    // Load from JSON file
    static ManusIntegrationConfig LoadFromFile(const std::string& config_path);

    // Apply to hand_ik::HandIKConfig
    void ApplyToHandIKConfig(hand_ik::HandIKConfig& ik_config) const;
};

// Simple JSON parser for our specific config structure
class ManusConfigLoader {
public:
    static ManusIntegrationConfig LoadConfig(const std::string& config_path);

private:
    static std::string ReadFileToString(const std::string& filepath);
    static std::string ExtractStringValue(const std::string& json, const std::string& key);
    static double ExtractDoubleValue(const std::string& json, const std::string& key);
    static int ExtractIntValue(const std::string& json, const std::string& key);
    static bool ExtractBoolValue(const std::string& json, const std::string& key);
    static std::vector<double> ExtractDoubleArray(const std::string& json, const std::string& key);

    // Enhanced connection parsing that handles offline mode
    static ManusConnectionConfig ParseConnection(const std::string& json_content);
};