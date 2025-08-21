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
    int max_iterations = 30;
    double residual_tolerance = 0.0005;
    double step_tolerance = 0.000001;
    double damping_init = 0.0001;
    double damping_factor = 2.0;
    double line_search_factor = 0.5;
    int max_line_search_steps = 6;
};

struct ManusIKWeights {
    double thumb_position = 1.0;
    double thumb_orientation = 0.1;
    std::array<double, 4> finger_weights = {1.0, 1.0, 1.0, 1.0};
    double plane_tolerance = 0.008;
};

struct ManusPassiveCoupling {
    double a = 0.0;
    double b = 0.137056;
    double c = 0.972037;
    double d = 0.0129125;
};

struct ManusPerformanceConfig {
    int target_fps = 60;
    int stats_interval_seconds = 5;
    double max_solve_time_ms = 5.0;
    int queue_size = 10;
};

struct ManusLoggingConfig {
    std::string level = "info";
    bool verbose_ik = false;
    bool performance_stats = true;
    bool skeleton_debug = false;
    bool coordinate_debug = false;
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
};