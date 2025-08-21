// File: src/ManusConfig.cpp
#include "ManusConfig.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <regex>

std::string ManusConfigLoader::ReadFileToString(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open config file: " + filepath);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

std::string ManusConfigLoader::ExtractStringValue(const std::string& json, const std::string& key) {
    std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]+)\"");
    std::smatch match;
    if (std::regex_search(json, match, pattern)) {
        return match[1].str();
    }
    return "";
}

double ManusConfigLoader::ExtractDoubleValue(const std::string& json, const std::string& key) {
    std::regex pattern("\"" + key + "\"\\s*:\\s*([0-9.+-eE]+)");
    std::smatch match;
    if (std::regex_search(json, match, pattern)) {
        return std::stod(match[1].str());
    }
    return 0.0;
}

int ManusConfigLoader::ExtractIntValue(const std::string& json, const std::string& key) {
    std::regex pattern("\"" + key + "\"\\s*:\\s*([0-9+-]+)");
    std::smatch match;
    if (std::regex_search(json, match, pattern)) {
        return std::stoi(match[1].str());
    }
    return 0;
}

bool ManusConfigLoader::ExtractBoolValue(const std::string& json, const std::string& key) {
    std::regex pattern("\"" + key + "\"\\s*:\\s*(true|false)");
    std::smatch match;
    if (std::regex_search(json, match, pattern)) {
        return match[1].str() == "true";
    }
    return false;
}

std::vector<double> ManusConfigLoader::ExtractDoubleArray(const std::string& json, const std::string& key) {
    std::regex pattern("\"" + key + "\"\\s*:\\s*\\[([^\\]]+)\\]");
    std::smatch match;
    std::vector<double> result;
    
    if (std::regex_search(json, match, pattern)) {
        std::string array_content = match[1].str();
        std::regex number_pattern("([0-9.+-eE]+)");
        std::sregex_iterator iter(array_content.begin(), array_content.end(), number_pattern);
        std::sregex_iterator end;
        
        while (iter != end) {
            result.push_back(std::stod((*iter)[1].str()));
            ++iter;
        }
    }
    
    return result;
}

ManusIntegrationConfig ManusConfigLoader::LoadConfig(const std::string& config_path) {
    std::cout << "[Config] Loading configuration from: " << config_path << std::endl;
    
    try {
        std::string json_content = ReadFileToString(config_path);
        ManusIntegrationConfig config;
        
        // Connection settings
        config.connection.host = ExtractStringValue(json_content, "host");
        if (config.connection.host.empty()) config.connection.host = "127.0.0.1";
        
        config.connection.port = ExtractIntValue(json_content, "port");
        if (config.connection.port == 0) config.connection.port = 9004;
        
        config.connection.timeout_ms = ExtractIntValue(json_content, "timeout_ms");
        if (config.connection.timeout_ms == 0) config.connection.timeout_ms = 5000;
        
        config.connection.retry_attempts = ExtractIntValue(json_content, "retry_attempts");
        if (config.connection.retry_attempts == 0) config.connection.retry_attempts = 3;
        
        // Coordinate system
        config.coordinate_system.handedness = ExtractStringValue(json_content, "handedness");
        config.coordinate_system.up_axis = ExtractStringValue(json_content, "up_axis");
        config.coordinate_system.view_axis = ExtractStringValue(json_content, "view_axis");
        config.coordinate_system.units = ExtractStringValue(json_content, "units");
        
        // Hand settings - look for right hand enabled flag
        std::string right_hand_section = json_content;
        size_t right_hand_pos = right_hand_section.find("\"right_hand\"");
        if (right_hand_pos != std::string::npos) {
            std::string right_section = right_hand_section.substr(right_hand_pos, 200);
            config.hands.right_enabled = ExtractBoolValue(right_section, "enabled");
            std::string skeleton_name = ExtractStringValue(right_section, "skeleton_name");
            if (!skeleton_name.empty()) {
                config.hands.right_skeleton_name = skeleton_name;
            }
        }
        
        // URDF path
        std::string urdf_path = ExtractStringValue(json_content, "urdf_path");
        if (!urdf_path.empty()) {
            config.urdf_path = urdf_path;
        }
        
        // IK Solver parameters
        int max_iter = ExtractIntValue(json_content, "max_iterations");
        if (max_iter > 0) config.solver_params.max_iterations = max_iter;
        
        double res_tol = ExtractDoubleValue(json_content, "residual_tolerance");
        if (res_tol > 0) config.solver_params.residual_tolerance = res_tol;
        
        double step_tol = ExtractDoubleValue(json_content, "step_tolerance");
        if (step_tol > 0) config.solver_params.step_tolerance = step_tol;
        
        double damp_init = ExtractDoubleValue(json_content, "damping_init");
        if (damp_init > 0) config.solver_params.damping_init = damp_init;
        
        double damp_factor = ExtractDoubleValue(json_content, "damping_factor");
        if (damp_factor > 0) config.solver_params.damping_factor = damp_factor;
        
        double line_search = ExtractDoubleValue(json_content, "line_search_factor");
        if (line_search > 0) config.solver_params.line_search_factor = line_search;
        
        int line_search_steps = ExtractIntValue(json_content, "max_line_search_steps");
        if (line_search_steps > 0) config.solver_params.max_line_search_steps = line_search_steps;
        
        // Weights
        double thumb_pos = ExtractDoubleValue(json_content, "thumb_position");
        if (thumb_pos > 0) config.weights.thumb_position = thumb_pos;
        
        double thumb_orient = ExtractDoubleValue(json_content, "thumb_orientation");
        if (thumb_orient >= 0) config.weights.thumb_orientation = thumb_orient;
        
        double plane_tol = ExtractDoubleValue(json_content, "plane_tolerance");
        if (plane_tol > 0) config.weights.plane_tolerance = plane_tol;
        
        auto finger_weights = ExtractDoubleArray(json_content, "finger_weights");
        if (finger_weights.size() >= 4) {
            for (int i = 0; i < 4; ++i) {
                config.weights.finger_weights[i] = finger_weights[i];
            }
        }
        
        // Passive coupling coefficients
        double coeff_a = ExtractDoubleValue(json_content, "a");
        double coeff_b = ExtractDoubleValue(json_content, "b");
        double coeff_c = ExtractDoubleValue(json_content, "c");
        double coeff_d = ExtractDoubleValue(json_content, "d");
        
        // Only use config coefficients if they appear to be set
        if (coeff_b != 0.0 || coeff_c != 0.0) {
            config.passive_coupling.a = coeff_a;
            config.passive_coupling.b = coeff_b;
            config.passive_coupling.c = coeff_c;
            config.passive_coupling.d = coeff_d;
        }
        
        // Performance settings
        int fps = ExtractIntValue(json_content, "target_fps");
        if (fps > 0) config.performance.target_fps = fps;
        
        int stats_interval = ExtractIntValue(json_content, "stats_interval_seconds");
        if (stats_interval > 0) config.performance.stats_interval_seconds = stats_interval;
        
        double max_solve_time = ExtractDoubleValue(json_content, "max_solve_time_ms");
        if (max_solve_time > 0) config.performance.max_solve_time_ms = max_solve_time;
        
        int queue_size = ExtractIntValue(json_content, "queue_size");
        if (queue_size > 0) config.performance.queue_size = queue_size;
        
        // Logging settings
        std::string log_level = ExtractStringValue(json_content, "level");
        if (!log_level.empty()) config.logging.level = log_level;
        
        config.logging.verbose_ik = ExtractBoolValue(json_content, "verbose_ik");
        config.logging.performance_stats = ExtractBoolValue(json_content, "performance_stats");
        config.logging.skeleton_debug = ExtractBoolValue(json_content, "skeleton_debug");
        config.logging.coordinate_debug = ExtractBoolValue(json_content, "coordinate_debug");
        
        std::cout << "[Config] Configuration loaded successfully" << std::endl;
        std::cout << "[Config] Connection: " << config.connection.host << ":" << config.connection.port << std::endl;
        std::cout << "[Config] URDF: " << config.urdf_path << std::endl;
        std::cout << "[Config] Max iterations: " << config.solver_params.max_iterations << std::endl;
        std::cout << "[Config] Performance stats: " << (config.logging.performance_stats ? "enabled" : "disabled") << std::endl;
        
        return config;
        
    } catch (const std::exception& e) {
        std::cerr << "[Config] Failed to load configuration: " << e.what() << std::endl;
        std::cerr << "[Config] Using default configuration" << std::endl;
        return ManusIntegrationConfig{}; // Return default config
    }
}

ManusIntegrationConfig ManusIntegrationConfig::LoadFromFile(const std::string& config_path) {
    return ManusConfigLoader::LoadConfig(config_path);
}

void ManusIntegrationConfig::ApplyToHandIKConfig(hand_ik::HandIKConfig& ik_config) const {
    std::cout << "[Config] Applying configuration to Hand IK solver..." << std::endl;
    
    // Apply solver parameters
    ik_config.max_iterations = solver_params.max_iterations;
    ik_config.residual_tolerance = solver_params.residual_tolerance;
    ik_config.step_tolerance = solver_params.step_tolerance;
    ik_config.damping_init = solver_params.damping_init;
    ik_config.damping_factor = solver_params.damping_factor;
    ik_config.line_search_factor = solver_params.line_search_factor;
    ik_config.max_line_search_steps = solver_params.max_line_search_steps;
    
    // Apply weights
    ik_config.thumb_pos_weight = weights.thumb_position;
    ik_config.thumb_rot_weight = weights.thumb_orientation;
    ik_config.plane_tolerance = weights.plane_tolerance;
    
    for (int i = 0; i < 4; ++i) {
        ik_config.finger_weights[i] = weights.finger_weights[i];
    }
    
    // Apply passive coupling coefficients (verify they match validated values)
    constexpr double tolerance = 1e-6;
    if (std::abs(passive_coupling.b - 0.137056) > tolerance ||
        std::abs(passive_coupling.c - 0.972037) > tolerance ||
        std::abs(passive_coupling.d - 0.0129125) > tolerance) {
        std::cerr << "[Config] WARNING: Passive coupling coefficients differ from validated values!" << std::endl;
        std::cerr << "[Config] Expected: b=0.137056, c=0.972037, d=0.0129125" << std::endl;
        std::cerr << "[Config] Got: b=" << passive_coupling.b << ", c=" << passive_coupling.c << ", d=" << passive_coupling.d << std::endl;
        std::cerr << "[Config] Using validated coefficients instead of config values" << std::endl;
        
        // Use validated coefficients
        for (int i = 0; i < 4; ++i) {
            ik_config.passive_coeffs[i] = {
                0.0,          // a: cubic coefficient (0.0)
                0.137056,     // b: quadratic coefficient 
                0.972037,     // c: linear coefficient
                0.0129125     // d: constant offset
            };
        }
    } else {
        // Apply config values if they match validated ones
        std::cout << "[Config] Using validated passive coupling coefficients from config" << std::endl;
        for (int i = 0; i < 4; ++i) {
            ik_config.passive_coeffs[i] = {
                passive_coupling.a,
                passive_coupling.b, 
                passive_coupling.c,
                passive_coupling.d
            };
        }
    }
    
    // Apply logging
    ik_config.verbose = logging.verbose_ik;
    
    std::cout << "[Config] Applied configuration to Hand IK solver:" << std::endl;
    std::cout << "[Config]   Max iterations: " << ik_config.max_iterations << std::endl;
    std::cout << "[Config]   Residual tolerance: " << ik_config.residual_tolerance << std::endl;
    std::cout << "[Config]   Plane tolerance: " << ik_config.plane_tolerance << std::endl;
    std::cout << "[Config]   Verbose mode: " << (ik_config.verbose ? "enabled" : "disabled") << std::endl;
}