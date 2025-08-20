#pragma once

#include <filesystem>
#include <string>
#include <vector>
#include <cstdlib>
#include <stdexcept>
#include <iostream>

namespace hand_ik {

/**
 * @brief Utility for resolving URDF file paths at runtime
 * 
 * Searches in this order:
 * 1. Command line argument: --urdf <path-or-basename>
 * 2. Environment variable: HAND_IK_URDF
 * 3. Fallback: <exe_dir>/surge_v13_hand_right_pybullet.urdf
 */
class UrdfResolver {
public:
    /**
     * @brief Parse command line arguments for --urdf option
     * @param argc Number of command line arguments
     * @param argv Command line arguments
     * @return URDF path from --urdf option, or empty string if not found
     */
    static std::string parseUrdfArg(int argc, char** argv) {
        for (int i = 1; i < argc - 1; ++i) {
            if (std::string(argv[i]) == "--urdf") {
                return std::string(argv[i + 1]);
            }
        }
        return "";
    }

    /**
     * @brief Ensure path has .urdf extension
     * @param s Input path or basename
     * @return Path with .urdf extension
     */
    static std::filesystem::path ensureUrdfExt(const std::string& s) {
        std::filesystem::path p(s);
        if (p.extension().empty()) {
            p.replace_extension(".urdf");
        }
        return p;
    }

    /**
     * @brief Resolve URDF file path using CLI args, env vars, and fallbacks
     * @param argc Number of command line arguments
     * @param argv Command line arguments
     * @return Absolute path to existing URDF file
     * @throws std::runtime_error if URDF file not found
     */
    static std::filesystem::path resolveUrdf(int argc, char** argv) {
        std::vector<std::filesystem::path> searchPaths;
        std::vector<std::string> searchSources;

        // 1. Command line argument: --urdf <path-or-basename>
        std::string argUrdf = parseUrdfArg(argc, argv);
        if (!argUrdf.empty()) {
            auto path = ensureUrdfExt(argUrdf);
            searchPaths.push_back(path);
            searchSources.push_back("CLI argument --urdf");
            if (std::filesystem::exists(path)) {
                std::cout << "[URDF] Resolved from CLI: " << std::filesystem::absolute(path) << std::endl;
                return std::filesystem::absolute(path);
            }
        }

        // 2. Environment variable: HAND_IK_URDF
        const char* envUrdf = std::getenv("HAND_IK_URDF");
        if (envUrdf) {
            auto path = ensureUrdfExt(envUrdf);
            searchPaths.push_back(path);
            searchSources.push_back("Environment variable HAND_IK_URDF");
            if (std::filesystem::exists(path)) {
                std::cout << "[URDF] Resolved from env HAND_IK_URDF: " << std::filesystem::absolute(path) << std::endl;
                return std::filesystem::absolute(path);
            }
        }

        // 3. Fallback: <exe_dir>/surge_v13_hand_right_pybullet.urdf
        if (argc > 0) {
            std::filesystem::path exeDir = std::filesystem::path(argv[0]).parent_path();
            auto path = exeDir / "surge_v13_hand_right_pybullet.urdf";
            searchPaths.push_back(path);
            searchSources.push_back("Executable directory fallback");
            if (std::filesystem::exists(path)) {
                std::cout << "[URDF] Resolved from fallback: " << std::filesystem::absolute(path) << std::endl;
                return std::filesystem::absolute(path);
            }
        }

        // If we get here, URDF was not found anywhere
        std::string errorMsg = "URDF file not found!\n\nChecked locations:\n";
        for (size_t i = 0; i < searchPaths.size(); ++i) {
            errorMsg += "  " + searchSources[i] + ": " + searchPaths[i].string() + "\n";
        }
        
        errorMsg += "\nRemediation options:\n";
        errorMsg += "  1. Provide --urdf <path> as command line argument\n";
        errorMsg += "  2. Set environment variable: export HAND_IK_URDF=<path>\n";
        errorMsg += "  3. Place 'surge_v13_hand_right_pybullet.urdf' next to the executable\n";
        errorMsg += "  4. Verify CMake copied the URDF during build\n";

        throw std::runtime_error(errorMsg);
    }

    /**
     * @brief Validate that the resolved URDF file is readable
     * @param urdfPath Path to URDF file
     * @throws std::runtime_error if file cannot be read
     */
    static void validateUrdf(const std::filesystem::path& urdfPath) {
        if (!std::filesystem::exists(urdfPath)) {
            throw std::runtime_error("URDF file does not exist: " + urdfPath.string());
        }
        
        if (!std::filesystem::is_regular_file(urdfPath)) {
            throw std::runtime_error("URDF path is not a regular file: " + urdfPath.string());
        }
        
        // Try to read the file size (basic accessibility check)
        std::error_code ec;
        auto fileSize = std::filesystem::file_size(urdfPath, ec);
        if (ec) {
            throw std::runtime_error("Cannot read URDF file: " + urdfPath.string() + " (" + ec.message() + ")");
        }
        
        if (fileSize == 0) {
            throw std::runtime_error("URDF file is empty: " + urdfPath.string());
        }
        
        std::cout << "[URDF] Validated: " << urdfPath << " (" << fileSize << " bytes)" << std::endl;
    }

    /**
     * @brief Complete URDF resolution and validation
     * @param argc Number of command line arguments
     * @param argv Command line arguments
     * @return Absolute path to validated URDF file
     * @throws std::runtime_error if URDF file not found or invalid
     */
    static std::filesystem::path resolveAndValidate(int argc, char** argv) {
        auto urdfPath = resolveUrdf(argc, argv);
        validateUrdf(urdfPath);
        return urdfPath;
    }
};

} // namespace hand_ik