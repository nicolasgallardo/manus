#pragma once

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <filesystem>
#include <iostream>
#include <cstdlib>

namespace hand_ik {

    // Polynomial coefficients for passive joint coupling: q_distal = a*q^3 + b*q^2 + c*q + d
    struct PassiveCoeffs {
        double a, b, c, d;

        double eval(double q_mcp) const {
            return a * q_mcp * q_mcp * q_mcp + b * q_mcp * q_mcp + c * q_mcp + d;
        }

        double derivative(double q_mcp) const {
            return 3 * a * q_mcp * q_mcp + 2 * b * q_mcp + c;
        }
    };

    // Per-finger plane information (frozen during solve)
    struct PlaneSet {
        std::array<Eigen::Vector3d, 4> mcp_origins;      // o_i: MCP joint centers in world frame
        std::array<Eigen::Vector3d, 4> flexion_axes;     // n_i: unit flexion axes in world frame
        std::array<Eigen::Matrix3d, 4> projection_matrices; // P_i = I - n_i*n_i^T
    };

    // Configuration structure
    struct HandIKConfig {
        // Joint and frame names
        std::array<std::string, 4> mcp_joint_names;     // index, middle, ring, pinky
        std::array<std::string, 4> distal_joint_names;  // corresponding distal joints
        std::array<std::string, 4> fingertip_frame_names;

        std::string thumb_rot_joint_name;    // thumb rotation (ab/adduction)
        std::string thumb_flex_joint_name;   // thumb flexion
        std::string thumb_tip_frame_name;

        // Passive coupling coefficients for each finger
        std::array<PassiveCoeffs, 4> passive_coeffs;

        // Joint limits for driven joints [lower, upper] in radians
        std::array<std::array<double, 2>, 4> mcp_limits;  // 4 fingers
        std::array<double, 2> thumb_rot_limits;
        std::array<double, 2> thumb_flex_limits;

        // Solver parameters
        int max_iterations = 100;
        double residual_tolerance = 1e-6;
        double step_tolerance = 1e-8;
        double damping_init = 1e-6;
        double damping_factor = 10.0;
        double line_search_factor = 0.8;
        int max_line_search_steps = 10;

        // Weights and tolerances
        double thumb_pos_weight = 1.0;
        double thumb_rot_weight = 0.1;
        std::array<double, 4> finger_weights = { 1.0, 1.0, 1.0, 1.0 };
        double plane_tolerance = 1e-2;  // for out-of-plane detection

        bool verbose = false;
    };

    // Target specification
    struct Targets {
        std::array<Eigen::Vector3d, 4> p_fingers;  // index, middle, ring, pinky
        Eigen::Vector3d p_thumb;
        std::optional<Eigen::Matrix3d> R_thumb;
    };

    // Solve report
    struct SolveReport {
        int iterations = 0;
        double initial_error = 0.0;
        double final_error = 0.0;
        bool converged = false;
        bool hit_limits = false;
        std::array<bool, 4> out_of_plane_flags = { false, false, false, false };
        std::array<double, 4> out_of_plane_distances = { 0.0, 0.0, 0.0, 0.0 };
        std::vector<double> error_history;
    };

    // URDF Resolution Helper
    inline std::string resolveUrdfPath(int argc, char* argv[], const std::string& default_filename = "surge_v13_hand_right_pybullet.urdf") {
        std::vector<std::string> checked_paths;

        // 1. Check --urdf command line argument
        for (int i = 1; i < argc - 1; ++i) {
            if (std::string(argv[i]) == "--urdf") {
                std::string path = argv[i + 1];
                // Append .urdf if missing
                if (path.find(".urdf") == std::string::npos) {
                    path += ".urdf";
                }
                checked_paths.push_back(path + " (from --urdf)");
                if (std::filesystem::exists(path)) {
                    return path;
                }
            }
        }

        // 2. Check HAND_IK_URDF environment variable
        const char* env_path = std::getenv("HAND_IK_URDF");
        if (env_path) {
            std::string path = env_path;
            // Append .urdf if missing
            if (path.find(".urdf") == std::string::npos) {
                path += ".urdf";
            }
            checked_paths.push_back(path + " (from HAND_IK_URDF)");
            if (std::filesystem::exists(path)) {
                return path;
            }
        }

        // 3. Check next to executable
        if (argc > 0) {
            std::filesystem::path exe_path(argv[0]);
            std::filesystem::path exe_dir = exe_path.parent_path();
            std::filesystem::path urdf_path = exe_dir / default_filename;
            checked_paths.push_back(urdf_path.string() + " (next to exe)");
            if (std::filesystem::exists(urdf_path)) {
                return urdf_path.string();
            }
        }

        // 4. Check current working directory
        std::filesystem::path cwd_path = std::filesystem::current_path() / default_filename;
        checked_paths.push_back(cwd_path.string() + " (current directory)");
        if (std::filesystem::exists(cwd_path)) {
            return cwd_path.string();
        }

        // Fail with helpful error message
        std::cerr << "URDF file not found! Checked paths:\n";
        for (const auto& path : checked_paths) {
            std::cerr << "  - " << path << "\n";
        }
        std::cerr << "\nTo fix this:\n";
        std::cerr << "  1. Copy " << default_filename << " to the executable directory\n";
        std::cerr << "  2. Use --urdf <path> argument\n";
        std::cerr << "  3. Set HAND_IK_URDF=<path> environment variable\n";

        throw std::runtime_error("URDF file not found: " + default_filename);
    }

    // Main IK solver class
    class HandIK {
    public:
        explicit HandIK(const HandIKConfig& config, const std::string& urdf_path);

        // Main solving interface
        bool solve(const Targets& targets, Eigen::VectorXd& qa_out, SolveReport* report = nullptr);

        // Utility functions
        Eigen::VectorXd expandActiveToFull(const Eigen::VectorXd& qa) const;
        void computeForwardKinematics(const Eigen::VectorXd& q) const;  // now const with mutable data

        // Diagnostics
        bool checkJacobianFiniteDiff(const Eigen::VectorXd& qa, double tolerance = 1e-5) const;
        bool testReachability(int num_tests = 100, double noise_scale = 0.1) const;

        // Accessors
        const HandIKConfig& getConfig() const { return config_; }
        Eigen::Vector3d getFingerTip(int finger_idx) const;
        Eigen::Vector3d getThumbTip() const;
        Eigen::Matrix3d getThumbOrientation() const;

    private:
        HandIKConfig config_;
        pinocchio::Model model_;
        mutable pinocchio::Data data_;  // mutable to allow FK in const methods

        // Own the configuration state (Pinocchio Data doesn't store q)
        Eigen::VectorXd qa_;      // active vector: [4 MCP, thumb_rot, thumb_flex]
        Eigen::VectorXd qfull_;   // full model configuration

        // Stored indices for efficiency
        std::array<pinocchio::JointIndex, 4> mcp_joint_ids_;
        std::array<pinocchio::JointIndex, 4> distal_joint_ids_;
        std::array<pinocchio::FrameIndex, 4> fingertip_frame_ids_;
        pinocchio::JointIndex thumb_rot_joint_id_;
        pinocchio::JointIndex thumb_flex_joint_id_;
        pinocchio::FrameIndex thumb_tip_frame_id_;

        // Active parameter dimension (4 MCP + 2 thumb = 6)
        static constexpr int N_ACTIVE = 6;

        // Internal methods
        void resolveJointAndFrameIndices();

        // Plane computation and residual methods (unified approach)
        PlaneSet computePlanes(const Eigen::VectorXd& qa_seed) const;
        void buildPlaneProjectedResidual(const Targets& targets, const PlaneSet& planes,
            Eigen::VectorXd& residual) const;
        void buildPlaneProjectedJacobian(const Targets& targets, const PlaneSet& planes,
            Eigen::MatrixXd& J_red, Eigen::VectorXd& residual) const;

        void clampActiveJoints(Eigen::VectorXd& qa) const;
        double computeTargetError(const Targets& targets, const PlaneSet& planes) const;
        void logIteration(int iter, double error, double step_norm, double damping) const;
        void updateInternalState(const Eigen::VectorXd& qa);  // updates qa_ and qfull_

        // Out-of-plane checking
        void checkOutOfPlaneTargets(const Targets& targets, const PlaneSet& planes,
            SolveReport& report) const;
    };

    // Utility functions
    HandIKConfig loadConfigFromYAML(const std::string& yaml_path);
    void saveConfigToYAML(const HandIKConfig& config, const std::string& yaml_path);

} // namespace hand_ik