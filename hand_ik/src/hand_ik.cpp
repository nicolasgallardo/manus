#include "hand_ik.hpp"
#include "math_constants.hpp"
#include <pinocchio/algorithm/model.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <limits>

namespace hand_ik {

    // Use project constants instead of platform-specific M_PI
    using hand_ik::kPi;
    using hand_ik::degToRad;
    using hand_ik::radToDeg;

    HandIK::HandIK(const HandIKConfig& config, const std::string& urdf_path)
        : config_(config), data_(model_) {

        // Load URDF into Pinocchio model
        pinocchio::urdf::buildModel(urdf_path, model_);

        // If model has a free-flyer, create a reduced model that locks the base
        if (model_.joints[1].shortname() == "JointModelFreeFlyer") {
            // Create reduced model without free-flyer
            std::vector<pinocchio::JointIndex> joints_to_lock = { 1 }; // lock first joint (free-flyer)
            pinocchio::buildReducedModel(model_, joints_to_lock, Eigen::VectorXd::Zero(6), model_);
        }

        data_ = pinocchio::Data(model_);

        // Initialize state vectors
        qa_ = Eigen::VectorXd::Zero(N_ACTIVE);
        qfull_ = Eigen::VectorXd::Zero(model_.nq);

        // Resolve joint and frame indices by name
        resolveJointAndFrameIndices();

        if (config_.verbose) {
            std::cout << "HandIK initialized with " << model_.nq << " configuration variables\n";
            std::cout << "Active parameters: " << N_ACTIVE << std::endl;
            std::cout << "Model nv (velocity): " << model_.nv << std::endl;
        }

        // Smoke test for passive coefficients
        static constexpr double kTestMcpDeg = 10.0;
        static constexpr double kTestMcpRad = degToRad(kTestMcpDeg); // 10 degrees in radians
        double computed_dip_rad = config_.passive_coeffs[0].eval(kTestMcpRad);
        double computed_dip_deg = radToDeg(computed_dip_rad);
        constexpr double expected_dip_deg = 0.00239207 * kTestMcpDeg * kTestMcpDeg + 0.97203665 * kTestMcpDeg + 0.73983459;

        if (std::abs(computed_dip_deg - expected_dip_deg) > 0.5) {
            throw std::runtime_error("Passive coupling coefficients validation failed in constructor");
        }

        if (config_.verbose) {
            std::cout << "Passive coupling validation: MCP=" << kTestMcpDeg << "° -> DIP=" << computed_dip_deg
                << "° (expected=" << expected_dip_deg << "°)" << std::endl;
        }
    }

    void HandIK::resolveJointAndFrameIndices() {
        // Resolve MCP and distal joint indices
        for (int i = 0; i < 4; ++i) {
            if (!model_.existJointName(config_.mcp_joint_names[i])) {
                throw std::runtime_error("MCP joint not found: " + config_.mcp_joint_names[i]);
            }
            if (!model_.existJointName(config_.distal_joint_names[i])) {
                throw std::runtime_error("Distal joint not found: " + config_.distal_joint_names[i]);
            }

            mcp_joint_ids_[i] = model_.getJointId(config_.mcp_joint_names[i]);
            distal_joint_ids_[i] = model_.getJointId(config_.distal_joint_names[i]);

            // Try to find the fingertip frame, fallback to distal joint frame if needed
            if (model_.existFrame(config_.fingertip_frame_names[i])) {
                fingertip_frame_ids_[i] = model_.getFrameId(config_.fingertip_frame_names[i]);
            }
            else {
                // Fallback: use the distal joint frame as fingertip
                std::cout << "Warning: Fingertip frame " << config_.fingertip_frame_names[i]
                    << " not found, using distal joint frame instead" << std::endl;
                    fingertip_frame_ids_[i] = model_.getFrameId(config_.distal_joint_names[i]);
            }
        }

        // Resolve thumb joint indices
        if (!model_.existJointName(config_.thumb_rot_joint_name)) {
            throw std::runtime_error("Thumb rotation joint not found: " + config_.thumb_rot_joint_name);
        }
        if (!model_.existJointName(config_.thumb_flex_joint_name)) {
            throw std::runtime_error("Thumb flexion joint not found: " + config_.thumb_flex_joint_name);
        }

        thumb_rot_joint_id_ = model_.getJointId(config_.thumb_rot_joint_name);
        thumb_flex_joint_id_ = model_.getJointId(config_.thumb_flex_joint_name);

        // Try to find the thumb tip frame, fallback to thumb joint frame if needed
        if (model_.existFrame(config_.thumb_tip_frame_name)) {
            thumb_tip_frame_id_ = model_.getFrameId(config_.thumb_tip_frame_name);
        }
        else {
            std::cout << "Warning: Thumb tip frame " << config_.thumb_tip_frame_name
                << " not found, using thumb joint frame instead" << std::endl;
            thumb_tip_frame_id_ = model_.getFrameId(config_.thumb_flex_joint_name);
        }
    }

    Eigen::VectorXd HandIK::expandActiveToFull(const Eigen::VectorXd& qa) const {
        if (qa.size() != N_ACTIVE) {
            throw std::runtime_error("Active parameter vector must have size " + std::to_string(N_ACTIVE));
        }

        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(model_.nq);

        // Set MCP joints and compute corresponding distal joints
        for (int i = 0; i < 4; ++i) {
            double q_mcp = qa[i];
            double q_distal = config_.passive_coeffs[i].eval(q_mcp);

            // Set the joint values (assuming single-DOF revolute joints)
            int mcp_q_idx = model_.joints[mcp_joint_ids_[i]].idx_q();
            int distal_q_idx = model_.joints[distal_joint_ids_[i]].idx_q();

            q_full[mcp_q_idx] = q_mcp;
            q_full[distal_q_idx] = q_distal;

            if (config_.verbose) {
                std::cout << "Finger " << i << ": MCP q_idx=" << mcp_q_idx << " (q=" << q_mcp
                    << "), Distal q_idx=" << distal_q_idx << " (q=" << q_distal << ")" << std::endl;
            }
        }

        // Set thumb joints directly
        int thumb_rot_q_idx = model_.joints[thumb_rot_joint_id_].idx_q();
        int thumb_flex_q_idx = model_.joints[thumb_flex_joint_id_].idx_q();

        q_full[thumb_rot_q_idx] = qa[4];  // thumb rotation
        q_full[thumb_flex_q_idx] = qa[5]; // thumb flexion

        if (config_.verbose) {
            std::cout << "Thumb: rot q_idx=" << thumb_rot_q_idx << " (q=" << qa[4]
                << "), flex q_idx=" << thumb_flex_q_idx << " (q=" << qa[5] << ")" << std::endl;
            std::cout << "Full q vector: " << q_full.transpose() << std::endl;
        }

        return q_full;
    }

    void HandIK::computeForwardKinematics(const Eigen::VectorXd& q) const {
        if (q.size() != model_.nq) {
            throw std::runtime_error("Configuration vector size mismatch: expected " +
                std::to_string(model_.nq) + ", got " + std::to_string(q.size()));
        }

        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::computeJointJacobians(model_, data_);
        pinocchio::updateGlobalPlacements(model_, data_);

        if (config_.verbose) {
            std::cout << "FK: qfull size=" << q.size() << ", model.nq=" << model_.nq
                << ", model.nv=" << model_.nv << std::endl;
        }
    }

    void HandIK::updateInternalState(const Eigen::VectorXd& qa) {
        qa_ = qa;
        qfull_ = expandActiveToFull(qa);
        computeForwardKinematics(qfull_);
    }

    PlaneSet HandIK::computePlanes(const Eigen::VectorXd& qa_seed) const {
        PlaneSet planes;
        const double delta = 1e-4; // Small perturbation for numerical differentiation

        // Update kinematics at the seed configuration
        Eigen::VectorXd q_full = expandActiveToFull(qa_seed);
        computeForwardKinematics(q_full);

        if (config_.verbose) {
            std::cout << "Computing frozen planes at qa: " << qa_seed.transpose() << std::endl;
        }

        for (int i = 0; i < 4; ++i) {
            // Get baseline fingertip position
            Eigen::Vector3d tip_base = data_.oMf[fingertip_frame_ids_[i]].translation();

            // Perturb MCP joint to estimate motion direction
            Eigen::VectorXd qa_plus = qa_seed;
            Eigen::VectorXd qa_minus = qa_seed;

            qa_plus[i] += delta;
            qa_minus[i] -= delta;

            // Clamp to limits
            qa_plus[i] = std::max(config_.mcp_limits[i][0],
                std::min(config_.mcp_limits[i][1], qa_plus[i]));
            qa_minus[i] = std::max(config_.mcp_limits[i][0],
                std::min(config_.mcp_limits[i][1], qa_minus[i]));

            // Compute perturbed fingertip positions
            Eigen::VectorXd q_plus = expandActiveToFull(qa_plus);
            computeForwardKinematics(q_plus);
            Eigen::Vector3d tip_plus = data_.oMf[fingertip_frame_ids_[i]].translation();

            Eigen::VectorXd q_minus = expandActiveToFull(qa_minus);
            computeForwardKinematics(q_minus);
            Eigen::Vector3d tip_minus = data_.oMf[fingertip_frame_ids_[i]].translation();

            // Estimate motion direction (tangent to the flexion arc)
            Eigen::Vector3d motion_dir = (tip_plus - tip_minus);
            double motion_magnitude = motion_dir.norm();

            if (motion_magnitude < 1e-8) {
                // Fallback: use a default flexion axis (roughly Y)
                planes.flexion_axes[i] = Eigen::Vector3d::UnitY();
                planes.mcp_origins[i] = tip_base - 0.1 * Eigen::Vector3d::UnitX();
                if (config_.verbose) {
                    std::cout << "  Finger " << i << ": minimal motion, using default axis" << std::endl;
                }
            }
            else {
                motion_dir.normalize();

                // Estimate center of rotation (MCP joint center)
                Eigen::Vector3d chord = tip_plus - tip_minus;
                Eigen::Vector3d midpoint = (tip_plus + tip_minus) * 0.5;
                Eigen::Vector3d to_base = tip_base - midpoint;

                double chord_length = chord.norm();
                double angle_change = (qa_plus[i] - qa_minus[i]);

                if (chord_length > 1e-6 && std::abs(angle_change) > 1e-6) {
                    // Estimate radius using small angle approximation
                    double radius = chord_length / (2.0 * std::sin(std::abs(angle_change) / 2.0));

                    // The perpendicular direction in the plane of motion
                    Eigen::Vector3d perpendicular = to_base - (to_base.dot(motion_dir)) * motion_dir;
                    if (perpendicular.norm() > 1e-6) {
                        perpendicular.normalize();
                        planes.mcp_origins[i] = tip_base - radius * perpendicular;
                    }
                    else {
                        // Fallback: use a point close to the fingertip
                        planes.mcp_origins[i] = tip_base - 0.1 * to_base;
                    }
                }
                else {
                    // Fallback for very small motion
                    planes.mcp_origins[i] = tip_base - 0.1 * to_base;
                }

                // The flexion axis is perpendicular to the motion direction and the radial direction
                Eigen::Vector3d radial_dir = (tip_base - planes.mcp_origins[i]).normalized();
                Eigen::Vector3d flexion_axis = motion_dir.cross(radial_dir);

                if (flexion_axis.norm() < 1e-6) {
                    // Fallback: use default axis
                    planes.flexion_axes[i] = Eigen::Vector3d::UnitY();
                    if (config_.verbose) {
                        std::cout << "  Finger " << i << ": degenerate cross product, using default axis" << std::endl;
                    }
                }
                else {
                    planes.flexion_axes[i] = flexion_axis.normalized();

                    // Ensure consistent orientation (flexion axis roughly in +Y direction)
                    if (planes.flexion_axes[i].y() < 0) {
                        planes.flexion_axes[i] *= -1.0;
                    }
                }

                if (config_.verbose) {
                    std::cout << "  Finger " << i << ": axis=" << planes.flexion_axes[i].transpose()
                        << ", origin=" << planes.mcp_origins[i].transpose() << std::endl;
                }
            }

            // Compute projection matrix: P = I - n*n^T
            planes.projection_matrices[i] = Eigen::Matrix3d::Identity() -
                planes.flexion_axes[i] * planes.flexion_axes[i].transpose();
        }

        // Restore original configuration
        computeForwardKinematics(q_full);

        return planes;
    }

    void HandIK::checkOutOfPlaneTargets(const Targets& targets, const PlaneSet& planes,
        SolveReport& report) const {
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d to_target = targets.p_fingers[i] - planes.mcp_origins[i];
            double out_of_plane_dist = std::abs(to_target.dot(planes.flexion_axes[i]));

            report.out_of_plane_distances[i] = out_of_plane_dist;
            report.out_of_plane_flags[i] = (out_of_plane_dist > config_.plane_tolerance);
        }
    }

    void HandIK::buildPlaneProjectedResidual(const Targets& targets, const PlaneSet& planes,
        Eigen::VectorXd& residual) const {
        // Count residual dimensions: 3 per finger + 3 for thumb pos + (3 for thumb rot if specified)
        int residual_dim = 3 * 4 + 3; // fingers + thumb position
        bool use_thumb_orientation = targets.R_thumb.has_value();
        if (use_thumb_orientation) {
            residual_dim += 3;
        }

        residual.resize(residual_dim);
        residual.setZero();
        int row_idx = 0;

        // Process each finger with plane projection
        for (int finger = 0; finger < 4; ++finger) {
            // Current fingertip position
            Eigen::Vector3d p_current = data_.oMf[fingertip_frame_ids_[finger]].translation();

            // Raw target position
            Eigen::Vector3d p_target_raw = targets.p_fingers[finger];

            // Project target into the frozen plane: p_target_projected = p_raw - [(p_raw - o) · n] * n
            Eigen::Vector3d to_target = p_target_raw - planes.mcp_origins[finger];
            double out_of_plane_component = to_target.dot(planes.flexion_axes[finger]);
            Eigen::Vector3d p_target_projected = p_target_raw - out_of_plane_component * planes.flexion_axes[finger];

            // Plane-projected residual: r_i = w_i * P_i * (p_target_projected - p_current)
            // Note: P_i * p_target_projected = p_target_projected since it's already in the plane
            // So we can directly compute: r_i = w_i * P_i * (p_target_raw - p_current)
            Eigen::Vector3d res_finger = planes.projection_matrices[finger] * (p_target_raw - p_current);

            residual.segment<3>(row_idx) = config_.finger_weights[finger] * res_finger;
            row_idx += 3;
        }

        // Process thumb position (no plane projection for thumb)
        Eigen::Vector3d p_thumb_current = data_.oMf[thumb_tip_frame_id_].translation();
        Eigen::Vector3d thumb_pos_residual = targets.p_thumb - p_thumb_current;

        residual.segment<3>(row_idx) = config_.thumb_pos_weight * thumb_pos_residual;
        row_idx += 3;

        // Process thumb orientation if specified
        if (use_thumb_orientation) {
            Eigen::Matrix3d R_current = data_.oMf[thumb_tip_frame_id_].rotation();

            // CRITICAL FIX: Use consistent convention with Jacobian
            // residual = log3(R_current^T * R_target) matches -J * delta_q convention
            Eigen::Matrix3d R_error = R_current.transpose() * targets.R_thumb.value();
            Eigen::Vector3d rot_residual = pinocchio::log3(R_error);

            residual.segment<3>(row_idx) = config_.thumb_rot_weight * rot_residual;
        }
    }

    void HandIK::buildPlaneProjectedJacobian(const Targets& targets, const PlaneSet& planes,
        Eigen::MatrixXd& J_red, Eigen::VectorXd& residual) const {

        // Build residual first using the frozen planes
        buildPlaneProjectedResidual(targets, planes, residual);

        // Initialize Jacobian
        J_red.resize(residual.size(), N_ACTIVE);
        J_red.setZero();

        int row_idx = 0;
        bool use_thumb_orientation = targets.R_thumb.has_value();

        if (config_.verbose) {
            std::cout << "Building analytical Jacobian (PROPER Pinocchio method):" << std::endl;
            std::cout << "  Residual size: " << residual.size() << std::endl;
            std::cout << "  Model nv: " << model_.nv << std::endl;
        }

        // CRITICAL FIX: Use proper Pinocchio analytical Jacobian computation
        // Process each finger with frozen plane projection
        for (int finger = 0; finger < 4; ++finger) {
            // Get frame Jacobian at fingertip frame in LOCAL_WORLD_ALIGNED
            Eigen::Matrix<double, 6, Eigen::Dynamic> J_frame(6, model_.nv);
            pinocchio::getFrameJacobian(model_, data_, fingertip_frame_ids_[finger],
                pinocchio::LOCAL_WORLD_ALIGNED, J_frame);

            // Extract translational part (3x model_.nv)
            auto J_trans = J_frame.topRows(3);

            // Get velocity column indices for MCP and distal joints
            int mcp_v_idx = model_.joints[mcp_joint_ids_[finger]].idx_v();
            int distal_v_idx = model_.joints[distal_joint_ids_[finger]].idx_v();

            if (config_.verbose) {
                std::cout << "  Finger " << finger << ": mcp_v_idx=" << mcp_v_idx
                    << ", distal_v_idx=" << distal_v_idx << std::endl;
            }

            // Extract individual Jacobian columns (these are analytical, not FD!)
            Eigen::Vector3d J_mcp = J_trans.col(mcp_v_idx);
            Eigen::Vector3d J_distal = J_trans.col(distal_v_idx);

            // Chain rule: effective Jacobian column for active MCP parameter
            // q_distal = passive_coupling(q_mcp), so dq_distal/dq_mcp = derivative
            double q_mcp = qfull_[model_.joints[mcp_joint_ids_[finger]].idx_q()];
            double df_dq = config_.passive_coeffs[finger].derivative(q_mcp);

            // CRITICAL: The effective Jacobian for the active MCP joint is:
            // J_eff = J_mcp + (df/dq_mcp) * J_distal
            // This accounts for both direct motion of MCP and indirect motion via passive distal
            Eigen::Vector3d J_eff = J_mcp + df_dq * J_distal;

            if (config_.verbose) {
                std::cout << "    q_mcp=" << q_mcp << ", df_dq=" << df_dq << std::endl;
                std::cout << "    J_mcp norm=" << J_mcp.norm() << ", J_distal norm=" << J_distal.norm()
                    << ", J_eff norm=" << J_eff.norm() << std::endl;
            }

            // Apply frozen plane projection: J_projected = w_i * P_i * J_eff
            Eigen::Vector3d J_projected = planes.projection_matrices[finger] * J_eff;

            if (config_.verbose) {
                std::cout << "    J_projected norm=" << J_projected.norm()
                    << ", weight=" << config_.finger_weights[finger] << std::endl;
            }

            // Set the Jacobian column with weight and correct sign for (target - current)
            J_red.block<3, 1>(row_idx, finger) = -config_.finger_weights[finger] * J_projected;
            row_idx += 3;
        }

        // Process thumb position using proper analytical Jacobian
        {
            Eigen::Matrix<double, 6, Eigen::Dynamic> J_thumb(6, model_.nv);
            pinocchio::getFrameJacobian(model_, data_, thumb_tip_frame_id_,
                pinocchio::LOCAL_WORLD_ALIGNED, J_thumb);

            auto J_thumb_trans = J_thumb.topRows(3);
            int thumb_rot_v_idx = model_.joints[thumb_rot_joint_id_].idx_v();
            int thumb_flex_v_idx = model_.joints[thumb_flex_joint_id_].idx_v();

            if (config_.verbose) {
                std::cout << "  Thumb: rot_v_idx=" << thumb_rot_v_idx
                    << ", flex_v_idx=" << thumb_flex_v_idx << std::endl;
                std::cout << "    J_thumb_rot norm=" << J_thumb_trans.col(thumb_rot_v_idx).norm()
                    << ", J_thumb_flex norm=" << J_thumb_trans.col(thumb_flex_v_idx).norm() << std::endl;
            }

            // Thumb position Jacobian columns (no plane projection for thumb)
            // CRITICAL FIX: Use negative sign to match residual convention (target - current)
            J_red.block<3, 1>(row_idx, 4) = -config_.thumb_pos_weight * J_thumb_trans.col(thumb_rot_v_idx);
            J_red.block<3, 1>(row_idx, 5) = -config_.thumb_pos_weight * J_thumb_trans.col(thumb_flex_v_idx);
            row_idx += 3;
        }

        // Process thumb orientation using proper analytical Jacobian
        if (use_thumb_orientation) {
            Eigen::Matrix<double, 6, Eigen::Dynamic> J_thumb(6, model_.nv);
            pinocchio::getFrameJacobian(model_, data_, thumb_tip_frame_id_,
                pinocchio::LOCAL_WORLD_ALIGNED, J_thumb);

            auto J_thumb_rot = J_thumb.bottomRows(3);
            int thumb_rot_v_idx = model_.joints[thumb_rot_joint_id_].idx_v();
            int thumb_flex_v_idx = model_.joints[thumb_flex_joint_id_].idx_v();

            if (config_.verbose) {
                std::cout << "    Thumb orientation Jacobian:" << std::endl;
                std::cout << "      rot column norm=" << J_thumb_rot.col(thumb_rot_v_idx).norm() << std::endl;
                std::cout << "      flex column norm=" << J_thumb_rot.col(thumb_flex_v_idx).norm() << std::endl;
            }

            // CRITICAL FIX: For orientation, use consistent sign convention
            // The Jacobian needs to match the log3(R_current^T * R_target) residual
            J_red.block<3, 1>(row_idx, 4) = -config_.thumb_rot_weight * J_thumb_rot.col(thumb_rot_v_idx);
            J_red.block<3, 1>(row_idx, 5) = -config_.thumb_rot_weight * J_thumb_rot.col(thumb_flex_v_idx);
        }

        if (config_.verbose) {
            std::cout << "  Final Jacobian norm: " << J_red.norm() << std::endl;
            std::cout << "  Final Jacobian max element: " << J_red.array().abs().maxCoeff() << std::endl;
        }
    }

    void HandIK::clampActiveJoints(Eigen::VectorXd& qa) const {
        // Clamp MCP joints
        for (int i = 0; i < 4; ++i) {
            qa[i] = std::max(config_.mcp_limits[i][0],
                std::min(config_.mcp_limits[i][1], qa[i]));
        }

        // Clamp thumb joints
        qa[4] = std::max(config_.thumb_rot_limits[0],
            std::min(config_.thumb_rot_limits[1], qa[4]));
        qa[5] = std::max(config_.thumb_flex_limits[0],
            std::min(config_.thumb_flex_limits[1], qa[5]));
    }

    bool HandIK::solve(const Targets& targets, Eigen::VectorXd& qa_out, SolveReport* report) {
        if (qa_out.size() != N_ACTIVE) {
            qa_out = Eigen::VectorXd::Zero(N_ACTIVE);
        }

        Eigen::VectorXd qa = qa_out;
        clampActiveJoints(qa);

        SolveReport local_report;
        SolveReport& rep = report ? *report : local_report;

        // CRITICAL: Compute frozen planes once at the seed configuration - same as tests
        PlaneSet frozen_planes = computePlanes(qa);

        if (config_.verbose) {
            std::cout << "Solver using frozen planes computed at seed qa: " << qa.transpose() << std::endl;
        }

        // Check for out-of-plane targets using the frozen planes
        checkOutOfPlaneTargets(targets, frozen_planes, rep);

        // Early exit if any target is out of plane (strict approach)
        bool any_out_of_plane = false;
        for (int i = 0; i < 4; ++i) {
            if (rep.out_of_plane_flags[i]) {
                any_out_of_plane = true;
                if (config_.verbose) {
                    std::cout << "Target for finger " << i << " is out of plane by "
                        << rep.out_of_plane_distances[i] << " m (tolerance: "
                        << config_.plane_tolerance << " m)" << std::endl;
                }
            }
        }

        if (any_out_of_plane) {
            if (config_.verbose) {
                std::cout << "Early exit due to out-of-plane targets" << std::endl;
            }
            qa_out = qa;
            rep.final_error = computeTargetError(targets, frozen_planes);
            return false;
        }

        double damping = config_.damping_init;

        for (int iter = 0; iter < config_.max_iterations; ++iter) {
            // Update internal state and kinematics for current qa
            updateInternalState(qa);

            // CRITICAL: Build Jacobian and residual using the same frozen planes as tests
            Eigen::MatrixXd J_red;
            Eigen::VectorXd residual;
            buildPlaneProjectedJacobian(targets, frozen_planes, J_red, residual);

            double error = residual.norm();
            if (iter == 0) {
                rep.initial_error = error;
            }

            rep.error_history.push_back(error);

            if (config_.verbose) {
                logIteration(iter, error, 0.0, damping);
            }

            // Check convergence
            if (error < config_.residual_tolerance) {
                rep.converged = true;
                break;
            }

            // Solve damped least squares: (J^T J + λI) Δqa = -J^T e
            Eigen::MatrixXd H = J_red.transpose() * J_red +
                damping * Eigen::MatrixXd::Identity(N_ACTIVE, N_ACTIVE);
            Eigen::VectorXd g = J_red.transpose() * residual;

            // Check for numerical issues
            Eigen::LDLT<Eigen::MatrixXd> ldlt(H);
            if (ldlt.info() != Eigen::Success) {
                if (config_.verbose) {
                    std::cout << "LDLT decomposition failed, increasing damping" << std::endl;
                }
                damping *= config_.damping_factor;
                continue;
            }

            Eigen::VectorXd delta_qa = -ldlt.solve(g);

            if (delta_qa.norm() < config_.step_tolerance) {
                if (config_.verbose) {
                    std::cout << "Step size below tolerance, converged" << std::endl;
                }
                break;
            }

            // Backtracking line search using frozen planes for error evaluation
            double alpha = 1.0;
            Eigen::VectorXd qa_new;
            double error_new = std::numeric_limits<double>::max();
            bool line_search_success = false;

            for (int ls = 0; ls < config_.max_line_search_steps; ++ls) {
                qa_new = qa + alpha * delta_qa;
                clampActiveJoints(qa_new);

                // Evaluate new error using the same frozen planes
                updateInternalState(qa_new);
                error_new = computeTargetError(targets, frozen_planes);

                if (error_new < error) {
                    line_search_success = true;
                    break;
                }
                alpha *= config_.line_search_factor;
            }

            // Update or increase damping based on line search success
            if (line_search_success) {
                qa = qa_new;
                damping /= config_.damping_factor;
                damping = std::max(damping, config_.damping_init);  // Don't go below initial
            }
            else {
                damping *= config_.damping_factor;
                if (config_.verbose) {
                    std::cout << "Line search failed, increasing damping to " << damping << std::endl;
                }
            }

            rep.iterations = iter + 1;
        }

        qa_out = qa;
        rep.final_error = computeTargetError(targets, frozen_planes);

        if (config_.verbose) {
            std::cout << "Solve completed: converged=" << rep.converged
                << ", final_error=" << rep.final_error
                << ", iterations=" << rep.iterations << std::endl;
        }

        return rep.converged;
    }

    double HandIK::computeTargetError(const Targets& targets, const PlaneSet& planes) const {
        Eigen::VectorXd residual;
        buildPlaneProjectedResidual(targets, planes, residual);
        return residual.norm();
    }

    void HandIK::logIteration(int iter, double error, double step_norm, double damping) const {
        std::cout << "Iter " << std::setw(3) << iter
            << ": error = " << std::scientific << std::setprecision(3) << error
            << ", λ = " << damping << std::endl;
    }

    // Accessors
    Eigen::Vector3d HandIK::getFingerTip(int finger_idx) const {
        if (finger_idx < 0 || finger_idx >= 4) {
            throw std::out_of_range("Finger index must be 0-3");
        }
        return data_.oMf[fingertip_frame_ids_[finger_idx]].translation();
    }

    Eigen::Vector3d HandIK::getThumbTip() const {
        return data_.oMf[thumb_tip_frame_id_].translation();
    }

    Eigen::Matrix3d HandIK::getThumbOrientation() const {
        return data_.oMf[thumb_tip_frame_id_].rotation();
    }

    // Jacobian finite difference check - must match solver exactly  
    bool HandIK::checkJacobianFiniteDiff(const Eigen::VectorXd& qa, double tolerance) const {
        // Step 1: Update internal state and ensure FK is computed
        const_cast<HandIK*>(this)->updateInternalState(qa);

        // Step 2: Compute frozen planes at this configuration - same as solver does
        PlaneSet frozen_planes = computePlanes(qa);

        // Step 3: Create targets OFFSET from current fingertip positions
        // This ensures non-zero residual and meaningful Jacobian
        Targets test_targets;
        const double target_offset = 0.01; // 1cm offset to create meaningful residual

        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d current_tip = getFingerTip(i);
            // Add small offset in X direction to create meaningful residual
            test_targets.p_fingers[i] = current_tip + target_offset * Eigen::Vector3d::UnitX();
        }
        test_targets.p_thumb = getThumbTip() + target_offset * Eigen::Vector3d::UnitY();
        test_targets.R_thumb = getThumbOrientation(); // Keep orientation unchanged

        // Step 4: Build analytic Jacobian using frozen planes - exactly what solver uses
        Eigen::MatrixXd J_analytic;
        Eigen::VectorXd residual_base;
        buildPlaneProjectedJacobian(test_targets, frozen_planes, J_analytic, residual_base);

        // Step 5: Compute finite difference Jacobian using the same frozen planes
        const double eps = 1e-6;  // Match the prompt specification
        Eigen::MatrixXd J_fd = Eigen::MatrixXd::Zero(J_analytic.rows(), N_ACTIVE);

        for (int j = 0; j < N_ACTIVE; ++j) {
            // Perturb active parameter by +/- eps
            Eigen::VectorXd qa_plus = qa;
            Eigen::VectorXd qa_minus = qa;
            qa_plus[j] += eps;
            qa_minus[j] -= eps;

            // Clamp to limits to avoid discontinuities
            const_cast<HandIK*>(this)->clampActiveJoints(qa_plus);
            const_cast<HandIK*>(this)->clampActiveJoints(qa_minus);

            // Compute residuals using frozen planes - key: same planes, only qa changes
            const_cast<HandIK*>(this)->updateInternalState(qa_plus);
            Eigen::VectorXd res_plus;
            buildPlaneProjectedResidual(test_targets, frozen_planes, res_plus);

            const_cast<HandIK*>(this)->updateInternalState(qa_minus);
            Eigen::VectorXd res_minus;
            buildPlaneProjectedResidual(test_targets, frozen_planes, res_minus);

            // Central difference: dR/dqa[j] = (R(qa + eps*e_j) - R(qa - eps*e_j)) / (2*eps)
            J_fd.col(j) = (res_plus - res_minus) / (2.0 * eps);
        }

        // Step 6: Compare analytical vs finite difference
        Eigen::MatrixXd diff = J_analytic - J_fd;
        double max_error = diff.array().abs().maxCoeff();

        // Step 7: Detailed diagnostics
        if (config_.verbose) {
            std::cout << "Jacobian FD check: max error = " << std::scientific << max_error << std::endl;
            std::cout << "Base residual norm: " << residual_base.norm() << std::endl;
            std::cout << "Target offset used: " << target_offset << " m" << std::endl;

            // Report per-column errors for diagnostics
            std::cout << "Per-column errors:" << std::endl;
            for (int j = 0; j < N_ACTIVE; ++j) {
                double col_error = (J_analytic.col(j) - J_fd.col(j)).norm();
                double col_norm_analytic = J_analytic.col(j).norm();
                double col_norm_fd = J_fd.col(j).norm();
                std::cout << "  Column " << j << ": error=" << col_error
                    << ", analytic_norm=" << col_norm_analytic
                    << ", fd_norm=" << col_norm_fd << std::endl;
            }

            // Additional debugging: Check if analytical Jacobian is computed properly
            std::cout << "Analytical J matrix stats:" << std::endl;
            std::cout << "  Size: " << J_analytic.rows() << "x" << J_analytic.cols() << std::endl;
            std::cout << "  Norm: " << J_analytic.norm() << std::endl;
            std::cout << "  Max element: " << J_analytic.array().abs().maxCoeff() << std::endl;

            std::cout << "FD J matrix stats:" << std::endl;
            std::cout << "  Size: " << J_fd.rows() << "x" << J_fd.cols() << std::endl;
            std::cout << "  Norm: " << J_fd.norm() << std::endl;
            std::cout << "  Max element: " << J_fd.array().abs().maxCoeff() << std::endl;
        }

        // Step 8: Restore original state
        const_cast<HandIK*>(this)->updateInternalState(qa);

        return max_error < tolerance;
    }

    bool HandIK::testReachability(int num_tests, double noise_scale) const {
        int successful_tests = 0;

        for (int test = 0; test < num_tests; ++test) {
            // Generate random reachable configuration
            Eigen::VectorXd qa_true = Eigen::VectorXd::Random(N_ACTIVE) * 0.5;
            const_cast<HandIK*>(this)->clampActiveJoints(qa_true);

            // Compute forward kinematics to get target positions
            const_cast<HandIK*>(this)->updateInternalState(qa_true);

            Targets targets;
            for (int i = 0; i < 4; ++i) {
                targets.p_fingers[i] = getFingerTip(i);
            }
            targets.p_thumb = getThumbTip();
            targets.R_thumb = getThumbOrientation();

            // Start from perturbed initial guess
            Eigen::VectorXd qa_init = qa_true + noise_scale * Eigen::VectorXd::Random(N_ACTIVE);
            const_cast<HandIK*>(this)->clampActiveJoints(qa_init);

            // Solve IK
            Eigen::VectorXd qa_solved = qa_init;
            SolveReport report;
            bool success = const_cast<HandIK*>(this)->solve(targets, qa_solved, &report);

            if (success&& report.final_error < 1e-3) {
                successful_tests++;
            }
        }

        double success_rate = double(successful_tests) / num_tests;
        if (config_.verbose) {
            std::cout << "Reachability test: " << successful_tests << "/" << num_tests
                << " (" << (success_rate * 100) << "%) successful" << std::endl;
        }

        return success_rate > 0.8; // 80% success rate threshold
    }

} // namespace hand_ik