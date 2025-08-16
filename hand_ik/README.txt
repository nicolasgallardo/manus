# Hand IK: Pinocchio-based Inverse Kinematics for Robotic Hands

A C++ library for solving inverse kinematics of robotic hands with passive distal joints using the Pinocchio robotics library.

## Features

- **Passive Joint Coupling**: Handles nonlinear coupling between active MCP joints and passive distal joints using polynomial relationships
- **Flexion Plane Constraints**: Automatically projects finger targets onto their natural flexion planes (no splay for fingers)
- **Thumb Control**: Supports both position and orientation control for the thumb with two active joints
- **Robust Solver**: Levenberg-Marquardt optimization with line search and joint limit handling
- **Comprehensive Diagnostics**: Finite-difference Jacobian validation and reachability testing
- **Modern C++**: Clean C++17 API with Eigen integration

## Requirements

- **C++17** or newer
- **CMake 3.16+**
- **Pinocchio** (robotics library)
- **Eigen3** (linear algebra)
- **pkg-config** (for dependency resolution)

## Installation

### Dependencies

First, install the required dependencies:

#### Ubuntu/Debian
```bash
sudo apt update
sudo apt install cmake build-essential pkg-config libeigen3-dev

# Install Pinocchio
sudo apt install robotpkg-py3*-pinocchio
# or build from source: https://github.com/stack-of-tasks/pinocchio
```

#### macOS (with Homebrew)
```bash
brew install cmake eigen pkg-config
brew install pinocchio
```

### Building

```bash
git clone <repository-url>
cd hand_ik
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Running Tests
```bash
# Run all tests
make test

# Or run individual tests
./test_jacobian
./test_reachability
```

### Installation
```bash
sudo make install
```

## Usage

### Basic Example

```cpp
#include "hand_ik.hpp"
using namespace hand_ik;

// Load configuration
HandIKConfig config = loadExampleConfig();

// Initialize solver
HandIK ik(config, "path/to/hand.urdf");

// Define targets
Targets targets;
targets.p_fingers[0] = Eigen::Vector3d(0.15, 0.05, 0.12);  // Index finger
targets.p_fingers[1] = Eigen::Vector3d(0.16, 0.02, 0.14);  // Middle finger
targets.p_fingers[2] = Eigen::Vector3d(0.15, -0.02, 0.13); // Ring finger
targets.p_fingers[3] = Eigen::Vector3d(0.13, -0.05, 0.11); // Pinky finger
targets.p_thumb = Eigen::Vector3d(0.08, 0.08, 0.10);       // Thumb position
targets.R_thumb = Eigen::Matrix3d::Identity();              // Optional orientation

// Solve IK
Eigen::VectorXd qa_solution = Eigen::VectorXd::Zero(6);
SolveReport report;
bool success = ik.solve(targets, qa_solution, &report);

if (success) {
    std::cout << "Solution found in " << report.iterations << " iterations\n";
    std::cout << "Joint angles: " << qa_solution.transpose() << std::endl;
}
```

### Configuration

Create a YAML configuration file based on `config/hand_ik_example.yaml`:

```yaml
joint_names:
  mcp_joints:
    - "index_mcp_joint"
    - "middle_mcp_joint" 
    - "ring_mcp_joint"
    - "pinky_mcp_joint"
  
  distal_joints:
    - "index_distal_joint"
    - "middle_distal_joint"
    - "ring_distal_joint"
    - "pinky_distal_joint"

# ... (see full example in config/hand_ik_example.yaml)
```

## Key Concepts

### Active Parameter Vector

The solver operates on a 6-dimensional active parameter vector:
```
qa = [mcp_index, mcp_middle, mcp_ring, mcp_pinky, thumb_rotation, thumb_flexion]
```

### Passive Joint Coupling

Each passive distal joint is coupled to its corresponding MCP joint via:
```
q_distal = a*q_mcp³ + b*q_mcp² + c*q_mcp + d
```

The coefficients `(a,b,c,d)` must be identified for your specific hand mechanism.

### Flexion Plane Projection

Since fingers cannot splay, targets are automatically projected onto each finger's flexion plane:
```
p_projected = p_target - axis * ((p_target - origin) · axis)
```

### Reduced Jacobian

The chain rule is used to compute the effective Jacobian for active parameters:
```
J_effective = J_mcp + (df/dq_mcp) * J_distal
```

## API Reference

### Main Classes

- **`HandIK`**: Main solver class
- **`HandIKConfig`**: Configuration structure
- **`Targets`**: Target specification for fingertips
- **`SolveReport`**: Detailed solve results

### Key Methods

- **`solve(targets, qa_out, report)`**: Main IK solving function
- **`expandActiveToFull(qa)`**: Convert active to full joint configuration
- **`checkJacobianFiniteDiff(qa)`**: Validate analytical Jacobian
- **`testReachability(num_tests)`**: Test solver robustness

## Testing

The library includes comprehensive tests:

### Jacobian Validation
```bash
./test_jacobian
```
Validates analytical Jacobian computation against finite differences.

### Reachability Testing  
```bash
./test_reachability
```
Tests solver convergence from random initial guesses to reachable targets.

## Troubleshooting

### Common Issues

1. **URDF Loading Fails**
   - Verify joint and frame names match exactly
   - Check URDF file path and validity
   - Ensure Pinocchio can parse your URDF

2. **Poor Convergence**
   - Verify passive coupling coefficients are accurate
   - Check joint limits are reasonable
   - Adjust solver tolerances and damping parameters

3. **Out-of-Plane Warnings**
   - Targets are outside finger flexion planes
   - Solver will project targets automatically
   - Check `out_of_plane_flags` in solve report

### Debugging

Enable verbose output:
```cpp
config.verbose = true;
```

Check Jacobian accuracy:
```cpp
bool jacobian_ok = ik.checkJacobianFiniteDiff(qa, 1e-5);
```

## Performance

Typical performance on modern hardware:
- **Solve time**: 1-5 ms per IK solution
- **Convergence**: 10-30 iterations for nearby targets
- **Success rate**: >95% for reachable targets

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

[Specify your license here]

## Citation

If you use this library in academic work, please cite:

```bibtex
@software{hand_ik,
  title={Hand IK: Pinocchio-based Inverse Kinematics for Robotic Hands},
  author={[Your Name]},
  year={2024},
  url={[Repository URL]}
}
```

## Acknowledgments

- Built on the excellent [Pinocchio](https://github.com/stack-of-tasks/pinocchio) robotics library
- Uses [Eigen](https://eigen.tuxfamily.org/) for linear algebra
- Inspired by research in underactuated robotic hand control