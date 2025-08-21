# Manus Core + Hand IK Integration

Complete real-time integration between Manus Core gloves and the Hand IK solver using Pinocchio.

## 🚀 Quick Start

### Prerequisites

1. **Manus Core 3.0** - Download and install from [Manus Developer Portal](https://developer.manus-meta.com/)
2. **Manus Gloves** - Paired and calibrated via Manus Dashboard
3. **Visual Studio 2022** - With C++ development tools
4. **vcpkg** - Package manager for C++ dependencies

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/nicolasgallardo/manus.git
   cd manus
   ```

2. **Extract Manus SDK:**
   - Download `MANUS_Core_3.0.0_SDK.zip` from Manus Developer Portal
   - Extract to `MANUS_Core_3.0.0_SDK/` in the repository root

3. **Install vcpkg dependencies:**
   ```bash
   vcpkg install boost-filesystem boost-system boost-serialization eigen3 --triplet x64-windows
   ```

4. **Build the integration:**
   ```bash
   .\build_manus_integration.bat
   ```

### Running

1. **Start Manus Core Dashboard** and ensure gloves are connected
2. **Run the integration:**
   ```bash
   build\bin\Release\SDKClient.exe --side=right --debug
   ```

## 📋 Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `--side=left\|right\|both` | Which hand(s) to track | `right` |
| `--host=IP` | Manus Core host address | `127.0.0.1` |
| `--port=PORT` | Manus Core port | `9004` |
| `--simulate` | Use simulated motion instead of real gloves | `false` |
| `--debug` | Enable verbose debug logging | `false` |
| `--urdf=PATH` | Custom URDF file path | Auto-detected |

## 🔧 Configuration

Edit `config/manus_integration.json` to customize:

### Basic Settings
- **Connection**: Manus Core host/port
- **Coordinate System**: Right-handed, Z-up, meters
- **Active Hands**: Left, right, or both

### Hand IK Parameters
- **Solver Tolerances**: Convergence criteria for real-time performance
- **Joint Limits**: Physical constraints from URDF
- **Passive Coupling**: Validated polynomial coefficients
- **Weights**: Balance between position and orientation tracking

### Performance Tuning
- **Target FPS**: 60Hz for smooth tracking
- **Max Solve Time**: 5ms for real-time constraints
- **Queue Size**: Buffer for high-frequency data

## 🏗️ Architecture

### Components

1. **SDKClient** - Main executable coordinating everything
2. **ManusHandIKBridge** - Converts between Manus and IK coordinate systems
3. **ManusSkeletonSetup** - Creates and manages Manus hand skeletons
4. **Hand IK Library** - Pinocchio-based inverse kinematics solver

### Data Flow

```
Manus Gloves → Manus Core → SDKClient → IK Bridge → Hand IK Solver
                    ↑                               ↓
            Skeleton Updates ← Skeleton Setup ← Joint Angles
```

### Real-time Pipeline

1. **Skeleton Stream Callback** - Receives finger positions from Manus Core
2. **Coordinate Conversion** - Transform from Manus to IK space
3. **IK Solving** - Compute joint angles for target fingertip positions
4. **Skeleton Update** - Send solved joint configuration back to Manus Core

## 📊 Performance

### Typical Performance (Intel i7, Release build)
- **IK Solve Time**: 0.5-1.0 ms average
- **Success Rate**: 96-99% for reachable targets  
- **Throughput**: 60+ FPS sustained
- **Memory Usage**: ~50MB working set

### Performance Monitoring
The integration prints real-time statistics every 5 seconds:
```
📊 Performance [5s]: FPS=60.0, IK solves=299, Avg solve time=0.567ms
```

## 🧪 Testing

### Automated Tests
```bash
# Build and run all tests
cmake --build build --config Release --target RUN_TESTS

# Individual test components
build\bin\Release\test_jacobian.exe          # Validate IK mathematics
build\bin\Release\test_reachability.exe      # Validate solver robustness  
build\bin\Release\test_manus_integration.exe # Validate integration pipeline
```

### Manual Testing
1. **Connection Test**: Verify connection to Manus Core
2. **Skeleton Creation**: Check hand skeletons appear in Manus Dashboard
3. **Real-time Tracking**: Move gloves and verify fingertip nodes update
4. **Performance**: Monitor solve times stay under 5ms

## 🛠️ Troubleshooting

### Common Issues

**"Failed to connect to Manus Core"**
- Ensure Manus Core Dashboard is running
- Check firewall settings for port 9004
- Verify `--host` and `--port` parameters

**"URDF file not found"**
- Ensure `surge_v13_hand_right_pybullet.urdf` is next to SDKClient.exe
- Or specify path with `--urdf=path/to/urdf`

**"IK solve failed" or low success rates**
- Check fingertip targets are within reachable workspace
- Verify passive coupling coefficients in config
- Enable `--debug` for detailed solver diagnostics

**Poor performance / high solve times**
- Reduce `max_iterations` in config (trade accuracy for speed)
- Increase `residual_tolerance` for faster convergence
- Check CPU usage and close unnecessary applications

### Debug Logging

Enable verbose output with `--debug`:
```bash
SDKClient.exe --side=right --debug --log=verbose
```

This shows:
- Detailed IK solver iterations
- Coordinate system conversions  
- Skeleton node mappings
- Performance bottlenecks

## 🔬 Calibration

### Finger Offset Calibration
1. Set `"auto_calibrate": true` in config
2. Hold hands in natural rest pose for 5 seconds
3. Integration will compute and save finger offsets
4. Restart with auto-calibration disabled

### Manual Calibration
Edit `finger_offsets` in config file:
```json
"finger_offsets": {
  "index": [0.002, -0.001, 0.000],
  "middle": [0.000, 0.000, 0.000],
  "ring": [-0.001, 0.001, 0.000],
  "pinky": [-0.003, 0.002, 0.000],
  "thumb": [0.001, 0.000, 0.001]
}
```

## 📁 Project Structure

```
manus/
├── src/
│   ├── SDKClientIntegration.cpp      # Main integration executable
│   ├── ManusHandIKBridge.cpp         # Coordinate system bridge
│   ├── ManusSkeletonSetup.cpp        # Skeleton management
│   └── hand_ik/                      # Hand IK library sources
├── include/
│   ├── ManusHandIKBridge.h
│   ├── ManusSkeletonSetup.h
│   └── hand_ik/                      # Hand IK library headers  
├── tests/
│   ├── test_manus_integration.cpp    # Integration smoke tests
│   └── hand_ik/                      # Hand IK unit tests
├── config/
│   └── manus_integration.json        # Configuration file
├── MANUS_Core_3.0.0_SDK/            # Manus SDK (extracted)
├── build_manus_integration.bat       # Build script
└── CMakeLists.txt                    # Build configuration
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality  
4. Ensure all tests pass
5. Submit a pull request

### Development Workflow

```bash
# Make changes to integration code
edit src/SDKClientIntegration.cpp

# Rebuild and test
.\build_manus_integration.bat
build\bin\Release\test_manus_integration.exe

# Test with real gloves
build\bin\Release\SDKClient.exe --debug
```

## 📄 License

This project builds on several open-source libraries:
- **Pinocchio** - Robotics library (BSD)
- **Eigen** - Linear algebra (MPL2)  
- **Manus SDK** - Proprietary (see Manus license)

## 🆘 Support

For issues with:
- **Hand IK mathematics**: Check Pinocchio documentation and hand_ik tests
- **Manus SDK integration**: Consult Manus Developer Portal and SDK samples  
- **Build/deployment**: Review error messages and troubleshooting section above

**Performance Requirements**: This integration is designed for real-time use with solve times under 5ms. For applications requiring higher precision, increase solver iterations and tolerances in the configuration.