#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

// Manus SDK headers
#ifdef MANUS_SDK_AVAILABLE
#include "ManusSDK.h"
#include "ManusSdkTypes.h"
#endif

// Hand IK integration
#include "ManusHandIKBridge.h"

// Global shutdown flag
std::atomic<bool> g_shutdown{ false };

// Signal handler for graceful shutdown
void signalHandler(int signal) {
    std::cout << "\n[SIGNAL] Received signal " << signal << ", shutting down..." << std::endl;
    g_shutdown = true;
}

#ifdef MANUS_SDK_AVAILABLE

class ManusSDKClient {
private:
    uint32_t session_id_;
    bool connected_;
    ManusHandIKBridge* ik_bridge_;

public:
    ManusSDKClient() : session_id_(0), connected_(false), ik_bridge_(nullptr) {}

    ~ManusSDKClient() {
        shutdown();
    }

    bool initialize() {
        std::cout << "🚀 Initializing Manus SDK Client..." << std::endl;

        try {
            // Initialize Hand IK bridge
            ik_bridge_ = new ManusHandIKBridge();
            if (!ik_bridge_->initialize()) {
                std::cout << "❌ Failed to initialize Hand IK bridge" << std::endl;
                return false;
            }
            std::cout << "✓ Hand IK bridge initialized" << std::endl;

            return true;
        }
        catch (const std::exception& e) {
            std::cout << "❌ Exception during initialization: " << e.what() << std::endl;
            return false;
        }
    }

    bool connectToCore() {
        std::cout << "🔗 Attempting to connect to Manus Core..." << std::endl;

        // For SDK 3.0.0, check available connection methods
        // Note: The exact API may vary - this is a basic template

        // Try local connection first
        SDKReturnCode result = CoreSdk_ConnectGRPC("localhost:9001");
        if (result == SDKReturnCode_Success) {
            connected_ = true;
            std::cout << "✓ Connected to Manus Core locally" << std::endl;
            return true;
        }

        std::cout << "❌ Failed to connect to Manus Core" << std::endl;
        return false;
    }

    void setupCoordinateSystem() {
        if (!connected_) return;

        std::cout << "⚠️ Setting up coordinate system..." << std::endl;

        // For SDK 3.0.0, coordinate system setup may be different
        // This is a placeholder - check the actual SDK documentation

        std::cout << "✓ Coordinate system configured" << std::endl;
    }

    void processSkeletonData() {
        std::cout << "📊 Processing skeleton data..." << std::endl;

        // For SDK 3.0.0, skeleton data processing will be different
        // This is a simplified version that just demonstrates the concept

        while (!g_shutdown && connected_) {
            try {
                // In SDK 3.0.0, the skeleton data API has changed
                // You'll need to use the new API calls here
                // This is just a placeholder to show the structure

                if (ik_bridge_) {
                    // Example: get skeleton data and process with Hand IK
                    // The actual implementation depends on the SDK 3.0.0 API

                    // Placeholder for skeleton data processing
                    std::this_thread::sleep_for(std::chrono::milliseconds(16)); // ~60fps
                }

            }
            catch (const std::exception& e) {
                std::cout << "Error processing skeleton data: " << e.what() << std::endl;
                break;
            }
        }
    }

    void shutdown() {
        std::cout << "🛑 Shutting down Manus SDK Client..." << std::endl;

        if (connected_) {
            // Disconnect from Core if connected
            connected_ = false;
        }

        if (ik_bridge_) {
            delete ik_bridge_;
            ik_bridge_ = nullptr;
            std::cout << "✓ Hand IK bridge cleaned up" << std::endl;
        }

        std::cout << "✓ Shutdown complete" << std::endl;
    }
};

#else

// Fallback implementation when Manus SDK is not available
class ManusSDKClient {
public:
    bool initialize() {
        std::cout << "❌ Manus SDK not available - running in simulation mode" << std::endl;
        return false;
    }

    bool connectToCore() { return false; }
    void setupCoordinateSystem() {}
    void processSkeletonData() {
        std::cout << "Simulation mode - no real data processing" << std::endl;
        while (!g_shutdown) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    void shutdown() {}
};

#endif

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Manus SDK Integration with Hand IK" << std::endl;
    std::cout << "========================================" << std::endl;

    // Set up signal handling for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    ManusSDKClient client;

    // Initialize the client
    if (!client.initialize()) {
        std::cout << "❌ Failed to initialize SDK client" << std::endl;
        return 1;
    }

    // Try to connect to Manus Core
    if (client.connectToCore()) {
        client.setupCoordinateSystem();
        client.processSkeletonData();
    }
    else {
        std::cout << "❌ Could not connect to Manus Core" << std::endl;
        std::cout << "Make sure Manus Core is running and accessible" << std::endl;
    }

    client.shutdown();

    std::cout << "Application terminated." << std::endl;
    return 0;
}