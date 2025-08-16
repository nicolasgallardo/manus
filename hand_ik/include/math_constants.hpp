#pragma once

namespace hand_ik {

    // Mathematical constants for cross-platform compatibility
    // MSVC doesn't define platform Pi constants by default, so we provide our own
    constexpr double kPi = 3.141592653589793238462643383279502884L;
    constexpr double kTwoPi = 2.0 * kPi;
    constexpr double kHalfPi = 0.5 * kPi;

    // Conversion utilities
    constexpr double degToRad(double degrees) {
        return degrees * kPi / 180.0;
    }

    constexpr double radToDeg(double radians) {
        return radians * 180.0 / kPi;
    }

} // namespace hand_ik