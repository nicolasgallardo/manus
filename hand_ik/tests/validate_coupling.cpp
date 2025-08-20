#include "hand_ik.hpp"
#include "math_constants.hpp"
#include <iostream>
#include <iomanip>

using namespace hand_ik;

// Your measured data points from Excel (in degrees)
const std::vector<std::pair<double, double>> measured_data = {
    {0, 0},
    {5, 10.1},
    {10, 11.7},
    {15, 14.8},
    {20, 19},
    {25, 24.15},
    {30, 30},
    {35, 36.35},
    {40, 42.9},
    {45, 49.6},
    {50, 56.27},
    {55, 62.8},
    {60, 69.3},
    {65, 75.6},
    {70, 81.8},
    {75, 87.8},
    {80, 93.8},
    {85, 99.7},
    {90, 105.6}
};

// Enhanced PassiveCoeffs for testing different models and units
struct TestPassiveCoeffs {
    double a, b, c, d;
    bool use_degrees;  // If true, evaluate in degrees; if false, in radians

    TestPassiveCoeffs(double a_, double b_, double c_, double d_, bool degrees = true)
        : a(a_), b(b_), c(c_), d(d_), use_degrees(degrees) {
    }

    double eval(double q_input) const {
        if (use_degrees) {
            // Input and output in degrees
            return a * q_input * q_input * q_input + b * q_input * q_input + c * q_input + d;
        }
        else {
            // Input and output in radians (for compatibility with existing code)
            return a * q_input * q_input * q_input + b * q_input * q_input + c * q_input + d;
        }
    }

    double derivative(double q_input) const {
        if (use_degrees) {
            return 3 * a * q_input * q_input + 2 * b * q_input + c;
        }
        else {
            return 3 * a * q_input * q_input + 2 * b * q_input + c;
        }
    }
};

void testModel(const std::string& model_name, const TestPassiveCoeffs& coeffs) {
    std::cout << "\n=== Testing " << model_name << " ===" << std::endl;
    std::cout << "Coefficients (" << (coeffs.use_degrees ? "degrees" : "radians") << " domain):" << std::endl;
    std::cout << "  a (cubic): " << coeffs.a << std::endl;
    std::cout << "  b (quadratic): " << coeffs.b << std::endl;
    std::cout << "  c (linear): " << coeffs.c << std::endl;
    std::cout << "  d (constant): " << coeffs.d << std::endl;
    std::cout << std::endl;

    std::cout << std::setw(8) << "MCP(deg)"
        << std::setw(12) << "Measured(deg)"
        << std::setw(12) << "Computed(deg)"
        << std::setw(12) << "Error(deg)"
        << std::setw(12) << "Error(%)" << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    double total_abs_error = 0.0;
    double max_abs_error = 0.0;
    double total_rel_error = 0.0;
    int num_points = 0;

    for (const auto& point : measured_data) {
        double mcp_deg = point.first;
        double measured_dip_deg = point.second;

        double computed_dip_deg;
        if (coeffs.use_degrees) {
            // Direct evaluation in degrees
            computed_dip_deg = coeffs.eval(mcp_deg);
        }
        else {
            // Convert to radians, evaluate, convert back
            double mcp_rad = degToRad(mcp_deg);
            double computed_dip_rad = coeffs.eval(mcp_rad);
            computed_dip_deg = radToDeg(computed_dip_rad);
        }

        double abs_error = std::abs(computed_dip_deg - measured_dip_deg);
        double rel_error = measured_dip_deg > 0 ? (abs_error / measured_dip_deg) * 100.0 : 0.0;

        total_abs_error += abs_error;
        max_abs_error = std::max(max_abs_error, abs_error);
        total_rel_error += rel_error;
        num_points++;

        std::cout << std::setw(8) << std::fixed << std::setprecision(1) << mcp_deg
            << std::setw(12) << std::setprecision(1) << measured_dip_deg
            << std::setw(12) << std::setprecision(1) << computed_dip_deg
            << std::setw(12) << std::setprecision(2) << abs_error
            << std::setw(12) << std::setprecision(1) << rel_error << "%"
            << std::endl;
    }

    std::cout << std::string(60, '-') << std::endl;
    std::cout << "Error Statistics:" << std::endl;
    std::cout << "  Average absolute error: " << std::setprecision(2)
        << (total_abs_error / num_points) << " degrees" << std::endl;
    std::cout << "  Maximum absolute error: " << std::setprecision(2)
        << max_abs_error << " degrees" << std::endl;
    std::cout << "  Average relative error: " << std::setprecision(1)
        << (total_rel_error / num_points) << "%" << std::endl;

    // Check acceptance criteria
    double avg_abs_error = total_abs_error / num_points;
    bool avg_error_ok = (avg_abs_error < 2.0);
    bool max_error_ok = (max_abs_error < 5.0);
    bool acceptable = avg_error_ok && max_error_ok;

    std::cout << std::endl;
    std::cout << "Acceptance Criteria:" << std::endl;
    std::cout << "  Avg error < 2.0°: " << (avg_error_ok ? "PASS" : "FAIL")
        << " (" << avg_abs_error << "°)" << std::endl;
    std::cout << "  Max error < 5.0°: " << (max_error_ok ? "PASS" : "FAIL")
        << " (" << max_abs_error << "°)" << std::endl;
    std::cout << "  Overall: " << (acceptable ? "PASS" : "FAIL") << std::endl;
}

int main(int argc, char** argv) {
    try {
        std::cout << "=== Passive Coupling Validation - Multiple Models ===" << std::endl;
        std::cout << "Comparing measured data with different polynomial models" << std::endl;

        // Test 1: SOURCE-OF-TRUTH quadratic model in degrees 
        std::cout << "\n### SOURCE-OF-TRUTH: Quadratic Model in Degrees ###" << std::endl;
        TestPassiveCoeffs source_of_truth{
            0.0,          // No cubic term
            0.00239207,   // Quadratic coefficient (fitted from data) - LOCKED
            0.97203665,   // Linear coefficient (fitted from data) - LOCKED  
            0.73983459,   // Constant offset (fitted from data) - LOCKED
            true  // degrees
        };
        testModel("SOURCE-OF-TRUTH Quadratic in Degrees", source_of_truth);

        // Test 2: Converted to radians (what the solver uses)
        std::cout << "\n### Solver Coefficients (Radians Domain) ###" << std::endl;
        // Correct unit conversion: y_rad = (beta*(180/pi))*x_rad^2 + gamma*x_rad + (delta*(pi/180))
        double beta = 0.00239207;
        double gamma = 0.97203665;
        double delta = 0.73983459;

        TestPassiveCoeffs solver_coeffs{
            0.0,                           // No cubic term
            beta * radToDeg(1.0),         // b = beta*(180/pi) ≈ 0.137056
            gamma,                         // c = gamma (unchanged) 
            delta * degToRad(1.0),        // d = delta*(pi/180) ≈ 0.0129125
            false  // radians
        };
        testModel("Solver Coefficients in Radians", solver_coeffs);

        // Verify the solver coefficients match expected values
        constexpr double b_expected = 0.137056;
        constexpr double c_expected = 0.972037;
        constexpr double d_expected = 0.0129125;
        constexpr double tol = 1e-6;

        bool coeffs_ok = (std::abs(solver_coeffs.b - b_expected) < tol &&
            std::abs(solver_coeffs.c - c_expected) < tol &&
            std::abs(solver_coeffs.d - d_expected) < tol);

        std::cout << "\n### Coefficient Verification ###" << std::endl;
        std::cout << "Expected solver coeffs: b=" << b_expected << ", c=" << c_expected << ", d=" << d_expected << std::endl;
        std::cout << "Computed solver coeffs: b=" << solver_coeffs.b << ", c=" << solver_coeffs.c << ", d=" << solver_coeffs.d << std::endl;
        std::cout << "Coefficients match: " << (coeffs_ok ? "PASS" : "FAIL") << std::endl;

        if (!coeffs_ok) {
            std::cerr << "FATAL: Solver coefficients don't match expected values!" << std::endl;
            return 1;
        }

        // Test 3: Original broken model (for comparison)
        std::cout << "\n### Original Broken Model (for reference) ###" << std::endl;
        TestPassiveCoeffs broken_model{
            0.0124 * degToRad(1.0),     // Wrong: cubic coefficient incorrectly converted
            0.0,                        // Wrong: missing quadratic term
            0.622 * degToRad(1.0),      // Wrong: linear coefficient incorrectly converted
            3.0025 * degToRad(1.0),     // Wrong: constant incorrectly converted
            false  // radians
        };
        testModel("Original Broken Model", broken_model);

        // Test 4: If we really wanted a cubic model in degrees
        std::cout << "\n### Cubic Model in Degrees (for completeness) ###" << std::endl;
        TestPassiveCoeffs cubic_deg{
            0.0024,       // Cubic coefficient from original Excel
            0.0,          // No quadratic (to match simplified form)
            0.972,        // Linear coefficient
            0.739,        // Constant
            true  // degrees
        };
        testModel("Cubic in Degrees", cubic_deg);

        std::cout << "\n=== Final Summary ===" << std::endl;
        std::cout << "SOURCE-OF-TRUTH (degrees): y_deg = 0.00239207*x_deg^2 + 0.97203665*x_deg + 0.73983459" << std::endl;
        std::cout << "SOLVER (radians): q_distal = 0.137056*q_mcp^2 + 0.972037*q_mcp + 0.0129125" << std::endl;
        std::cout << "Original broken model: major errors due to unit conversion mistakes" << std::endl;

        std::cout << "\n=== Recommendations ===" << std::endl;
        std::cout << "1. Lock in the source-of-truth coefficients shown above" << std::endl;
        std::cout << "2. Use the correctly converted radian coefficients in the solver" << std::endl;
        std::cout << "3. Add assertions in tests to detect any coefficient drift" << std::endl;

        std::cout << "\n=== Unit Conversion Formula Reference ===" << std::endl;
        std::cout << "For polynomial y_deg = beta*x_deg^2 + gamma*x_deg + delta" << std::endl;
        std::cout << "To get y_rad = a*x_rad^3 + b*x_rad^2 + c*x_rad + d:" << std::endl;
        std::cout << "  a = 0 (no cubic term)" << std::endl;
        std::cout << "  b = beta*(180/pi)" << std::endl;
        std::cout << "  c = gamma" << std::endl;
        std::cout << "  d = delta*(pi/180)" << std::endl;

        return 0;

    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}