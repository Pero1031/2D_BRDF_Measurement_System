/**
 * @file src/Math/MathUtils.hpp
 * @brief Mathematical constants and utility functions for BRDF calculations.
 * @details
 * Provides:
 * - High-precision constants via C++20 <numbers>
 * - Unit conversions (Degrees <-> Radians)
 * - Spherical coordinate mapping (essential for BRDF optics)
 * - Scalar interpolation and clamping
 */

#pragma once

#include <cmath>
#include <algorithm> // for std::clamp
#include <numbers>   // C++20: Standard mathematical constants

#include "Core/Types.hpp"
#include "Math/Algebra.hpp" // Required for Vec3 return types

namespace brdf {

    /** @name Constants
     * Mathematical constants using C++20 std::numbers for maximum precision
     * relative to the 'Real' type (float or double).
     */
     ///@{

     /// Pi (3.14159...)
    constexpr Real PI = std::numbers::pi_v<Real>;

    /// 1 / Pi (Useful for normalization factors in BRDF integration)
    constexpr Real INV_PI = std::numbers::inv_pi_v<Real>;

    /// Pi / 2 (90 degrees in radians)
    constexpr Real HALF_PI = Real(0.5) * PI;

    /// 2 * Pi (360 degrees in radians)
    constexpr Real TWO_PI = Real(2.0) * PI;

    /// Small epsilon for floating-point comparisons to avoid division by zero.
    constexpr Real EPSILON = Real(1e-6);

    ///@}


    /** @name Angle Conversions
     * Helpers to interface between hardware (Degrees) and math/Eigen (Radians).
     */
     ///@{

     /**
      * @brief Converts degrees to radians.
      * @param deg Angle in degrees.
      * @return Angle in radians.
      */
    template <typename T>
    inline constexpr T toRadians(T deg) {
        return deg * (PI / T(180));
    }

    /**
     * @brief Converts radians to degrees.
     * @param rad Angle in radians.
     * @return Angle in degrees.
     */
    template <typename T>
    inline constexpr T toDegrees(T rad) {
        return rad * (T(180) / PI);
    }
    ///@}


    /** @name Coordinate System Utilities
     * Functions specific to optical geometry.
     */
     ///@{

     /**
      * @brief Converts spherical coordinates to a Cartesian direction vector (Z-up).
      * * @details
      * Uses a standard optical/physics coordinate system:
      * - **Z-axis**: Surface normal (Zenith).
      * - **Theta**: Polar angle from Z-axis [0, PI].
      * - **Phi**: Azimuthal angle around Z-axis [0, 2*PI].
      * * @param theta Polar angle in radians.
      * @param phi Azimuthal angle in radians.
      * @return Normalized 3D direction vector (Vec3).
      */
    inline Vec3 sphericalToCartesian(Real theta, Real phi) {
        // Optimizing by calculating sine once
        Real sinTheta = std::sin(theta);

        return Vec3(
            sinTheta * std::cos(phi), // x
            sinTheta * std::sin(phi), // y
            std::cos(theta)           // z
        );
    }
    ///@}


    /** @ name Scalar Math Utilities
     * General purpose math helpers.
     */
     ///@{

     /**
      * @brief Linearly interpolates between a and b.
      * @note Wraps C++20 std::lerp where appropriate or uses standard formula.
      */
    template <typename T>
    inline T lerp(T a, T b, T t) {
        // C++20 introduces std::lerp in <cmath>, but simple implementation 
        // is often sufficient for basic types.
        return a + t * (b - a);
    }

    /**
     * @brief Clamps a value to the range [0, 1].
     * @param val Input value.
     * @return Value clamped between 0.0 and 1.0.
     */
    template <typename T>
    inline T saturate(T val) {
        return std::clamp(val, T(0), T(1));
    }

    ///@}

} // namespace brdf