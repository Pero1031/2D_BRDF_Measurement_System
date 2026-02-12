/*
* @file include/Math/Algebra.hpp
* @brief Linear algebra type definitions based on Eigen.
* @details
* This file defines standard type aliases for vectors, matrices, quaternions,
* and transformations used throughout the BRDF measurement system.
*
* It acts as a wrapper around the Eigen library to ensure consistent precision
* (via the Real type) and to simplify template syntax in the codebase.
*
* @note All types depend on the precision defined in Core/Types.hpp (float vs double).
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Core/Types.hpp"

namespace brdf {

	using Vec2 = Eigen::Matrix<Real, 2, 1>;
	using Vec3 = Eigen::Matrix<Real, 3, 1>;
	using Vec4 = Eigen::Matrix<Real, 4, 1>;
	using VecX = Eigen::Matrix<Real, Eigen::Dynamic, 1>;

	using Mat2 = Eigen::Matrix<Real, 2, 2>;
	using Mat3 = Eigen::Matrix<Real, 3, 3>;
	using Mat4 = Eigen::Matrix<Real, 4, 4>;

	using Quat = Eigen::Quaternion<Real>;
	using Isometry3 = Eigen::Transform<Real, 3, Eigen::Isometry>; 
	using Affine3 = Eigen::Transform<Real, 3, Eigen::Affine>;   

} // namespace brdf