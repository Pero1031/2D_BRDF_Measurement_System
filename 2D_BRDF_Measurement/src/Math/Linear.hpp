/*
* @file src/Math/Linear.hpp
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Core/Types.hpp"

namespace brdf {

	using Vec2 = Eigen::Matrix<Real, 2, 1>;
	using Vec3 = Eigen::Matrix<Real, 3, 1>;
	using Vec4 = Eigen::Matrix<Real, 4, 1>;

	using Mat2 = Eigen::Matrix<Real, 2, 2>;
	using Mat3 = Eigen::Matrix<Real, 3, 3>;
	using Mat4 = Eigen::Matrix<Real, 4, 4>;

	using Quat = Eigen::Quaternion<Real>;
	using Isometry3 = Eigen::Transform<Real, 3, Eigen::Isometry>; // 回転+並進（剛体変換）
	using Affine3 = Eigen::Transform<Real, 3, Eigen::Affine>;   // スケールも含めるなら

} // namespace brdf