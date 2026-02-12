/*
* @file include/Core/types.hpp
* @brief Fundamental type definitions for the BRDF measurement system.
*/

#pragma once

#include <cstdint>  // For fixed-width integer types

namespace brdf {

	using Real = double; 

	using i32 = std::int32_t;
	using u32 = std::uint32_t;
	using i64 = std::int64_t;
	using u64 = std::uint64_t;

} // namespace brdf