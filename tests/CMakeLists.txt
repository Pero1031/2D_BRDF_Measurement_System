find_package(Catch2 3 CONFIG REQUIRED)

add_executable(brdf_tests
    test_math.cpp
    test_kinematics.cpp
    test_angles.cpp
    test_old_system_golden.cpp
    test_headers.cpp
)

target_link_libraries(brdf_tests PRIVATE brdf::core Catch2::Catch2WithMain)

# 旧システムの出力から切り出した照合用データ(tests/data/)の場所をテストに教える
target_compile_definitions(brdf_tests PRIVATE
    BRDF_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

include(Catch)
catch_discover_tests(brdf_tests)
