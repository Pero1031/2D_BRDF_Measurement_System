find_package(Catch2 3 CONFIG REQUIRED)

add_executable(brdf_tests
    test_math.cpp
    test_kinematics.cpp
    test_angles.cpp
    test_old_system_golden.cpp
    test_segment.cpp
    test_rig_safety.cpp
    test_brdf_and_camera.cpp
    test_sim_arm.cpp
    test_sim_imager.cpp
    test_setup_config.cpp
    test_recipe_file.cpp
    test_preflight.cpp
    test_session.cpp
    test_commands.cpp
    test_calibration.cpp
    test_calibrate.cpp
    test_spot_xml.cpp
    test_topcon_recipe_xml.cpp
    test_topcon_imager.cpp
    test_rc8_arm.cpp
    test_real_run.cpp
    test_analysis.cpp
    test_view.cpp
    test_analyze_command.cpp
    test_headers.cpp
)

target_link_libraries(brdf_tests PRIVATE brdf::core brdf::vision Catch2::Catch2WithMain)

# 旧システムの出力から切り出した照合用データ(tests/data/)の場所をテストに教える
target_compile_definitions(brdf_tests PRIVATE
    BRDF_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")

include(Catch)
catch_discover_tests(brdf_tests)
