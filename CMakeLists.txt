cmake_minimum_required(VERSION 3.25)

project(brdf_measurement
    VERSION 0.1.0
    DESCRIPTION "2D BRDF measurement system"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ソースは UTF-8(BOM付き)で統一している。MSVC には明示的に伝える。
if(MSVC)
    add_compile_options(/utf-8 /W4 /permissive- /Zc:__cplusplus)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX)
endif()

find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(tl-expected CONFIG REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)
find_package(CLI11 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)

# ---------------------------------------------------------------------------
# brdf_core: ライブラリ本体(計算・ドライバのインターフェース・モック)
# ---------------------------------------------------------------------------
add_library(brdf_core
    src/geometry/Rc8Kinematics.cpp
    src/geometry/RigGeometry.cpp
    src/geometry/Segment.cpp
    src/optics/Angles.cpp
    src/sim/BrdfModel.cpp
    src/sim/PinholeCamera.cpp
    src/sim/SimulatedRig.cpp
    src/sim/MockRobotArm.cpp
    src/sim/MockSpectralImager.cpp
    src/sim/CameraSpotProjector.cpp
    src/io/SetupConfig.cpp
    src/io/RecipeFile.cpp
    src/io/CalibrationFile.cpp
    src/io/SpotXml.cpp
    src/io/MeasurementsCsv.cpp
    src/analysis/RadianceFactor.cpp
    src/app/Analyze.cpp
    src/app/AngleTable.cpp
    src/app/Preflight.cpp
    src/app/MeasurementSession.cpp
    src/app/DeviceFactory.cpp
    src/app/ResultFiles.cpp
    src/app/ProjectLayout.cpp
    src/app/Commands.cpp
    src/calibration/BoardLayout.cpp
    src/calibration/Homography.cpp
    src/calibration/SampleToBoard.cpp
    src/calibration/HomographySpotProjector.cpp
)
add_library(brdf::core ALIAS brdf_core)

target_include_directories(brdf_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(brdf_core PUBLIC
    Eigen3::Eigen
    spdlog::spdlog
    tl::expected
    tomlplusplus::tomlplusplus)

# toml++ のヘッダは src/io 内だけで使う(公開ヘッダには出さない)が、リンクは必要
target_compile_definitions(brdf_core PUBLIC BRDF_VERSION="${PROJECT_VERSION}")

# ---------------------------------------------------------------------------
# brdf_vision: 画像処理(OpenCV を使う部分。公開ヘッダには OpenCV の型を出さない)
# ---------------------------------------------------------------------------
add_library(brdf_vision
    src/vision/ImageIO.cpp
    src/vision/HomographyFit.cpp
    src/vision/CircleGridDetector.cpp
    src/vision/Calibrate.cpp
)
add_library(brdf::vision ALIAS brdf_vision)
target_include_directories(brdf_vision PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(brdf_vision
    PUBLIC brdf::core
    PRIVATE opencv_core opencv_imgproc opencv_imgcodecs opencv_calib3d opencv_features2d)

# ---------------------------------------------------------------------------
# brdf: コマンドライン
# ---------------------------------------------------------------------------
add_executable(brdf src/cli/main.cpp)
target_link_libraries(brdf PRIVATE brdf::core brdf::vision CLI11::CLI11)

# ---------------------------------------------------------------------------
# examples: モックだけで動くデモ
# ---------------------------------------------------------------------------
add_executable(brdf_mock_demo examples/mock_demo.cpp)
target_link_libraries(brdf_mock_demo PRIVATE brdf::core)

# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
option(BRDF_BUILD_TESTS "Build unit tests" ON)
if(BRDF_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
