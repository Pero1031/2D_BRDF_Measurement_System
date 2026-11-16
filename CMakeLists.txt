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
# resources/view_template.html を、C++ のバイト配列として埋め込む(動作シミュレーションの HTML のひな型)
# ---------------------------------------------------------------------------
set(VIEW_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/resources/view_template.html)
file(READ ${VIEW_TEMPLATE} VIEW_TEMPLATE_HEX HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," VIEW_TEMPLATE_BYTES "${VIEW_TEMPLATE_HEX}")
string(LENGTH "${VIEW_TEMPLATE_HEX}" VIEW_TEMPLATE_HEX_LENGTH)
math(EXPR VIEW_TEMPLATE_SIZE "${VIEW_TEMPLATE_HEX_LENGTH} / 2")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/generated/ViewTemplate.cpp
"#include <cstddef>
namespace brdf::app::detail {
    extern const unsigned char kViewTemplateBytes[] = {${VIEW_TEMPLATE_BYTES}0x00};
    extern const std::size_t kViewTemplateSize = ${VIEW_TEMPLATE_SIZE};
}
")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${VIEW_TEMPLATE})

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
    src/io/TopconRecipeXml.cpp
    src/driver/topcon/TopconImager.cpp
    src/driver/rc8/Rc8Arm.cpp
    src/io/MeasurementsCsv.cpp
    src/analysis/RadianceFactor.cpp
    src/app/Analyze.cpp
    src/app/ViewExport.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/generated/ViewTemplate.cpp
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

# ---------------------------------------------------------------------------
# 実機の分光放射計(TOPCON SDK)。SDK がある PC だけで ON にする。
#   cmake --preset default -DBRDF_WITH_TOPCON=ON -DHS_SDK_DIR=<SDK のフォルダ>
# HS_SDK_DIR は、include/hs_api.h と lib/hs_api.lib の親(CI-100 インストーラの SDK\c)。
# 実行時には、同じ版の hs_api.dll(x64)が brdf.exe のそばか PATH 上に必要。
# ---------------------------------------------------------------------------
option(BRDF_WITH_TOPCON "Build the real TOPCON SDK driver (needs the SDK)" OFF)
set(HS_SDK_DIR "" CACHE PATH "Folder of the TOPCON SDK (contains include/hs_api.h and lib/hs_api.lib)")
if(BRDF_WITH_TOPCON)
    if(NOT EXISTS "${HS_SDK_DIR}/include/hs_api.h" OR NOT EXISTS "${HS_SDK_DIR}/lib/hs_api.lib")
        message(FATAL_ERROR "BRDF_WITH_TOPCON=ON needs -DHS_SDK_DIR=<folder with include/hs_api.h and lib/hs_api.lib>")
    endif()
    target_sources(brdf_core PRIVATE src/driver/topcon/RealHsApi.cpp)
    target_include_directories(brdf_core PRIVATE "${HS_SDK_DIR}/include")
    target_link_libraries(brdf_core PUBLIC "${HS_SDK_DIR}/lib/hs_api.lib")
    target_compile_definitions(brdf_core PUBLIC BRDF_WITH_TOPCON=1)
endif()

# ---------------------------------------------------------------------------
# 実機のロボットアーム(DENSO RC8、b-CAP)。b-CAP の SDK がある PC だけで ON にする。
#   cmake --preset default -DBRDF_WITH_BCAP=ON -DBCAP_SDK_DIR=<...>/CapLib/DENSO/RC8
# BCAP_SDK_DIR は、Include/C/bCAPClient/bcap_client.h と Src.Lib/*.c を含むフォルダ(DENSO の b-CAP クライアント SDK、MIT ライセンス)。
# 配布されている bCAPClient*.lib は 32 ビット版だけなので、SDK のソースを、このプロジェクトと同じ 64 ビットの静的ライブラリとして
# ビルドする。実行時に必要な DLL はない。
# ---------------------------------------------------------------------------
option(BRDF_WITH_BCAP "Build the real DENSO RC8 (b-CAP) arm driver (needs the b-CAP client SDK)" OFF)
set(BCAP_SDK_DIR "" CACHE PATH "Folder of the b-CAP client SDK (contains Include/C/bCAPClient/bcap_client.h and Src.Lib/*.c)")
if(BRDF_WITH_BCAP)
    if(NOT EXISTS "${BCAP_SDK_DIR}/Include/C/bCAPClient/bcap_client.h" OR NOT EXISTS "${BCAP_SDK_DIR}/Src.Lib/bCAPClient/bcap_client.c")
        message(FATAL_ERROR "BRDF_WITH_BCAP=ON needs -DBCAP_SDK_DIR=<folder with Include/C/bCAPClient/bcap_client.h and Src.Lib/bCAPClient/bcap_client.c>")
    endif()
    enable_language(C)

    # ヘッダは、ビルドフォルダにコピーして使う。SDK 同梱の古い stdint.h は、今の MSVC のものと衝突するので、コピーしない
    # (ソースの #include "stdint.h" は、MSVC 標準のものに解決される)
    set(BCAP_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/bcap_include)
    file(GLOB BCAP_HEADERS "${BCAP_SDK_DIR}/Include/C/*.h")
    list(FILTER BCAP_HEADERS EXCLUDE REGEX "/stdint[.]h$")
    file(COPY ${BCAP_HEADERS} DESTINATION ${BCAP_INCLUDE_DIR})
    file(COPY "${BCAP_SDK_DIR}/Include/C/bCAPClient/bcap_client.h" DESTINATION ${BCAP_INCLUDE_DIR}/bCAPClient)

    add_library(bcap_client STATIC
        "${BCAP_SDK_DIR}/Src.Lib/bcap_common.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_com.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_common.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_device.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_robotalk.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_socket.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_tcp.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_thread.c"
        "${BCAP_SDK_DIR}/Src.Lib/dn_udp.c"
        "${BCAP_SDK_DIR}/Src.Lib/bCAPClient/bcap_client.c")
    target_include_directories(bcap_client PRIVATE ${BCAP_INCLUDE_DIR} ${BCAP_INCLUDE_DIR}/bCAPClient)
    target_compile_definitions(bcap_client PRIVATE _USE_WIN_API WIN32 _WINDOWS _CRT_SECURE_NO_WARNINGS _WINSOCK_DEPRECATED_NO_WARNINGS)
    # SDK のソースは 32 ビット時代のもので、ポインタを int に切り詰める警告が多数出る。切り詰めているのは
    # 同じバッファ内の 2 つのポインタの差(数 KB)とソケット番号(小さい整数)だけで、実害はない。SDK のコードなので警告は消す。
    target_compile_options(bcap_client PRIVATE /W0)
    target_link_libraries(bcap_client PUBLIC ws2_32)

    target_sources(brdf_core PRIVATE src/driver/rc8/RealBcapApi.cpp)
    target_include_directories(brdf_core PRIVATE ${BCAP_INCLUDE_DIR} ${BCAP_INCLUDE_DIR}/bCAPClient)
    target_link_libraries(brdf_core PUBLIC bcap_client)
    target_compile_definitions(brdf_core PUBLIC BRDF_WITH_BCAP=1)
endif()

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
