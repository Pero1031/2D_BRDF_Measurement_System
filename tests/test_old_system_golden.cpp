// 旧システムの出力との照合テスト。
//
// tests/data/angle_table_out*_sample.csv は、旧システムの MakeTable が出力した
// D:\Data\AngleTable\out1〜3.csv から切り出したもの。右アームの6関節角から、
// 入射角 (theta_in, phi_in) と観察角 (theta_out, phi_out) [deg] を計算した結果が並んでいる。
// 新しい順運動学と角度計算が、これと一致することを確かめる。

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "brdf/geometry/RigGeometry.hpp"
#include "brdf/optics/Angles.hpp"

using namespace brdf;

namespace {

    struct GoldenRow {
        geometry::Joints right;
        Real theta_in, phi_in, theta_out, phi_out;   // [deg]
    };

    std::vector<GoldenRow> readGolden(const std::string& path) {
        std::ifstream ifs(path);
        REQUIRE(ifs.good());

        std::vector<GoldenRow> rows;
        std::string line;
        std::getline(ifs, line);   // ヘッダ
        while (std::getline(ifs, line)) {
            if (line.empty() || line == "\r") continue;
            std::stringstream ss(line);
            std::string cell;
            std::vector<Real> v;
            while (std::getline(ss, cell, ',')) v.push_back(std::stod(cell));
            REQUIRE(v.size() == 10);
            rows.push_back({geometry::jointsFromDegrees(v[0], v[1], v[2], v[3], v[4], v[5]),
                            v[6], v[7], v[8], v[9]});
        }
        return rows;
    }

    // 周期 360° を考慮した角度の差 [deg]
    Real circularDiffDeg(Real a, Real b) {
        Real d = std::fmod(std::fabs(a - b), 360.0);
        return std::min(d, 360.0 - d);
    }

    // 旧システムが MakeTable を実行した時点の装置パラメータ。
    // データから逆算して求めた(tests/data/README.md を参照)。
    geometry::RigSetup setupUsedForOldAngleTables() {
        geometry::RigSetup s;
        s.camera_position = Vec3(1272.0, 710.0, 554.0);
        return s;
    }

    constexpr Real kToleranceDeg = 0.002;   // 旧CSVは有効桁6桁

    // 旧システムの φ を、新システムの方向から再現する。
    //
    // 旧 Calc::calcAngle には、θ>90°(光源やカメラが基準面の裏側にある)のときに
    // φ が 180° 反転して出るバグがある(射影ベクトルを r/(r·Z) - Z で作っているため、
    // r·Z<0 だと向きが逆になる。旧コードも「cosの値が負の値を取りました」と警告していた)。
    // 新システムは正しい方位角を返すので、旧データと比べるときだけこの反転を足す。
    Real phiInOldConvention(const optics::Direction& d, bool& flipped) {
        Real phi = toDegrees(d.phi);
        flipped = d.theta > HALF_PI;
        if (flipped) phi = std::fmod(phi + 180.0, 360.0);
        return phi;
    }

    void checkAgainstGolden(const std::vector<GoldenRow>& rows, const geometry::Joints& left) {
        const geometry::RigGeometry rig(setupUsedForOldAngleTables());
        const Vec3 light = rig.lightPosition(left);

        Real worst = 0.0;
        int flipped_rows = 0;
        for (const GoldenRow& row : rows) {
            const geometry::SurfaceFrame frame = rig.tipFrame(row.right);
            const optics::GeometryAngles a = optics::computeAngles(frame, light, rig.cameraPosition());

            bool flip_in = false, flip_out = false;
            const Real phi_in  = phiInOldConvention(a.incidence, flip_in);
            const Real phi_out = phiInOldConvention(a.observation, flip_out);
            if (flip_in || flip_out) ++flipped_rows;

            const Real e0 = std::fabs(toDegrees(a.incidence.theta) - row.theta_in);
            const Real e1 = circularDiffDeg(phi_in, row.phi_in);
            const Real e2 = std::fabs(toDegrees(a.observation.theta) - row.theta_out);
            const Real e3 = circularDiffDeg(phi_out, row.phi_out);
            worst = std::max({worst, e0, e1, e2, e3});

            INFO("right joints [deg] = " << toDegrees(row.right(0)) << ", " << toDegrees(row.right(1)) << ", "
                 << toDegrees(row.right(2)) << ", " << toDegrees(row.right(3)) << ", "
                 << toDegrees(row.right(4)) << ", " << toDegrees(row.right(5)));
            CHECK(e0 < kToleranceDeg);
            CHECK(e1 < kToleranceDeg);
            CHECK(e2 < kToleranceDeg);
            CHECK(e3 < kToleranceDeg);
        }
        INFO("worst error [deg] = " << worst);
        CHECK(worst < kToleranceDeg);

        // θ>90° の行(旧システムが φ を反転する行)が実際にデータに含まれ、上の補正が使われていること。
        INFO("rows where the old system flips phi (theta > 90 deg) = " << flipped_rows);
        CHECK(flipped_rows > 0);
    }

    struct Case {
        const char* file;
        Real left_j1;    // 左アーム(光源)の J1。他の関節は (0, 90, 0, 0, 0)。
    };

} // namespace

TEST_CASE("new kinematics and angles reproduce the old system's angle tables", "[golden]") {
    const Case c = GENERATE(Case{"angle_table_out1_sample.csv", 65.0},
                            Case{"angle_table_out2_sample.csv", 60.0},
                            Case{"angle_table_out3_sample.csv", 50.0});
    INFO("file = " << c.file);

    const auto rows = readGolden(std::string(BRDF_TEST_DATA_DIR) + "/" + c.file);
    REQUIRE(rows.size() > 100);

    checkAgainstGolden(rows, geometry::jointsFromDegrees(c.left_j1, 0, 90, 0, 0, 0));
}

// 旧データの全行(約 35 万行)で照合する。通常のテストでは実行しない。
//   set BRDF_OLD_ANGLE_TABLE_DIR=D:\Data\AngleTable
//   brdf_tests "[.full]"
TEST_CASE("full comparison with D:/Data/AngleTable", "[.full]") {
    char* dir = nullptr;
#ifdef _MSC_VER
    size_t len = 0;
    _dupenv_s(&dir, &len, "BRDF_OLD_ANGLE_TABLE_DIR");
#else
    dir = std::getenv("BRDF_OLD_ANGLE_TABLE_DIR");
#endif
    REQUIRE(dir != nullptr);
    const std::string base(dir);
#ifdef _MSC_VER
    std::free(dir);
#endif

    const Case cases[] = {{"out1.csv", 65.0}, {"out2.csv", 60.0}, {"out3.csv", 50.0}};
    for (const Case& c : cases) {
        INFO("file = " << c.file);
        checkAgainstGolden(readGolden(base + "/" + c.file),
                           geometry::jointsFromDegrees(c.left_j1, 0, 90, 0, 0, 0));
    }
}
