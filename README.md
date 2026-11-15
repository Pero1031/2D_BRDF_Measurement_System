# 2D BRDF Measurement

A system that measures BRDFs (bidirectional reflectance distribution functions) with two robot arms and a 2D spectroradiometer.
It is a restructured, generalized and documented rewrite of the previous in-house measurement system.

> **Work in progress.** It currently runs on mocks only; no real hardware is connected yet (see [Status](#status)).

## What the system measures

```
                  2D spectroradiometer (fixed)
                              ▲ captures
   light  ─────────▶  ┌───────────┐
 (left arm tip)       │  sample   │ ◀── held by the right arm tip
                      └───────────┘
```

- The left arm moves the light source, the right arm moves the sample, and a fixed 2D spectroradiometer captures it.
- The light and the camera are at finite distances, so every point on the sample sees a slightly different incidence and observation direction.
  A single capture therefore yields spectra for many angle combinations (θi, φi, θo, φo) at once.
- The input is the **arm poses**; the angle of each point is computed from the poses.

A detailed analysis of the previous system is in [docs/LegacySystemAnalysis.md](docs/LegacySystemAnalysis.md) (Japanese).

## Status

| Phase | Content | State |
|---|---|---|
| P0 | Foundation (CMake, tests, folder layout) | Done |
| P1 | Geometry and angles (forward kinematics, angle computation, check against previous data) | Done |
| P2 | Driver v2 and simulator (`ISpectralImager`, `SimulatedRig`, mock arms and imager) | Done |
| P3 | Project folders, TOML configuration, measurement session, CLI | Next |
| P4 | Calibration board, homography, spot XML | Not started |
| P5 | Analysis (white-board normalization, BRDF reconstruction) | Not started |
| P6 | Real drivers (DENSO RC8 via b-CAP, TOPCON `hs_api`) | Not started |
| P7 | Documentation (usage, setup procedure, troubleshooting) | Not started |

The full plan is in [docs/DesignPlan.md](docs/DesignPlan.md) (Japanese).

### What works today

- Arm tip position and orientation (forward kinematics): `geometry::Rc8Kinematics`
- World positions of the light, the sample and the camera: `geometry::RigGeometry`
- Incidence and observation angles of a point on the sample: `optics::computeAngles`
- A test that reproduces the previous system's angle tables (about 350,000 rows)
- A simulated rig: mock robot arms with joint limits and arm-to-arm clearance checks (`sim::MockRobotArm`)
- A simulated 2D spectroradiometer that "measures" a known BRDF (Lambert or Phong) through a pinhole camera model (`sim::MockSpectralImager`)
- Driver interfaces for the real hardware: `driver::IRobotArm`, `driver::ISpectralImager`
- `examples/mock_demo.cpp`: one full simulated pose, from moving the arms to per-spot angles and spectra

### What does not exist yet

Real hardware control, automated measurement runs, calibration-board processing and BRDF analysis.

## Hardware

- Robot arms: 2 × DENSO **VP-6242** (6-axis) with RC8 controllers. The link dimensions in `geometry::ArmLinks` match the manufacturer's outline drawing, and the joint limits in `geometry::JointLimits` are the factory software-limit values from the VP robot manual.
- Spectroradiometer: TOPCON SR-5000H.
- The arm radius used by the clearance check (`RigSetup::arm_radius`) is an estimate from the outline drawing, and it does not include the sample plate.

## Build and test

Requirements:

- Visual Studio 2022 ("Desktop development with C++")
- CMake 3.25 or later
- The `vcpkg/` checkout in the repository root (dependencies are declared in `vcpkg.json`)

```bash
cmake --preset default
cmake --build --preset debug
ctest --preset debug
```

- The first `cmake --preset default` takes a long time because vcpkg builds the dependencies (including OpenCV). Later runs take a few seconds.
- You can also open this folder in Visual Studio with "Open a local folder"; it uses the same presets.

Run the mock demo:

```bash
build/Debug/brdf_mock_demo.exe
```

Compare against every row of the previous system's angle tables (not part of the normal test run):

```bash
set BRDF_OLD_ANGLE_TABLE_DIR=D:\Data\AngleTable
build\tests\Debug\brdf_tests.exe "[.full]"
```

## Repository layout

```
include/brdf/   public headers
  core/           types, matrices, math utilities
  geometry/       joint angles, forward kinematics, rig layout
  optics/         incidence / observation angle computation
  driver/         interfaces for the robot arm and the spectral imager
  sim/            simulator: rig, mock arm, mock imager, BRDF models, camera
  domain/         recipe (to be redesigned in P3)
src/            implementation
tests/          unit tests and checks against the previous system (tests/data/)
examples/       demos that run on mocks only
docs/           analysis, design, plan
tools/          helper scripts
```

## Coordinate systems and angle conventions

- World frame: origin = base of the left arm, unit = mm.
- Angles are measured in a surface frame. θ = angle from the normal (Z axis), φ = angle from the X axis toward the Y axis. Units are radians internally.
- There are two sample frames and their **φ differs by 90°**. The paper frame is the standard.

| Frame | Axes | Used for |
|---|---|---|
| `paperFrame` (standard) | paper X = tip +Y, paper Y = tip −X, paper Z = tip +Z | per-point sample angles |
| `tipFrame` | the arm tip's X, Y, Z as they are | checking against the previous angle tables |

Rig parameters such as the camera position are collected in `geometry::RigSetup`.
**The camera position changes from one measurement day to the next**, so real measurements must record it in a configuration file (implemented in P3) and store it with the results.

## Differences from the previous system

- No absolute paths and no constants hard-coded in headers (project folders and TOML instead).
- Computations are pure functions: no printing, no waiting for input, no `exit()`.
- The previous `calcAngle` bug that flipped φ by 180° when θ > 90° is fixed.

## Character encoding

Sources are unified to **UTF-8 with BOM and CRLF** (see `.editorconfig`).

```bash
python tools/check_encoding.py        # check
python tools/check_encoding.py --fix  # fix
```

## Documentation

- [docs/LegacySystemAnalysis.md](docs/LegacySystemAnalysis.md): analysis of the previous system and the results of checking against its data (Japanese)
- [docs/DesignPlan.md](docs/DesignPlan.md): design and development plan (Japanese)
- [tests/data/README.md](tests/data/README.md): origin of the comparison data (Japanese)
