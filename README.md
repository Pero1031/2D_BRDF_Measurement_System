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
| P3 | Project folders, TOML configuration, measurement session, CLI | Done |
| P4 | Calibration board, homography, spot XML | Done |
| P5 | Analysis (white-plate normalization, BRDF) | Done |
| P6 | Real drivers (DENSO RC8 via b-CAP, TOPCON `hs_api`) | In progress: the TOPCON driver is written and tested against a fake SDK (not yet tried on the device); the arm driver is next |
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
- Project folders with `setup.toml` (rig layout) and `recipe.toml` (poses, spots, capture settings); mistyped keys and wrong values are reported with the file, section and key
- A measurement session that runs a recipe on the (simulated) devices and saves per-spot angles and spectra as CSV, together with a record of the settings used
- A pre-flight check that finds joint-limit violations, arm collisions (including on the way to a pose) and spots whose light or camera is behind the surface, without moving anything
- Import of the previous system's pose CSV (all 52 poses are accepted)
- Calibration-board processing: circle-grid detection, a least-squares homography per pose, and SR-5000 spot definition files (XML). Checked against the previous system's real data: 13 images of 77 circles are detected and fitted, and the homographies agree with the saved ones to within 1 pixel at the paper corners; a previous spot XML is read and written back identically
- Analysis: radiance factor = sample / white plate (the same poses), and the BRDF = factor × ρ / π. Optional correction for the different irradiance at the sample point and the white reference point, which removes the systematic error of the previous system (about ±11% in the simulator's default geometry). A round-trip test recovers the simulator's true glossy BRDF to 1e-4
- A 3D animation of the arm motion (`brdf view`): one self-contained HTML file that opens in any browser, offline. Shows both arms, the light, the sample plate, the camera and its field of view, the spots and the light / viewing directions; play, step, jump to a pose, rotate the view; joint angles, the distance between the arms and the spot angles; arms turn red when they get too close, including *while moving between poses*
- A simulated camera board: the simulator draws the calibration board through its camera model, so `calibrate` → `run` can be tried end to end without hardware
- The `brdf` command line: `init`, `check`, `run`, `angles`, `import-poses`
- A driver for the real TOPCON spectroradiometer (`driver::topcon::TopconImager`), built on a thin interface to the SDK (`ITopconApi`). It writes the recipe and spot definition the SDK needs, measures, and returns the spectrum of every spot, so the manual step in TOPCON's own software is gone. It is tested with a fake SDK that records the call order and checks that nothing is leaked; it has not been tried on the device yet. Design notes: [docs/TopconSdkDesign.md](docs/TopconSdkDesign.md) (Japanese)

### What does not exist yet

Control of the real robot arms (`mode = "real"` reports "not supported yet"), and anything that has been verified on the real devices.

## Hardware

- Robot arms: 2 × DENSO **VP-6242** (6-axis) with RC8 controllers. The link dimensions in `geometry::ArmLinks` match the manufacturer's outline drawing, and the joint limits in `geometry::JointLimits` are the factory software-limit values from the VP robot manual.
- Spectroradiometer: TOPCON SR-5000H.
- The arm radius used by the clearance check (`RigSetup::arm_radius`) is an estimate from the outline drawing, and it does not include the sample plate.

## Quick start

```bash
build/Debug/brdf.exe check examples/sim_project     # pre-flight check, nothing is moved
build/Debug/brdf.exe run   examples/sim_project     # simulated measurement; type OK for the checklist (or --yes)
```

| Command | What it does |
|---|---|
| `brdf init <dir>` | create a project folder with template `setup.toml` and `recipe.toml` |
| `brdf check <dir>` | check limits, collisions and angles without moving any device |
| `brdf calibrate <dir>` | photograph the calibration board at every pose, detect the circles and save one homography per pose |
| `brdf run <dir> [--kind sample\|white\|calib] [--yes]` | run the measurement and save the result under `result/<date>_<time>_<kind>/` |
| `brdf angles <dir>` | write only the angles of every pose and spot |
| `brdf analyze <dir> [--sample R] [--white R] [--irradiance-correction]` | compute the radiance factor and the BRDF from a sample run and a white-plate run |
| `brdf view <dir> [--steps N]` | export the poses as a 3D animation (one HTML file) under `result/<date>_<time>_view/` |
| `brdf spots <dir>` | write SR-5000 spot definition files (XML), one per pose, from the calibration |
| `brdf import-poses <csv> [--out recipe.toml]` | build a recipe from the previous system's pose CSV |

A project folder holds everything about one measurement, and no absolute path is written in it:

```
my_measurement/
  setup.toml      rig layout (arm dimensions and limits, light and camera positions) and device mode
  recipe.toml     poses, spots on the sample (mm), capture settings, checklist
  calibration/
    images/<pose>.png    the calibration-board images
    homographies.csv     one homography per pose, with the reprojection error
  result/<date>_<time>_analysis/
    radiance_factor.csv  one row per (pose, spot): angles, then the radiance factor per wavelength
    brdf.csv             the same, as BRDF [1/sr]
    analysis_manifest.toml  which runs and which settings were used
  result/<date>_<time>_<kind>/
    measurements.csv     one row per (pose, spot): angles [deg] then the spectrum
    run_manifest.toml    when and how it was measured
    setup.toml, recipe.toml   copies of the settings used
```

### Watching the arms move

```bash
build/Debug/brdf.exe view examples/sim_project     # then open result/<date>_<time>_view/view.html in a browser
```

The animation follows the order the session uses: from the previous pose, the left arm moves first, then the right arm, with the joint angles interpolated linearly (as the controller's joint move does). Drag to rotate, wheel to zoom, Shift + drag to pan. The arms are drawn as thick sticks (a simplified model, not the real outline).
Use it before a real measurement to see where the arms and the light go; the pre-flight findings of `brdf check` are shown for each pose.

### Measuring the white plate and analysing

The reflectance is the ratio of the sample's radiance to that of a standard white plate measured at the same poses:

```bash
build/Debug/brdf.exe run examples/sim_project --yes                # the sample
build/Debug/brdf.exe run examples/sim_project --kind white --yes   # put the white plate on the arm first
build/Debug/brdf.exe analyze examples/sim_project                  # the latest sample and white runs
```

- A white run measures only the reference point (sample coordinates (0, 0), the crosshair) at every pose; the plate is too small to cover the sample.
- The two runs must have the same poses and the same rig layout (camera position and so on); `analyze` checks this and says what differs.
- `analyze` uses the white-plate value at the reference point as the value of the whole pose, as the previous system did. Points away from the reference point are lit differently (the previous system's systematic error: too low far from the light, too high near it). `--irradiance-correction` (or `irradiance_correction = true` in `[analysis]`) divides it out, treating the light as a point source.
- `[analysis]` in `setup.toml` also sets the plate's reflectance and the wavelength range. The LED is weak near 380 nm and 770 nm, so those wavelengths are unreliable.

### Using the calibration board

By default the simulator projects the spots with its own camera model (`projector = "camera-model"`). Real devices need the calibration board (`projector = "board"`):

```bash
build/Debug/brdf.exe calibrate examples/sim_project   # mount the board where the sample goes; one homography per pose
# set  projector = "board"  in setup.toml, then
build/Debug/brdf.exe run   examples/sim_project
build/Debug/brdf.exe spots examples/sim_project       # spot definition files for the SR-5000 software
```

How the board is mounted matters: because the circles form a symmetric grid, the image alone cannot tell which way round the board is.
`board_u` / `board_v` in `setup.toml` say which sample axes (`"+x"`, `"-x"`, `"+y"`, `"-y"`) the board's right and down directions correspond to,
and the board must appear rotated by less than 90° in the image.

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
- The driver for the real TOPCON spectroradiometer needs the TOPCON SDK and is off by default. To build it, add `-DBRDF_WITH_TOPCON=ON -DHS_SDK_DIR=<SDK folder>` to `cmake --preset default`, where the folder contains `include/hs_api.h` and `lib/hs_api.lib`. The program then needs the matching `hs_api.dll` (x64) next to `brdf.exe` or on `PATH`. Without the option, everything else builds and runs as before.

Run the mock demo:

```bash
build/Debug/brdf_mock_demo.exe
```

Compare against the previous system's real data (not part of the normal test run):

```bash
rem every row of the angle tables
set BRDF_OLD_ANGLE_TABLE_DIR=D:\Data\AngleTable
rem the 13 calibration-board images (the old project's AgInkBeta folder)
set BRDF_LEGACY_CALIB_DIR=<path to AgInkBeta>
build\tests\Debug\brdf_tests.exe "[.full]"
```

## Repository layout

```
include/brdf/   public headers
  core/           types, matrices, math utilities
  geometry/       joint angles, forward kinematics, rig layout
  optics/         incidence / observation angle computation
  driver/         interfaces for the robot arm and the spectral imager
  io/             setup.toml, recipe.toml, calibration CSV and spot XML reading / writing, pose CSV import
  calibration/    board layout, homography, how the board is mounted, per-pose spot projection
  vision/         (OpenCV) circle-grid detection, homography fit, image files, the calibrate command
  app/            measurement session, pre-flight check, project layout, commands
  sim/            simulator: rig, mock arm, mock imager, BRDF models, camera
  domain/         recipe (poses, spots, capture settings)
src/            implementation (src/cli: the `brdf` command line)
tests/          unit tests and checks against the previous system (tests/data/)
examples/       the mock demo and a ready-to-run simulated project (examples/sim_project)
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
