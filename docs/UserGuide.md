# User guide

How to use the `brdf` command line, from a simulated trial run to a real measurement.
The Japanese version is [UserGuide.ja.md](UserGuide.ja.md).

> **Status.** Everything below that says "simulation" is working and tested. The real-device parts (`mode = "real"`) are implemented and tested against fakes, but **have not been tried on the real arms or the spectroradiometer yet**. Read [SafetyProcedure.md](SafetyProcedure.md) (Japanese) before moving a real arm.

## 1. The workflow in one page

```
brdf init <dir>                  1. create a project folder
   edit setup.toml, recipe.toml  2. describe the rig and the poses
brdf check <dir>                 3. check limits, collisions and angles (nothing moves)
brdf view <dir>                  4. watch the arm motion in a browser
brdf calibrate <dir>             5. (board mode) photograph the calibration board, fit one homography per pose
brdf run <dir> --kind white      6. measure the white plate at every pose
brdf run <dir>                   7. measure the sample at every pose
brdf analyze <dir>               8. radiance factor and BRDF = sample / white plate
```

Steps 5 and 6 are needed for a real measurement. In simulation you can skip step 5 (the simulator knows its own camera) and still run steps 6 to 8.

## 2. Build

```bash
cmake --preset default
cmake --build --preset debug
```

Or, for faster rebuilds, `tools\build.cmd debug` (Ninja; see the README). The executable is `build/Debug/brdf.exe` (or `build-ninja/Debug/brdf.exe`).

For real devices you need a build with both SDKs: `-DBRDF_WITH_BCAP=ON -DBCAP_SDK_DIR=<...>` and `-DBRDF_WITH_TOPCON=ON -DHS_SDK_DIR=<...>` (see the README). `hs_api.dll` (x64) must be on `PATH`.

## 3. Try it in simulation

A ready-to-run project is in `examples/sim_project`. Nothing is moved; the devices are simulated.

```bash
build/Debug/brdf.exe check   examples/sim_project            # pre-flight check
build/Debug/brdf.exe view    examples/sim_project            # then open the HTML file it prints
build/Debug/brdf.exe run     examples/sim_project --yes      # measure the sample (simulated)
build/Debug/brdf.exe run     examples/sim_project --kind white --yes
build/Debug/brdf.exe analyze examples/sim_project
```

Results are written under `examples/sim_project/result/`.

To start your own project:

```bash
build/Debug/brdf.exe init my_measurement
```

This creates `my_measurement/setup.toml` and `recipe.toml` with explanatory comments. No absolute path is written in either file.

## 4. The project folder

```
my_measurement/
  setup.toml      the rig and the devices (changes rarely; the camera position changes each measurement day)
  recipe.toml     the poses, the spots on the sample, the capture settings, the checklist
  calibration/
    images/<pose>.png    calibration-board images (written by `brdf calibrate`)
    homographies.csv     one homography per pose, with the reprojection error
  result/<date>_<time>_<kind>/
    measurements.csv     one row per (pose, spot): angles [deg], then the spectrum
    run_manifest.toml    when and how it was measured
    setup.toml, recipe.toml   copies of the settings used
  result/<date>_<time>_analysis/
    radiance_factor.csv, brdf.csv, analysis_manifest.toml
  work/           (real mode only) temporary recipe / spot files for the TOPCON SDK
```

### setup.toml

| Section | What it holds |
|---|---|
| `[devices]` | `mode = "sim"` (simulation) or `"real"` (real devices) |
| `[arm]`, `[arm.links]`, `[arm.limits]` | arm radius for the clearance check, link lengths, joint limits (defaults are the VP-6242 values) |
| `[rig]` | base positions of the two arms, light offset, **camera position**, sample surface offset |
| `[calibration]`, `[calibration.board]` | `projector` (`"camera-model"` or `"board"`), the board layout, how the board is mounted (`board_u`, `board_v`) |
| `[imager]` | `device = "SR5100"` (default) or `"SR5000"`, binning, lens, image size |
| `[controllers]` | RC8 controllers: IP addresses (left `192.168.0.2`, right `192.168.0.5`), port 5007, timeouts, `max_speed` (default 0.05 = 5 %), `abortable_moves` |
| `[analysis]` | white-plate reflectance, wavelength range, `irradiance_correction` |
| `[spot_xml]` | spot size for the spot definition files |
| `[sim.*]` | simulator settings (used only when `mode = "sim"`) |

All lengths are in mm and angles in degrees. A misspelled key or a wrong value is reported with the file, section and key.
**Update `camera_position` every measurement day**; a copy of `setup.toml` is saved with every result.

### recipe.toml

- `[recipe]`: name, operator, `kind` (`sample`, `white` or `calib`), and `checklist` (items you must confirm before a run).
- `[capture]`: `integration_time_ms` (0 = automatic, real device), `arm_speed` (0 to 1; never above `max_speed`), wavelength range and step.
- `[[spots]]`: points on the sample, `id`, `x`, `y` in mm (paper frame, origin at the reference point). Or a grid with `[spot_grid]` (`center`, `step`, `count`).
- `[[poses]]`: `left` (light arm) and `right` (sample arm) joint angles J1 to J6 in degrees.

To reuse the previous system's poses: `brdf import-poses <csv> --out recipe.toml` (a CSV with the header `RJ1..RJ6,LJ1..LJ6`; all 52 poses of the previous system are accepted).

## 5. Command reference

| Command | What it does |
|---|---|
| `brdf init <dir>` | Create a project folder with template files. |
| `brdf check <dir>` | Check joint limits, arm-to-arm collisions (also along the way between poses) and angles. Moves nothing. Run it before every real run. |
| `brdf view <dir> [--steps N]` | Export a 3D animation of the poses to `result/<date>_<time>_view/view.html`. Drag to rotate, wheel to zoom, Shift + drag to pan. |
| `brdf calibrate <dir>` | Photograph the calibration board at every pose, detect the circles, save one homography per pose. |
| `brdf run <dir> [--kind sample\|white\|calib] [--yes]` | Run the measurement and save it. It shows the checklist and waits for you to type `OK`; `--yes` skips only the checklist. |
| `brdf angles <dir>` | Write only the incidence and observation angles of every pose and spot. |
| `brdf analyze <dir> [--sample R] [--white R] [--irradiance-correction]` | Radiance factor and BRDF from a sample run and a white-plate run (default: the latest of each). |
| `brdf spots <dir>` | Write spot definition files (XML for the TOPCON software), one per pose, from the calibration. Needs `projector = "board"`. |
| `brdf import-poses <csv> [--out F] [--force]` | Build a recipe from the previous system's pose CSV. |
| `brdf arm status <dir>` | (real mode) Show the current joint angles of both arms. Moves nothing. |
| `brdf arm move <dir> --arm left\|right (--to=J1,..,J6 \| --standard \| --jog N=DEG) [--speed S]` | (real mode) Move **one** arm. Used for the first moves. |

`brdf --version` shows the version; `brdf <command> --help` shows the options.

## 6. Measuring the white plate and analysing

BRDF is derived from the ratio of the sample's radiance to that of a standard white plate measured at the same poses.

1. `brdf run <dir>` with the sample on the right arm.
2. Put the white plate on the arm and run `brdf run <dir> --kind white`. A white run measures only the reference point (0, 0) at every pose.
3. `brdf analyze <dir>`.

The two runs must have the same poses and the same rig layout (camera position and so on); `analyze` checks this and says what differs.
`--irradiance-correction` removes the previous system's systematic error (the light reaches different points of the sample with different strength); without it the result matches the previous system.
The LED is weak near 380 nm and 770 nm, so those wavelengths are unreliable.

## 7. Using the calibration board (real devices)

Real devices find the spots in the camera image through a homography fitted from the calibration board. In `setup.toml`:

1. Mount the board where the sample goes. Because the circles form a symmetric grid, the image alone cannot tell which way round the board is. Set `board_u` and `board_v` to the sample axes (`"+x"`, `"-x"`, `"+y"`, `"-y"`) that the board's right and down directions correspond to; the board must appear rotated by less than 90° in the image.
2. `brdf calibrate <dir>`. Check the reprojection error it reports against `max_rms_px`.
3. Set `projector = "board"`, then `brdf run <dir>`.

## 8. Real devices

The real-device mode is built in two layers: the **arms** (DENSO RC8 over b-CAP) and the **spectroradiometer** (TOPCON SR-5100 over `hs_api`). Both SDKs must be built in.

1. In `setup.toml`: `[devices] mode = "real"`, and check `[controllers]` (IP addresses, port), `[arm.limits]` and `[imager] device`.
2. Follow [SafetyProcedure.md](SafetyProcedure.md) (Japanese): remove everything from the arms' reach, keep the emergency stop in reach, start at **5 % speed**, and go through the stages: `brdf arm status`, then small `brdf arm move --jog` steps, then the standard pose, then `brdf run`.
3. `brdf run` in real mode shows the devices, the **effective** speed, the range of motion and how to stop, and needs a typed `OK` (`--yes` does not skip it).

Safety guards that are always on in real mode (`run`, `calibrate`, `arm`):

- A speed cap (`max_speed`, 5 % by default).
- The target is checked against the joint limits before anything is sent, and the encoders are checked after the move.
- The arms' actual position is read first, and the path to the first pose is checked.
- Collisions are checked at 20 points along every move between poses, but only arm against arm. The camera, spectroradiometer, floor, table and sample plate are **not** modelled.
- A warning for poses outside the joint range the previous system used.
- On any failure: both arms are halted, the motors are turned off, and the connection is closed.
- Ctrl+C stops before the next pose. With `abortable_moves = true` it also halts the arm in the middle of a move (not verified on the arms yet; this is not a replacement for the emergency stop). A second Ctrl+C ends the program.

## 9. Troubleshooting

| Message / code | Meaning and what to do |
|---|---|
| `JointLimit` | A target is outside `[arm.limits]`. Change the pose. |
| `Collision` | Two arms come too close (the check uses a 60 mm radius around each arm). Change the pose, or look at `brdf view`. |
| `Timeout` | A device did not answer in time; it may have been moving. The arm is halted and the connection is closed. Check the state with the pendant, then retry. |
| `Communication` | Cannot connect, or the connection was lost. Check the cable, the IP address in `[controllers]`, and the controller's power. |
| `Cancelled` | You pressed Ctrl+C. Nothing is left running. |
| `InvalidArgument` ... `nothing was moved` | The request or the answer was refused before any move (for example, `OK` was not typed exactly). |
| `NotImplemented` ... `BRDF_WITH_BCAP` / `BRDF_WITH_TOPCON` | `mode = "real"` needs a build with both SDKs (section 2). |
| `FocusError` | The measuring distance does not match the real distance: check the focus and the camera position. |
| `Saturated` | The measurement is too bright: check the light, the integration time or the ND filter. |
| `brdf spots needs ... projector = "board"` | Do the calibration first (section 7). |
| Tests are slow | `tools\build.cmd debug test` skips the 8 slow calibration-board tests; `test-all` runs everything. |

## 10. Where to read more

- [LegacySystemAnalysis.md](LegacySystemAnalysis.md): the previous system and how this one was checked against its data (Japanese)
- [DesignPlan.md](DesignPlan.md): design and plan (Japanese)
- [SafetyProcedure.md](SafetyProcedure.md): the first moves of the real arms (Japanese)
- [Rc8ArmDesign.md](Rc8ArmDesign.md), [TopconSdkDesign.md](TopconSdkDesign.md): the real drivers, and what is still unverified on the devices (Japanese)
