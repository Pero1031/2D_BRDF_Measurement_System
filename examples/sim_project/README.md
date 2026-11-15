# Example project (simulation)

A ready-to-run measurement project. It uses the simulated devices (`mode = "sim"` in `setup.toml`),
so it needs no hardware.

```bash
build/Debug/brdf.exe check examples/sim_project     # joint limits, collisions and angles, without moving anything
build/Debug/brdf.exe run   examples/sim_project     # type OK for the checklist (or add --yes)
build/Debug/brdf.exe angles examples/sim_project    # only the angles of every pose and spot
```

Results are written to `result/<date>_<time>_<kind>/`:

| File | Content |
|---|---|
| `measurements.csv` | one row per (pose, spot): angles in degrees, then the spectrum |
| `run_manifest.toml` | when and how it was measured |
| `setup.toml`, `recipe.toml` | copies of the settings used for this run |

Files in this folder:

- `setup.toml`: rig layout (arm dimensions and limits of the DENSO VP-6242, light and camera positions) and the simulated sample.
- `recipe.toml`: the poses (left arm = light, right arm = sample), the spots on the sample surface, the capture settings and the checklist.
