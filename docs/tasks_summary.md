# Tasks summary for examples/design_search/tasks.py

This document summarizes what information each class in `examples/design_search/tasks.py` holds: constructor parameters, attributes (state), and important methods that mutate or consume state.

## ForwardSpeedTask (abstract base)
- Purpose: base task defining common simulation parameters and an objective function for forward motion.
- Constructor parameters (defaults):
  - `time_step` (float) — simulation step size (1/240)
  - `discount_factor` (float) — discount factor for optimization (0.99)
  - `interval` (int) — control/action interval (16)
  - `horizon` (int) — planning horizon (16)
  - `episode_len` (int) — episode length (128)
  - `noise_seed` (int) — RNG seed offset for noise (0)
  - `force_std`, `torque_std` (float) — noise std for added forces/torques (0.0)
- Attributes:
  - `time_step`, `discount_factor`, `interval`, `horizon`, `episode_len`, `noise_seed`, `force_std`, `torque_std`
  - `objective_fn`: `rd.DotProductObjective()` with default weight vectors:
    - `base_dir_weight = [-2.0, 0.0, 0.0]`
    - `base_up_weight = [0.0, 2.0, 0.0]`
    - `base_vel_weight = [2.0, 0.0, 0.0]`
  - `result_bound` (float) = 10.0 — maximum reasonable result
- Methods:
  - `get_objective_fn()` → returns `objective_fn`
  - `add_noise(sim, time_step_idx)` — applies per-timestep random link forces/torques using `noise_seed + time_step_idx` and `force_std`/`torque_std`
  - `add_terrain(sim)` — abstract, implemented by subclasses to populate the simulator

## FlatTerrainTask
- Purpose: simple flat ground
- Attributes:
  - `floor`: `rd.Prop` BOX sized `[40.0, 1.0, 10.0]`
- Methods:
  - `add_terrain(sim)` — places `floor` at `[0.0, -1.0, 0.0]`

## RidgedTerrainTask
- Purpose: floor plus many small bumps (ridges)
- Constructor params: `seed` (int)
- Attributes:
  - `seed`
  - `floor`: `rd.Prop` BOX `[20.0, 1.0, 10.0]`
  - `bump`: `rd.Prop` BOX `[0.1, 0.2, 10.0]`
- Methods:
  - `add_terrain(sim)` — places `floor` and 20 bumps positioned with RNG seeded by `seed`

## GapTerrainTask
- Purpose: multiple platforms separated by gaps
- Constructor params: `x_min`, `x_max`, `seed`
- Attributes:
  - `seed`
  - `platform_x` (np.array) — platform center x positions
  - `platforms` (list[`rd.Prop`]) — per-platform BOX props with computed half-widths
  - `floor_x`, `floor` — secondary floor prop placed lower
- Methods:
  - `add_terrain(sim)` — places the platforms at `platform_x` and a lower `floor`

## SteppedTerrainTask
- Purpose: series of small platforms forming stepped terrain
- Constructor params: `x_min`, `x_max`, `seed`
- Attributes:
  - `seed`
  - `platform_x` (np.array)
  - `platforms` (list[`rd.Prop`])
- Methods:
  - `add_terrain(sim)` — places platforms with small random y increments

## WallTerrainTask
- Purpose: floor with a series of walls and side walls as obstacles
- Constructor params: `seed`
- Attributes:
  - `seed`
  - `floor`: `rd.Prop` BOX `[20.0,1.0,10.0]`
  - `wall`: `rd.Prop` BOX `[0.05,0.5,0.25]`
  - `side_wall`: `rd.Prop` BOX `[20,0.5,0.05]`
- Constructor override: sets `horizon=32` when calling base constructor
- Methods:
  - `add_terrain(sim)` — places floor, two side walls, and 10 walls arranged by RNG

## FrozenLakeTask
- Purpose: flat, low-friction surface
- Attributes:
  - `floor`: `rd.Prop` BOX `[20.0, 0.05, 10.0]`
  - `floor.color` — visual property set to `[0.8, 0.9, 1.0]`
- Methods:
  - `add_terrain(sim)` — places `floor`

## HillTerrainTask
- Purpose: heightfield / hilly terrain
- Constructor params: `seed`
- Attributes:
  - `seed`
  - `rng`: `np.random.RandomState(seed)`
  - `heightfield`: `rd.HeightfieldProp` created from clipped random heights `y` (shape (97,33))
- Methods:
  - `add_terrain(sim)` — places `heightfield`

## NewRidgedTerrainTask & NewRidgedTerrainTask2
- Purpose: ridged terrain variants with modified objective weights
- Constructor params: `seed`
- Attributes:
  - `seed`, `floor`, `bump` (like `RidgedTerrainTask`)
  - updated `objective_fn` weights:
    - `base_dir_weight = [-1.0, 0.0, 0.0]`
    - `base_up_weight = [0.0, 1.0, 0.0]`
    - `base_vel_weight = [8.0, 0.0, 0.0]`
- Methods:
  - `add_terrain(sim)` — places floor and bumps; `NewRidgedTerrainTask2` sets first two bumps with explicit heights and others higher

## NewGapTerrainTask
- Purpose: gap terrain variant with wider gaps at the beginning and modified objective weights
- Constructor params: `x_min`, `x_max`, `seed`
- Attributes:
  - `seed`, `platform_x`, `platforms`, `floor_x`, `floor` (similar to `GapTerrainTask`)
  - updated `objective_fn` weights:
    - `base_dir_weight = [-1.0, 0.0, 0.0]`
    - `base_up_weight = [0.0, 1.0, 0.0]`
    - `base_vel_weight = [3.0, 0.0, 0.0]`
- Methods:
  - `add_terrain(sim)` — places platforms and a lower floor

## NewSteppedTerrainTask1 / 2 / 3 / 4
- Purpose: stepped terrain variants with differences in vertical offsets and increments
- Shared attributes:
  - `seed`, `platform_x`, `platforms` (platform props with larger heights)
- Differences (y placement):
  - Task1: `y = -5.0 + i * 0.07`
  - Task2: `y = -5.0 + i * 0.05`
  - Task3: `y` starts at `-5.0` and increments by `rng.uniform(0.0, min(0.02*i, 0.1))`
  - Task4: similar placement to others and also updates `objective_fn` weights to emphasize forward velocity
- Methods:
  - `add_terrain(sim)` — places the sequence of step/platform props according to the per-class rules

## NewWallTerrainTask
- Purpose: wall-course variant with different wall sizes and objective weights
- Constructor params: `seed`
- Attributes:
  - `seed`, `floor`, `wall`, `side_wall`
  - updated `objective_fn` weights (like some New* classes):
    - `base_dir_weight = [-1.0, 0.0, 0.0]`
    - `base_up_weight = [0.0, 1.0, 0.0]`
    - `base_vel_weight = [8.0, 0.0, 0.0]`
  - sets `horizon = 32` when calling base constructor
- Methods:
  - `add_terrain(sim)` — places floor, side walls, and walls at fixed spacings

## Common patterns and notes
- Most tasks:
  - use `seed` / `rng` for reproducible terrain generation
  - create `rd.Prop` or `rd.HeightfieldProp` objects for static geometry
  - compute `platforms` and their `platform_x` (centers) from parameters and RNG
  - override `objective_fn` weight vectors to change reward behavior (direction, up alignment, velocity emphasis)
- `add_terrain(sim)` is the primary method for subclasses to mutate the `sim`; it places props and does not return values
- `add_noise` (base class) applies per-timestep forces/torques controlled by `noise_seed`, `force_std`, and `torque_std`

---
