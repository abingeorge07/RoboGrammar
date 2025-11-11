# MPPI flow and where trajectory optimization runs

This document explains precisely how and when MPPI trajectory optimization is invoked in the search code, which classes are involved, and the key knobs you can tune. It is based on the search script `examples/graph_learning/heuristic_search_algo_mpc.py`, the environment `examples/graph_learning/RobotGrammarEnv.py`, and `examples/design_search/design_search.py` where `simulate()` and MPPI are implemented.

## Short summary
- Outer search samples candidate robot designs (rule sequences).
- When a numeric reward is required for a completed design, the search calls `RobotGrammarEnv.get_reward(...)`.
- `get_reward(...)` builds a `rd.Robot` and calls `simulate(robot, task, opt_seed, thread_count, ...)`.
- `simulate()` constructs an `rd.MPPIOptimizer` (the MPPI controller) and uses it to optimize a control/input sequence for that robot on the `task`.
- The MPPI optimizer runs many internal parallel simulations (each created by a `make_sim_fn()` factory) and scores trajectories using `task.get_objective_fn()` (an `rd.DotProductObjective`). The optimized open-loop sequence is applied to one `main_sim` to produce the final reward returned to the search.

---

## Top-level call chain (precise)

1. `heuristic_search_algo_mpc.search_algo(args)`
   - Builds `task` (a `ForwardSpeedTask` subclass, e.g. `FlatTerrainTask`).
   - Creates `RobotGrammarEnv(task, rules, seed, mpc_num_processes)`.
   - During design sampling, when a candidate design is chosen it checks validity and then calls `env.get_reward(selected_design)`.

2. `RobotGrammarEnv.get_reward(robot_graph)` (examples/graph_learning/RobotGrammarEnv.py)
   - If `enable_reward_oracle` is True: evaluate with a pre-trained GNN and return the oracle value (no MPPI).
   - Otherwise: build `robot = build_normalized_robot(robot_graph)`, pick an `opt_seed`, and call:
       `simulate(robot, self.task, opt_seed, self.mpc_num_processes, episode_count=1)`

3. `simulate(robot, task, opt_seed, thread_count, episode_count=1)` (examples/design_search/design_search.py)
   - `presimulate(robot)` quickly tests initial collision and computes an initial position.
   - Build `make_sim_fn()` that creates a `rd.BulletSimulation(task.time_step)`, calls `task.add_terrain(sim)`, and adds the robot into that sim. This factory is passed to MPPI so each MPPI sample runs in its own sim instance.
   - Create `value_estimator` (either `rd.FCValueEstimator` or `rd.NullValueEstimator` depending on `episode_count`) and `input_sampler = rd.DefaultInputSampler()`.
   - `objective_fn = task.get_objective_fn()` (usually an `rd.DotProductObjective` instance created by the task).
   - For each episode (typically just one):
     - Construct the MPPI optimizer:
       ```py
       optimizer = rd.MPPIOptimizer(
           kappa=1.0,
           discount_factor=task.discount_factor,
           dof_count=dof_count,
           interval=task.interval,
           horizon=task.horizon,
           sample_count=512,                # initial sample count
           thread_count=thread_count,
           seed=opt_seed + episode_idx,
           make_sim_fn=make_sim_fn,
           objective_fn=objective_fn,
           value_estimator=value_estimator,
           input_sampler=input_sampler)
       ```
     - Call `optimizer.update()` once (heavy initial sampling with 512 samples) and then `optimizer.set_sample_count(64)` to reduce per-step cost.
     - For each timestep `j` in `range(task.episode_len)`:
       - `optimizer.update()` — runs MPPI sampling & scoring (parallel calls to each sample's `runSimulation`), aggregates and updates `optimizer.input_sequence_`.
       - `input_sequence[:, j] = optimizer.input_sequence[:, 0]` — take the immediate action (column 0).
       - `optimizer.advance(1)` — advance MPPI sims by one step.
       - Apply that action to the `main_sim`: run `task.interval` inner simulation steps, call `task.add_noise(...)` each inner step, step the sim, and record `objective_fn(main_sim)` for each inner step.
   - After the rollout, return `(input_sequence, np.mean(rewards))` to the caller.

---

## Exact MPPI behavior (implementation notes)

- `rd.MPPIOptimizer` is a C++ class exposed to Python via bindings. Internally it:
  - holds `sample_count` separate `Simulation` instances (created with `make_sim_fn()`), one per sample;
  - in `update()` it enqueues `runSimulation` tasks into a `ThreadPool` (parallel across threads) to simulate each sampled input sequence and obtain returns;
  - applies a soft-max style weighting `exp(kappa * (return - max_return))` to recombine sampled input sequences into a new `input_sequence_` (a weighted average);
  - `runSimulation` uses `objective_fn(sim)` at each simulated step to compute per-sample returns and collects final observations for `value_estimator`.

## Where `task` and `objective_fn` are used

- `task.add_terrain(sim)` is invoked in every simulation instance created by `make_sim_fn()` so MPPI samples all see the same terrain.
- `task.get_objective_fn()` returns an `rd.DotProductObjective` (or other objective) instance. MPPI calls this objective frequently to compute per-step instantaneous reward during simulated rollouts.

## Important parameters & knobs you can tune

- In task classes: `task.interval`, `task.horizon`, `task.episode_len`, `task.time_step`, `task.discount_factor` — these directly affect MPPI horizon, temporal resolution and discounting.
- MPPI construction arguments in `simulate()`:
  - initial `sample_count = 512` (heavy warm start), then reduced to 64 via `set_sample_count(64)` — change these to adjust quality vs runtime.
  - `thread_count` (`mpc_num_processes`) determines parallelism; increase for faster MPPI at the cost of CPU threads.
  - `kappa` influences the soft-max weighting of samples.
- `objective_fn` is provided by `task.get_objective_fn()` — changing objective weights (e.g., `base_vel_weight`) directly changes what MPPI optimizes for.

## When MPPI is NOT run

- If `RobotGrammarEnv` was constructed with `enable_reward_oracle=True`, `get_reward()` will call the GNN reward oracle (`reward_oracle_evaluate`) and skip MPPI entirely.
- If `presimulate(robot)` detects an immediate self-collision, `simulate()` returns early and MPPI is not constructed.

## Timing / performance notes and typical pattern

- A common pattern (and the one used here) is:
  1. Do one expensive, high-sample MPPI `update()` to get a good initial `input_sequence_`.
  2. Reduce sample count and do cheaper updates at each timestep while rolling out the best action (receding horizon / model predictive control style).
  
  That pattern trades upfront computational cost for better-quality warm starts and then faster per-step updates.

---