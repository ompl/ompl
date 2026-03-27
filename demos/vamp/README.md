# VAMP-OMPL Demos

This directory includes demos using [VAMP](https://github.com/KavrakiLab/vamp) SIMD-accelerated state validity checks. VAMP uses robot-specific compiled forward kinematics operations and vectorized collision checking for state and motion validation.

## Python

### Requirements
Install the required dependencies:
```bash
pip install requirements.txt
```

### Geometric Planning

A simple script is provided to demonstrate the usage of VAMP with OMPL using Python bindings. Run the script from the `demos` subdirectory:

```bash
python3 vamp/VampPlanning.py
```

Run a benchmark with multiple planners and planning iterations:

```bash
python3 vamp/VampPlanning.py --benchmark --n_trials 100
```

If PlannerArena is installed, you can visualize at `localhost:8888` or the port printed in the terminal output.

Visualize the robot, environment and planned trajectory at `localhost:8080`:
```bash
python3 vamp/VampPlanning.py --visualize
```

### Motion Benchmaker Dataset

A script solving the 700 problems from 7 different problem sets on the MotionBenchmaker dataset. It supports Panda, UR5 and Fetch robots, changing the robot argument to `panda`, `ur5` or `fetch`. The VAMP submodule at `external/vamp` is required. To run:

```bash
python motion_benchmaker_demo.py --robot panda --visualize
```

To run a benchmark with multiple planners, on one of the problems for each set:

```bash
python motion_benchmaker_demo.py --robot panda --benchmark --n-trials <n_trials>
```