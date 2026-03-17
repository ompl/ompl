# VAMP-OMPL Demos

This directory includes demos using [VAMP](https://github.com/KavrakiLab/vamp) SIMD-accelerated state validity checks. VAMP uses robot-specific compiled forward kinematics operations and vectorized collision checking for state and motion validation.

## Python

### Requirements
Install the required dependencies:
```bash
pip install requirements.txt
```

Install planner arena:
```bash
git clone https://github.com/ompl/plannerarena
cd plannerarena
pip install -e .
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

If PlannerArena is installed, you can visualize at localhost:8888 or the port printed in the terminal output.

Visualize the robot, environment and planned trajectory:
```bash
python3 vamp/VampPlanning.py --visualize
```