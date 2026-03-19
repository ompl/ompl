# VAMP-OMPL Demos

This directory includes demos using [VAMP](https://github.com/KavrakiLab/vamp) SIMD-accelerated state validity checks. VAMP uses robot-specific compiled forward kinematics operations and vectorized collision checking for state and motion validation.

## C++

### Geometric Planning

Run the simple geometric planning example from `VAMPPlanning.cpp`:

```bash
cd build/Release
cmake ../..
make -j <cores>
./demos/demo_VAMPPlanning
```

This will run a single plan instance on a spherized collision environment. To run OMPL's benchmarking on this, run:

```bash
./demos/demo_VAMPBenchmark --benchmark <number_of_trials>
```

This will run the benchmark for the specified number of trials with planners specified in `VAMPPlanning.cpp` and output the results in a `.log` file.

### MotionBenchmaker

First, extract the json motionbenchmaker from the included VAMP submodule, from your `ompl` directory:

```bash
cd external/vamp/resources
python problem_tar_to_pkl_json.py
```

This will generate a file in `<ompl_directory>/external/vamp/resources/panda/problems.json`. Check the `problem_tar_to_pkl_json.py` script for more details and how to use with other robots.

Go back to your OMPL build directory and run the benchmark:

```bash
./demos/demo_VAMPMotionBenchmaker --problem-file <path_to_generated_JSON_file> --robot panda --planner RRTConnect --timeout 1.0 
```

This will run the specified planner on all 100 problems for each 7 problem types from the [MotionBenchmaker dataset](https://github.com/KavrakiLab/motion_bench_maker). To run OMPL's benchmark:

```bash
./demos/demo_VAMPMotionBenchmaker --problem-file <path_to_generated_JSON_file> --robot panda --planner RRTConnect --timeout 1.0 --benchmark <number_of_trials>
```

This will run a benchmark for `<number_of_trials>` trials on one problem for each of the 7 MotionBenchmaker problem types, and output the results as a `.log` file.

### Using log files from Benchmark

Use the provided Python script to analyze the generated `.log` files from the benchmark runs:

```bash
python <ompl_directory>/scripts/ompl_benchmark_statistics.py <generated_log_filename>.log -d <generated_log_filename>.db
```

These can be uploaded to [PlannerArena](https://planner-arena.kavrakilab.rice.edu/) for visualization