# Creating Python Bindings for a New Planner {#pybindingsPlanner}

It is often convenient to test a planner by using a minimal Python program that defines a simple motion planning problem of interest. Also, making your planner available in Python increases its reach among users. In both cases you need to create Python bindings for your planner. At a high level, the steps are:

- [Add bindings for your planner](#pybinding)
- [Optionally, provide hints for planner parameter ranges that can be used.](#params)

\attention
Pay extra care when binding multithreaded planners. See [Python bindings good practices](python.html#py_good_practices) for details.

## Add bindings for your planner {#pybinding}

The bindings folder follows a similar structure to the `src` folder. For a new planner binding, create a new file under the bindings folder hierarchy at the same location as your planner implementation would be. In that file, write the binding code for your planner. You can follow the existing planner bindings as an example (`py-bindings/geometric/planners/rrt/RRT.cpp` or the skeleton at `py-bindings/templates/PlannerSkeleton.cpp`), as well as refer to the [Nanobind documentation](https://nanobind.readthedocs.io). Finally, call your newly added function in `ompl/py-bindings/python.cpp` and rebuild the bindings.

### Multilevel planners

Multilevel planners (`QRRT`, `QMP`, etc.) live in the `ompl.multilevel` submodule. They accept either a single `SpaceInformation` (non-multilevel mode) or a list of `SpaceInformation` objects for quotient-space planning:

```python
from ompl import base as ob, multilevel as ml

si_vec = [si_coarse, si_fine]
planner = ml.QRRT(si_vec)
planner.setProblemDefinition(pdef)
planner.setup()
planner.solve(5.0)
```

Register multilevel bindings in `python.cpp` after projection and parameter types. See `py-bindings/multilevel/planners/detail.hpp` for the shared `BundleSpaceSequence` binding helper.

### ODE control solvers

ODE-based control uses `ompl.control.ODEBasicSolver` (and related classes) with Python callables. Wrap user callbacks with GIL acquisition — see `py-bindings/control/ODESolver.cpp`. Run `scripts/list_binding_gaps.py` to audit remaining unbound public headers.

Some C++ APIs are intentionally excluded from Python because of threading/GIL issues; see [Excluded from Python bindings](python.html#py_excluded_bindings).

## Planner parameters {#params}

The OMPL Planner class has a method called ompl::base::Planner::declareParam to define parameters that can be changed by the user. It is highly recommended that you use this method for all your planner parameters. It is possible to specify a suggested range of values as an optional argument to ompl::Planner::declareParam. This range can be used by external tools for example to create the appropriate control widgets, so that users can change the parameter values. See ompl::base::GenericParam::rangeSuggestion_ for the syntax used to specify parameter ranges.
