# Creating Python Bindings for a New Planner {#pybindingsPlanner}

It is often convenient to test a planner through either the OMPL.app GUI or by using a minimal Python program that defines a simple motion planning problem of interest. In both cases you need to create Python bindings for your planner. At a high level, the steps are:

- [Update the Python binding generation code](#pybinding)
- [Optionally, provide hints for planner parameter ranges that can then be used to create the appropriate controls in the OMPL.app GUI.](#params)

\attention
Please note that it is difficult to create Python bindings for multi-threaded planners. The problem is that only one thread at a time can access the Python interpreter. If a user creates an instance of a Python class derived from, e.g., ompl::base::StateValidityChecker or ompl::control::StatePropagator, the C++ code may call the Python interpreter simultaneously from multiple threads. In this case you need to modify you planner so that when called from Python it always runs in a single thread. In some cases, such as the ompl::geometric::PRM planner, this can be difficult. For PRM we wrote a special single-threaded version of the solve function (see `ompl/py-bindings/PRM.SingleThreadSolve.cpp`) that is used in the Python bindings instead of the default multi-threaded solve method.

## Planner parameters {#params}

The OMPL Planner class has a method called ompl::base::Planner::declareParam to define parameters that can be changed by the user. It is highly recommended that you use this method for all your planner parameters. It is possible to specify a suggested range of values as an optional argument to ompl::Planner::declareParam. This range will be used by the OMPL.app GUI to create the appropriate control widgets, so that users can change the parameter values through the GUI. See ompl::base::GenericParam::rangeSuggestion_ for the syntax used to specify parameter ranges.
