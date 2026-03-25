# Creating Python Bindings for a New Planner {#pybindingsPlanner}

It is often convenient to test a planner by using a minimal Python program that defines a simple motion planning problem of interest. Also, making your planner available in Python increases its reach among users. In both cases you need to create Python bindings for your planner. At a high level, the steps are:

- [Add bindings for your planner](#pybinding)
- [Optionally, provide hints for planner parameter ranges that can then be used to create the appropriate controls in the OMPL.app GUI.](#params)

\attention
Pay extra care when binding multithreaded planners. See [Python bindings good practices](python.html#py_good_practices) for details.

## Add bindings for your planner {#pybinding}

The bindings folder follows a similar structure to the `src` folder. For a new planner binding, create a new file under the bindings folder hierarchy at the same location as your planner implementation would be. In that file, write the binding code for your planner. You can follow the existing planner bindings as an example, as well as refer to the [Nanobind documentation](https://nanobind.readthedocs.io). Finally, call your newly added function in `ompl/nanobinds/python.cpp` and rebuild the bindings.

## Planner parameters {#params}

The OMPL Planner class has a method called ompl::base::Planner::declareParam to define parameters that can be changed by the user. It is highly recommended that you use this method for all your planner parameters. It is possible to specify a suggested range of values as an optional argument to ompl::Planner::declareParam. This range can be used by external tools for example to create the appropriate control widgets, so that users can change the parameter values. See ompl::base::GenericParam::rangeSuggestion_ for the syntax used to specify parameter ranges.
