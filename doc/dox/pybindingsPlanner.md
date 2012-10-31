# Creating Python bindings for a new planner

It is often convenient to test a planner through either the OMPL.app GUI or by using a minimal Python program that defines a simple motion planning problem of interest. In both cases you need to create Python bindings for your planner. We will use RRT* as an example planner to illustrate the steps involved. All the relevant code can be found in \c ompl/src/ompl/contrib/rrt_star. At a high level, the steps are:

- [Create the right directory/file structure](#dirfile)
- [Use the OMPL CMake macros for creating python bindings](#cmake)
- [Use your planner in Python (including the OMPL.app GUI)](#python)
- [Optionally, provide hints for planner parameter ranges that can then be used to create the appropriate controls in the OMPL.app GUI.](#params)


# Directory/file structure {#dirfile}

It is a good idea to keep Python binding files separate from the rest of the code. For RRT* we have created a subdirectory called \c py-bindings. Within this directory, we use the following layout:

- \c CMakeLists.txt - Build system file to generate and compile the python binding code, explained below
- \c generate_bindings.py - A script that relies on the OMPL python module and Py++. It is fairly straightforward to tweak this file for your planners by replacing \c RRTstar and \c BallTreeRRTstar with the names of your planners. If you have a control-based planner, replace occurrences of \c geometric with \c control.
- \c headers_rrtstar.txt - A plain text file that simply contains a list of all header files, one per line, for which Python bindings need to be generated.
- \c ompl/rrtstar/__init__.py - Typically, this is a minimal module initialization file that just imports all the symbols of the binary module that will be created after compilation. It is, of course, possible to add any additional classes or functions that you might need.

# Using the OMPL CMake macros {#cmake}

We have written our own CMake macros for generating python bindings. Although the RRT* code is distributed with OMPL, it should be possible to use the CMake macros even if your project simply uses OMPL as a dependency. You have to make sure that the macros are in the CMake module path by adding a line like this to your CMakeLists.txt:

    set(CMAKE_MODULE_PATH "/path/to/ompl/CMakeModules")

The main steps to create python bindings in your own CMakeLists.txt are then:

    include(PythonBindingUtils)
    # path to python module
    set(OMPL_RRTSTAR_BINDINGS_DIR "...")
    # create target "update_rrtstar_bindings" to generate binding code
    create_module_generation_targets(rrtstar "${CMAKE_CURRENT_SOURCE_DIR}")
    # create target "py_ompl_rrtstar" to compile generated code
    create_module_target(rrtstar ${CMAKE_CURRENT_SOURCE_DIR} "${OMPL_RRTSTAR_BINDINGS_DIR}/ompl")

Please look at \c py-bindings/CMakeLists.txt for a complete example.

# Using your planner in Python (including the OMPL.app GUI) {#python}

As long as your planner derives from ompl::base::Planner, you can use your planner in Python just like any other planner:
~~~{.py}
from ompl import geometric, rrtstar
...
ss = geometric.SimpleSetup(space)
planner = rrtstar(ss.getSpaceInformation())
ss.setPlanner(planner)
...
~~~
Optionally, you can add your planners to the list of known geometric (or control-based) planners:
~~~{.py}
from ompl import geometric, rrtstar
import ompl
ompl.initializePlannerLists()
geometric.planners.addPlanner('ompl.rrtstar.RRTstar')
geometric.planners.addPlanner('ompl.rrtstar.BallTreeRRTstar')
~~~
This is already done in \c ompl_app.py for RRT*, but you can easily add similar lines for your own planners. Doing so will allow you to select your planners in the GUI.

# Planner parameters {#params}

The OMPL Planner class a method called ompl::base::Planner::declareParam to define parameters that can be changed by the user. It is highly recommended that you use this method for all your planner parameters. It is possible to specify a suggested range of values as an optional argument to ompl::Planner::declareParam. This range will be used by the OMPL.app GUI to create the appropriate controls, so that users can change the parameter values through the GUI. See ompl::base::GenericParam::rangeSuggestion_ for the syntax used to specify parameter ranges.
