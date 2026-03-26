# Python Bindings {#python}

[TOC]

The python bindings were designed to mirror the C++ OMPL API as closely as possible, so if you are interested in using the bindings, it is highly recommended to start by checking out the python demos in `ompl/demos`. In most cases, the workflow should mirror a corresponding C++ implementation with the expected differences in syntax. The Python bindings are generated using [Nanobind](https://nanobind.readthedocs.io) which creates the `ompl` Python module. The main namespaces (ompl::base, ompl::control, ompl::geometric) are available as sub-modules. To quickly get an idea of what classes, functions, etc., are available within each submodule, type something like this at the Python prompt:

~~~{.py}
from ompl import base, control, geometric, tools, util
dir(base), dir(control), dir(geometric), dir(tools), dir(util)
~~~

## A simple example {#py_example}

Below is a simple annotated example. It is available in ompl/py-bindings/demos/RigidBodyPlanning.py.

~~~{.py}
from ompl import base as ob
from ompl import geometric as og

def isStateValid(state):
    # Some arbitrary condition on the state (note that thanks to
    # dynamic type checking we can just call getX() and do not need
    # to convert state to an SE2State.)
    return state.getX() < .6

def plan():
    # create an SE2 state space
    space = ob.SE2StateSpace()

    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(isStateValid)

    sampler = ss.getStateSpace().allocDefaultStateSampler()
    start = ss.getStateSpace().allocState()
    # we can pick a random start state...
    sampler.sampleUniform(start)
    # ... or set specific values
    start.setX(.5)

    goal = ss.getStateSpace().allocState()
    # we can pick a random goal state...
    sampler.sampleUniform(goal)
    # ... or set specific values
    goal.setX(-.5)

    ss.setStartAndGoalStates(start, goal)

    # this will automatically choose a default planner with
    # default parameters
    solved = ss.solve(1.0)

    if solved:
        # try to shorten the path
        ss.simplifySolution()
        # print the simplified path
        print(ss.getSolutionPath())


if __name__ == "__main__":
    plan()
~~~

## Differences between the C++ and Python API's {#py_api_diffs}

- C++ basic types and standard library containers are converted to analogous basic types and arrays in python.
- There is no need to call `freeState()` in Python, since python automatically cleans up object references that go out of scope. Similarly, the class `ScopedState<>` is not included in the bindings. Instead, use `allocState()` to create the desired state and it will be automatically cleaned up when there are no more references to it. Since the bindings support inheritance, you can also use members and call functions of the most derived state type it can be cast to, without manually calling `as<>()`.

- The print method (for classes that have one) is mapped to the special python method `__repr__`, so a C++ call like `foo.print(std::cout)` becomes `print(foo)` in python. Similarly, a C++ call like `foo.printSettings(std::cout)` becomes `print(foo.settings())` in python.
- The code for constrained motion planning heavily relies on the [Eigen C++ library](http://eigen.tuxfamily.org/index.php?title=Main_Page). Input and output arguments of type `Eigen::Ref<Eigen::VectorXd>` or `Eigen::Ref<Eigen::MatrixXd>` are automatically converted to `numpy.array` types. This is done without copying data; under the hood there are simply wrappers that pass pointers to the raw data. These wrappers still need to be dynamically allocated and freed, so do not expect constrained planning in Python to be very fast. See the Python demos in `ompl/demos/ConstrainedPlanningSphere.py` for an example.

Many of the python demo and test programs are direct ports of the corresponding C++ programs. If you compare these programs, the sometimes subtle differences will become more obvious. In the python programs you will notice that we can create python classes that derive from C++ classes and pass instances of such classes to C++ functions. Similarly, we can create python functions (such as state validity checkers or propagate functions) that can be called by C++ code.

## (Re)generating the Python bindings {#updating_python_bindings}

\attention See also the tutorial [Creating Python bindings for a new planner](pybindingsPlanner.html)

The Python bindings are subdivided into modules, to reflect the main namespaces: ompl::base, ompl::control, and ompl::geometric. The code in the ompl/src/ompl/util directory is available in a submodule as well. Whenever you change the API to OMPL, you will need to update the Python bindings. Updating the bindings is a two-step process. First, the code for the modules needs to be generated. Second, the code needs to be compiled into binary Python modules.

Two options are offered for generating the python bindings. For the simplest one, you can switch your current directory to the `ompl/py-bindings` folder and run `pip install .`, which will install the module to your currently active python environment.

Alternatively, you can install the module at an external location by specifying the `OMPL_PYTHON_INSTALL_PREFIX` cmake variable and run cmake build as usual. In that case, you will need to set your PYTHONPATH environment variable to the same path so that python can find the module

## Usage of the Python bindings: Good practices {#py_good_practices}

Although almost all C++ functionality is exposed to Python, there are some caveats to be aware of:

- C++ threads and python code don't mix well. If you have multi-threaded C++ code where multiple threads can call the Python interpreter at the same time, then this can lead to deadlocks or crashes. This is because in python only one thread can execute code, unless a free-threaded version of python is used, see the [Nanobind thread guide](https://nanobind.readthedocs.io/en/latest/free_threaded.html) for a longer discussion.
- Just because you \em can create Python classes that derive from C++ classes, this doesn't mean it is a good idea. You pay a performance penalty each time your code crosses the Python-C++ barrier (objects may need to be copied, locks acquired, etc.). This means that it is an especially bad idea to override low-level classes. For low-level functionality it is best to stick to the built-in OMPL functionality and use just the callback functions (e.g., for state validation and state propagation). It is also highly recommended to use the ompl::geometric::SimpleSetup and ompl::control::SimpleSetup classes rather than the lower-level classes for that same reason.
