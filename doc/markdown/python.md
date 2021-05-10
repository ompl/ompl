# Python Bindings {#python}

[TOC]

Almost all of the functionality of the C++ OMPL library is accessible through Python using more or less the same API. Some important differences will be described below. The Python bindings are generated with [Py++](https://github.com/ompl/pyplusplus), which relies on [Boost.Python](https://www.boost.org/doc/libs/release/libs/python/doc). The bindings are packaged in the ompl module. The main namespaces (ompl::base, ompl::control, ompl::geometric) are available as sub-modules. To quickly get an idea of what classes, functions, etc., are available within each submodule, type something like this at the Python prompt:

~~~{.py}
from ompl import base, control, geometric, tools, util
dir(base), dir(control), dir(geometric), dir(tools), dir(util)
~~~

## Usage of the Python bindings: Good practices {#py_good_practices}

Although almost all C++ functionality is exposed to Python, there are some caveats to be aware off:

- By default OMPL often returns a reference to an internal object when the original C++ function or method returns a reference or takes a reference as input. This means that you have to be careful about the scope of variables. Python objects need to exist as long as there exist at least one OMPL object that contains a reference to it. Analogously, you should not try to use Python variables that point to OMPL objects that have already been destroyed.
- C++ threads and Boost.Python don't mix; see the [Boost.Python FAQ](https://www.boost.org/libs/python/doc/v2/faq.html). If you have multi-threaded C++ code where multiple threads can call the Python interpreter at the same time, then this can lead to crashes. For that reason, we do not have Python bindings for the parallelized RRT and SBL planners, because they can call a Python state validator function at the same time.
- Just because you \em can create Python classes that derive from C++ classes, this doesn't mean it is a good idea. You pay a performance penalty each time your code crosses the Python-C++ barrier (objects may need to copied, locks acquired, etc.). This means that it is an especially bad idea to override low-level classes. For low-level functionality it is best to stick to the built-in OMPL functionality and use just the callback functions (e.g., for state validation and state propagation). It is also highly recommended to use the ompl::geometric::SimpleSetup and ompl::control::SimpleSetup classes rather than the lower-level classes for that same reason.

## Important differences between C++ and Python {#cpp_py_diffs}

- There are no templates in Python, so templated C++ classes and functions need to be fully instantiated to allow them to be exposed to python.
- There are no C-style pointers in python, and no “new” or “delete” operators. This could be a problem, but can be dealt with mostly by using [shared_ptr](https://en.cppreference.com/w/cpp/memory/shared_ptr)'s. If a C++ function takes pointer input/output parameters, _usually_ a reference to the object is passed in the python bindings. In other words, you should be able to get and set the current value of an object through its methods. In some cases odd side effects may occur if you pass temporary objects (e.g., `function_call(Constructor_call())`), so it's advisable to create variables with the appropriate scope.

## Differences between the C++ and Python API's {#py_api_diffs}

- An STL vector of int's is of type vectorInt in Python (analogously for other types).
- The C++ class State has been renamed AbstractState, while the C++ class ScopedState\<\> is called State in Python.
- The C++ class ScopedState<RealVectorStateSpace> is called RealVectorState. The ScopedState's for the other pre-defined state spaces have been renamed analogously.
- The C++ class RealVectorStateSpace::StateType has been renamed to RealVectorStateInternal (analogously for the other state space types), to emphasize that an end user should really be using RealVectorState.
- To get a reference to a C++ State stored inside a ScopedState from Python, use the "()" operator:
  ~~~{.py}
  spc = SE3StateSpace()  # create an SE(3) state space
  s = State(spc)         # allocate a state corresponding to a C++ ScopedState<SE3StateSpace>
  sref = s()             # sref corresponds to a C++ State*. In this case sref is
                         # a reference to SE3StateSpace::StateType
  sref.setX(1.0)         # set the X coordinate
  s().setX(1.0)          # this also works
  ~~~

- The print method (for classes that have one) is mapped to the special python method __str__, so a C++ call like `foo.print(std::cout)` becomes `print(foo)` in python. Similarly, a C++ call like `foo.printSettings(std::cout)` becomes `print(foo.settings())` in python.
- The code for constrained motion planning heavily relies on the [Eigen C++ library](http://eigen.tuxfamily.org/index.php?title=Main_Page). Input and output arguments of type `Eigen::Ref<Eigen::VectorXd>` or `Eigen::Ref<Eigen::MatrixXd>` are automatically converted to `numpy.array` types. This is done without copying data; under the hood there are simply wrappers that pass pointers to the raw data. These wrappers still need to be dynamically allocated and freed, so do not expect constrained planning in Python to be very fast. See the Python demos in `ompl/demos/constraint` for some examples.

Many of the python demo and test programs are direct ports of the corresponding C++ programs. If you compare these programs, the sometimes subtle differences will become more obvious. In the python programs you will notice that we can create python classes that derive from C++ classes and pass instances of such classes to C++ functions. Similarly, we can create python functions (such as state validity checkers or propagate functions) that can be called by C++ code.

## A simple example {#py_example}

Below is a simple annotated example. It is available in ompl/py-bindings/demos/RigidBodyPlanning.py.

~~~{.py}
from ompl import base as ob
from ompl import geometric as og

def isStateValid(state):
    # "state" is of type SE2StateInternal, so we don't need to use the "()"
    # operator.
    #
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
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    start = ob.State(space)
    # we can pick a random start state...
    start.random()
    # ... or set specific values
    start().setX(.5)

    goal = ob.State(space)
    # we can pick a random goal state...
    goal.random()
    # ... or set specific values
    goal().setX(-.5)

    ss.setStartAndGoalStates(start, goal)

    # this will automatically choose a default planner with
    # default parameters
    solved = ss.solve(1.0)

    if solved:
        # try to shorten the path
        ss.simplifySolution()
        # print the simplified path
        print ss.getSolutionPath()


if __name__ == "__main__":
    plan()
~~~

## Creating std::function objects from Python functions {#pyfunction_to_stdfunction}

OMPL relies heavily on std::function objects for callback functions. To specify a Python function as a callback function, that function needs to be cast to the right function type. The simple example above already showed how to do this for a state validity checker function:

~~~{.py}
ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
~~~

If you need to pass extra arguments to a function, you can use the Python `partial` function like so:

~~~{.py}
from functools import partial

def isStateValid(spaceInformation, state):
    return spaceInformation.satiesfiesBounds(state)

def plan()
    ...
    isValidFn = ob.StateValidityCheckerFn(partial(isStateValid, ss.getSpaceInformation()))
    ss.setStateValidityChecker(isValidFn)
    ...
~~~

### (Re)generating the Python bindings {#updating_python_bindings}

\attention See also the tutorial [Creating Python bindings for a new planner](pybindingsPlanner.html)

The Python bindings are subdivided into modules, to reflect the main namespaces: ompl::base, ompl::control, and ompl::geometric. The code in the ompl/src/ompl/util directory is available in a submodule as well. Whenever you change the API to OMPL, you will need to update the Python bindings. Updating the bindings is a two-step process. First, the code for the modules needs to be generated. Second, the code needs to be compiled into binary Python modules.

#### Code generation {#binding_code_generation}

__The code for the Python bindings can be generated by typing “`make update_bindings`.”__ This creates one header file per module, formed by concatenating all relevant header files for that module. This header file is then parsed by Py++ and the appropriate C++ code is generated. This code uses Boost.Python. Py++ is smart enough to create wrapper classes when necessary, register Python \<-\> C++ type conversions, and so on. If you only need to update the bindings for one module (say `base`), you can type “`make update_base_bindings`.” Any diagnostic information from Py++ is stored in a log file in the build directory for each module (`pyplusplus_base.log` for the module `base` and likewise for other modules). You can remove all generated code by typing “`make clean_bindings`”. This is sometimes necessary if Py++ gets confused about which code needs to be regenerated. If this happens, all the generated code might still compile, but you can get odd crashes or errors at runtime.

For each module the relevant header files are listed in `ompl/py-bindings/headers_<modulename>.txt.` The order in which the header files are listed is important. A header file should not be included by another header file listed above it. If you have created new header files, you should add the names of these files to the appropriate `headers_<modulename>.txt.` file.

### Compiling the Python modules {#compile_bindings}

To compile the Python modules type “`make py_ompl`” (or simply “make”). If you only want to compile one python module (say `base`), type “`make py_ompl_base`.” The modules will appear as libraries in the lib subdirectory in the build directory, but they are also copied to `ompl/py-bindings/ompl/<modulename>/_<modulename>.so`.

#### Forcing CMake to do The Right Thing {#bindings_cmake}

Every attempt has been to have CMake correctly identify dependencies and only compile code when necessary. If you want force CMake to regenerate the bindings from scratch, you can type “`make clean_bindings`,” followed by “`make update_bindings`” again. If, on the other hand, you want to (temporarily) disable the compilation of Python bindings, type:

~~~{.sh}
cmake -D OMPL_BUILD_PYBINDINGS:BOOL=OFF .
~~~

in your build directory. You can re-enable them, by running this command again, but with OFF changed to ON. Changing these settings can also be done through the CMake GUI.
