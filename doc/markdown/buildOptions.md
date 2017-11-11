# Build Options

If you are building OMPL from source, there are several options that you can use to configure how OMPL is compiled. The options can be set via the command line like so: `cmake -D<option>=ON ../..`. If you previously ran cmake in your build directory, it is safest to remove the CMakeCache.txt before re-running cmake with different options. Below is a list of all options.

Option                        | Default value | Description
------------------------------|---------------|-----------------------------------------------------------------
OMPLAPP_PQP                   | ON            | Enable support for the [PQP](http://gamma.cs.unc.edu/SSV) collision checking library (in addition to [FCL](http://gamma.cs.unc.edu/FCL)). Available only in the OMPL.app package.
OMPL_BUILD_DEMOS              | ON            | Compile the OMPL demo programs. (The binaries are never installed.)
OMPL_BUILD_PYBINDINGS         | ON            | Whether to compile the Python bindings (requires Py++).
OMPL_BUILD_PYTESTS            | ON            | Whether the Python tests should be added to the `test` target.
OMPL_BUILD_TESTS              | ON            | Wether to compile the C++ unit tests
OMPL_REGISTRATION             | ON            | Whether the registration page is shown. (Disabling it might be useful for build bots.)
OMPL_USE_PYPY                 | OFF           | Whether [PyPy](https://pypy.org) is used to generate the Python bindings. This can greatly speed up this process. It's enabled by default if PyPy was found by CMake. If PyPy is not installed in the default path, you can specify the path on the command line: `cmake -DPYPY=~/pypy3-v5.9.0-linux64/bin/pypy3 -DPYTHON_EXEC=/usr/bin/python3 ../..`
OMPL_VERSIONED_INSTALL        | OFF           | Install header files in include/ompl-X.Y/ompl, where X and Y are the major and minor version numbers.
