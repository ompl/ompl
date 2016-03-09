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
OMPL_VERSIONED_INSTALL        | OFF           | Whether directories and executables created by `make install` have a version suffix. Useful if you want to install multiple versions of OMPL in the same directory.
