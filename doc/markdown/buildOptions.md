# Build Options {#buildOptions}

If you are building OMPL from source, there are several options that you can use to configure how OMPL is compiled. The options can be set via the command line like so: `cmake -D<option>=ON ../..`. If you previously ran cmake in your build directory, it is safest to remove the CMakeCache.txt before re-running `cmake` with different options. Below is a list of all options.

| Option                        | Default value | Description |
|-------------------------------|---------------|-----------------------------------------------------------------|
| OMPL_BUILD_DEMOS              | ON            | Compile the OMPL demo programs. (The binaries are never installed.) |
| OMPL_BUILD_PYBINDINGS         | ON            | Whether to compile the Python bindings (requires Py++). |
| OMPL_BUILD_PYTESTS            | ON            | Whether the Python tests should be added to the `test` target. |
| OMPL_BUILD_TESTS              | ON            | Wether to compile the C++ unit tests |
| OMPL_REGISTRATION             | ON            | Whether the registration page is shown. (Disabling it might be useful for build bots.) |
| OMPL_VERSIONED_INSTALL        | ON            | Install header files in include/ompl-X.Y/ompl, where X and Y are the major and minor version numbers. |

There also several optional dependencies. By default, if an optional dependency is detected by `cmake`, support for this dependency is enabled. If this is not what you want, you can run `cmake` like so:

     cmake -DCMAKE_DISABLE_FIND_PACKAGE_<PackageName>=ON ../..

where `<PackageName>` is, e.g., `pypy`, `flann`, `spot`, etc.
