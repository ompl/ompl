The Open Motion Planning Library (OMPL)
=======================================

**OMPL** is a free sampling-based motion planning library with **VAMP integration** for high-performance collision checking using SIMD acceleration.

Continuous Integration Status
-----------------------------

[![Build](https://github.com/ompl/ompl/actions/workflows/build.yml/badge.svg?branch=pr-github-actions)](https://github.com/ompl/ompl/actions/workflows/build.yml)
[![Format](https://github.com/ompl/ompl/actions/workflows/format.yml/badge.svg?branch=pr-github-actions)](https://github.com/ompl/ompl/actions/workflows/format.yml?branch=pr-github-actions)

Installation
------------

Visit the [OMPL installation page](https://ompl.kavrakilab.org/core/installation.html) for
detailed installation instructions.

OMPL has the following required dependencies:

* [Boost](https://www.boost.org) (version 1.68 or higher)
* [CMake](https://www.cmake.org) (version 3.12 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp) - Used for parsing YAML configuration files, required for VAMP demos

The following dependencies are optional:

* [**VAMP**](https://github.com/KavrakiLab/vamp) (enabled by default) - Vector-Accelerated Motion Planning for high-performance collision checking with SIMD optimization
* [Py++](https://github.com/ompl/ompl/blob/main/doc/markdown/installPyPlusPlus.md) (needed to generate Python bindings)
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  https://ompl.kavrakilab.org/core)
* [Flann](https://github.com/flann-lib/flann/tree/1.9.2) (FLANN can be used for nearest neighbor queries by OMPL)
* [Spot](http://spot.lrde.epita.fr) (Used for constructing finite automata from LTL formulae.)

Once dependencies are installed, you can build OMPL on Linux, macOS,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands:

    git submodule update --init --recursive  # for VAMP integration
    mkdir -p build/Release
    cd build/Release
    cmake ../..
    # next step is optional
    make -j 4 update_bindings # if you want Python bindings
    make -j 4 # replace "4" with the number of cores on your machine
