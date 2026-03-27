The Open Motion Planning Library (OMPL)
=======================================

**OMPL** is an open source sampling-based motion planning library

- Over 40 sampling-based planning algorithms (RRT-Connect, PRM, KPIECE, RRT*, and **many more**) across more than 20 state spaces (SE(3), Euclidean space, and others)
- Easily extensible to custom planners in both Python and C++
- SIMD-accelerated planning with VAMP for **millisecond planning** in both Python and C++

Installation
------------

Visit the [OMPL installation page](https://ompl.kavrakilab.org/core/installation.html) for
detailed installation instructions.

OMPL has the following required dependencies:

* [Boost](https://www.boost.org) (version 1.68 or higher)
* [CMake](https://www.cmake.org) (version 3.12 or higher)
* [Eigen](http://eigen.tuxfamily.org) (version 3.3 or higher)

The following dependencies are optional:

* [**VAMP**](https://github.com/KavrakiLab/vamp) (enabled by default) - Vector-Accelerated Motion Planning for high-performance collision checking with SIMD optimization
* [Doxygen](http://www.doxygen.org) (needed to create a local copy of the documentation at
  https://ompl.kavrakilab.org/core)
* [Flann](https://github.com/flann-lib/flann/tree/1.9.2) (FLANN can be used for nearest neighbor queries by OMPL)
* [Spot](http://spot.lrde.epita.fr) (Used for constructing finite automata from LTL formulae.)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp) (Used for reading and writing YAML world descriptions in the PlanarManipulator demos)

Once dependencies are installed, you can build OMPL on Linux, macOS,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands:

    git submodule update --init --recursive 
    mkdir -p build/Release
    cd build/Release
    cmake ../..
    make -j <num_cores> # replace <num_cores> with the number of cores on your machine

To install the Python bindings, go to the top-level directory of OMPL and type the following commands:
```
git submodule update --init --recursive
cd py-bindings
pip install . 
```
