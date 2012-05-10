The Open Motion Planning Library (OMPL)

Visit http://ompl.kavrakilab.org/core/installation.html for
detailed installation instructions.

OMPL has the following required dependencies:
 * Boost (version 1.44 or higher)
 * CMake (version 2.8.2 or higher)

The following dependencies are optional:
 * ODE (needed to compile support for planning using the Open Dynamics Engine)
 * Py++ (version from repository, needed to generate Python bindings)
 * Doxygen (needed to create a local copy of the documentation at
   http://ompl.kavrakilab.org/core)

Once dependencies are installed, you can build OMPL on Linux, OS X,
and MS Windows. Go to the top-level directory of OMPL and type the
following commands:
   mkdir -p build/Release
   cd build/Release
   cmake ../..
   # next two steps are optional
   make installpyplusplus && cmake . # download & install Py++
   make update_bindings # if you want Python bindings
   make -j 4 # replace "4" with the number of cores on your machine
