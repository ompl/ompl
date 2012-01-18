Visit http://ompl.kavrakilab.org/core/installation.html for
detailed installation instructions.

OMPL has the following required dependencies:
 * Boost (version 1.44 or higher)
 * CMake (version 2.8.2 or higher)

The following dependencies are optional:
 * ODE (needed to compile support for planning using ODE)
 * Google-test (needed to run the test programs)
 * Py++ (needed to generate Python bindings)
 * Doxygen (needed to create a local copy of the documentation at
   http://ompl.kavrakilab.org/core)

Once dependencies are installed, you can build OMPL on Linux and OS X.
Go to the top-level directory of OMPL and type the following commands:
   mkdir -p build/Release
   cd build/Release
   cmake -DCMAKE_BUILD_TYPE=Release ../..
   make update_bindings  # if you want Python bindings and have Py++ installed
   make -j 4 # replace "4" with the number of cores on your machine

