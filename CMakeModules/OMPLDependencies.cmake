# Eigen3
set_package_properties(Eigen3 PROPERTIES
    URL "http://eigen.tuxfamily.org"
    PURPOSE "A linear algebra library used throughout OMPL.")
find_package(Eigen3 REQUIRED NO_MODULE)

# Boost
set_package_properties(Boost PROPERTIES
    URL "https://www.boost.org"
    PURPOSE "Used throughout OMPL for data serialization, graphs, etc.")
set(Boost_USE_MULTITHREADED ON)
find_package(Boost 1.68 REQUIRED COMPONENTS serialization program_options)

# Threads
set_package_properties(Threads PROPERTIES
    URL "https://en.wikipedia.org/wiki/POSIX_Threads"
    PURPOSE "Pthreads is sometimes needed, depending on OS / compiler.")
find_package(Threads QUIET)

# Triangle
set_package_properties(Triangle PROPERTIES
    URL "http://www.cs.cmu.edu/~quake/triangle.html"
    PURPOSE "Used to create triangular decompositions of polygonal 2D environments.")
find_package(Triangle QUIET)
set(OMPL_EXTENSION_TRIANGLE ${TRIANGLE_FOUND})

# FLANN
set_package_properties(flann PROPERTIES
    URL "https://github.com/mariusmuja/flann"
    PURPOSE "If detetected, FLANN can be used for nearest neighbor queries by OMPL.")
find_package(flann CONFIG 1.9.2 QUIET)
if (FLANN_FOUND)
    set(OMPL_HAVE_FLANN 1)
else ()
    set(OMPL_HAVE_FLANN 0)
endif()

# Spot
set_package_properties(spot PROPERTIES
    URL "http://spot.lrde.epita.fr"
    PURPOSE "Used for constructing finite automata from LTL formulae.")
find_package(spot)
if (spot_FOUND)
    set(OMPL_HAVE_SPOT 1)
else ()
    set(OMPL_HAVE_SPOT 0)
endif()

# Doxygen
set_package_properties(Doxygen PROPERTIES
    URL "http://doxygen.org"
    PURPOSE "Used to create the OMPL documentation (i.e., https://ompl.kavrakilab.org).")
find_package(Doxygen)

# R
set_package_properties(R PROPERTIES
    URL "https://www.r-project.org"
    PURPOSE "Needed for running Planner Arena locally.")
find_program(R_EXEC R)

# Python
set_package_properties(Python PROPERTIES
    URL "https://www.python.org"
    PURPOSE "Used for python bindings.")
find_package(Python QUIET)

