#include <nanobind/nanobind.h>

namespace nb = nanobind;

// Forward declarations
void initState(nb::module_&);
void initStateSpace(nb::module_&);
void initSE2StateSpace(nb::module_&);
void initRealVectorStateSpace(nb::module_&);
void initRealVectorBounds(nb::module_&);

NB_MODULE(base, m) {
    initState(m);
    initStateSpace(m);
    initSE2StateSpace(m);
    initRealVectorStateSpace(m);
    initRealVectorBounds(m);
}