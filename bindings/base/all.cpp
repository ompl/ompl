#include <nanobind/nanobind.h>

namespace nb = nanobind;

// Forward declarations
void initState(nb::module_&);
void initSE2StateSpace(nb::module_&);
void initRealVectorStateSpace(nb::module_&);

NB_MODULE(base, m) {
    initState(m);
    initSE2StateSpace(m);
    initRealVectorStateSpace(m);
}