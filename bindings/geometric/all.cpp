#include <nanobind/nanobind.h>

namespace nb = nanobind;

// Forward declarations
void initSimpleSetup(nb::module_&);
void initRRT(nb::module_&);

NB_MODULE(geometric, m) {
    initSimpleSetup(m);
    initRRT(m);
}