#include <nanobind/nanobind.h>

namespace nb = nanobind;

// Forward declarations
void initSimpleSetup(nb::module_&);

NB_MODULE(geometric, m) {
    initSimpleSetup(m);
}