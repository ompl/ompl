#include <nanobind/nanobind.h>

namespace nb = nanobind;

// Forward declarations
void initSimpleSetup(nb::module_&);

NB_MODULE(_geometric_ext, m) {
    initSimpleSetup(m);
}