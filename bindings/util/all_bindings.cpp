#include "nanobind/nanobind.h"

namespace nb = nanobind;

// Forward declarations for initialization functions
// void init_console(nb::module_&);
void init_geometric_equations(nb::module_&);
void init_random_numbers(nb::module_&);
void init_ppm(nb::module_&);
void init_prolate_hyperspheroid(nb::module_&);

NB_MODULE(util, m) {
    // Initialize all submodules
    // init_console(m);
    init_geometric_equations(m);
    init_random_numbers(m);
    init_ppm(m);
    init_prolate_hyperspheroid(m);
}