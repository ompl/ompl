#include <nanobind/nanobind.h>
#include "ompl/base/Path.h"

namespace nb = nanobind;

void initPath(nb::module_& m) {
    nb::class_<ompl::base::Path>(m, "Path");
}