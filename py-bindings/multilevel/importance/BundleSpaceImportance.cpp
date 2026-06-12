#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/importance/BundleSpaceImportance.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initImportance_BundleSpaceImportance(nb::module_ &m)
{
    nb::class_<om::BundleSpaceImportance>(m, "BundleSpaceImportance")
        .def("eval", &om::BundleSpaceImportance::eval)
        .def("clear", &om::BundleSpaceImportance::clear);
}
