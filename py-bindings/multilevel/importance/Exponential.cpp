#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/importance/Exponential.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initImportance_Exponential(nb::module_ &m)
{
    nb::class_<om::BundleSpaceImportanceExponential, om::BundleSpaceImportance>(m, "Exponential")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
