#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/importance/Uniform.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initImportance_Uniform(nb::module_ &m)
{
    nb::class_<om::BundleSpaceImportanceUniform, om::BundleSpaceImportance>(m, "Uniform")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
