#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/importance/Greedy.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initImportance_Greedy(nb::module_ &m)
{
    nb::class_<om::BundleSpaceImportanceGreedy, om::BundleSpaceImportance>(m, "Greedy")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
