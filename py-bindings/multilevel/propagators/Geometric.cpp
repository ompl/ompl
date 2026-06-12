#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/propagators/Geometric.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initPropagators_Geometric(nb::module_ &m)
{
    nb::class_<om::BundleSpacePropagatorGeometric, om::BundleSpacePropagator>(m, "Geometric")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
