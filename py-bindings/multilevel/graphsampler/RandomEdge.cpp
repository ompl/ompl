#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/graphsampler/RandomEdge.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initGraphsampler_RandomEdge(nb::module_ &m)
{
    nb::class_<om::BundleSpaceGraphSamplerRandomEdge, om::BundleSpaceGraphSampler>(m, "RandomEdge")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
