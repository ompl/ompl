#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include "ompl/control/planners/ltl/PropositionalDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersLtl_PropositionalDecomposition(nb::module_ &m)
{
    struct PyPropositionalDecomposition : oc::PropositionalDecomposition
    {
        NB_TRAMPOLINE(oc::PropositionalDecomposition, 2);

        oc::World worldAtRegion(int rid) override
        {
            NB_OVERRIDE_PURE(worldAtRegion, rid);
        }

        int getNumProps() const override
        {
            NB_OVERRIDE_PURE(getNumProps);
        }
    };

    nb::class_<oc::PropositionalDecomposition, oc::Decomposition, PyPropositionalDecomposition>(m,
                                                                                                "PropositionalDecompos"
                                                                                                "ition")
        .def(nb::init<const oc::DecompositionPtr &>(), nb::arg("decomp"))
        .def("worldAtRegion", &oc::PropositionalDecomposition::worldAtRegion, nb::arg("rid"))
        .def("getNumProps", &oc::PropositionalDecomposition::getNumProps)
        .def("getNumRegions", &oc::PropositionalDecomposition::getNumRegions)
        .def("getRegionVolume", &oc::PropositionalDecomposition::getRegionVolume, nb::arg("rid"))
        .def("locateRegion", &oc::PropositionalDecomposition::locateRegion, nb::arg("s"))
        .def("project", &oc::PropositionalDecomposition::project, nb::arg("s"), nb::arg("coord"))
        .def("getNeighbors", &oc::PropositionalDecomposition::getNeighbors, nb::arg("rid"), nb::arg("neighbors"));
}
