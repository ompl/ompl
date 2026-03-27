#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSyclop_GridDecomposition(nb::module_ &m)
{
    struct PyGridDecomposition : oc::GridDecomposition
    {
        // virtual functions: project(), sampleFullState()
        NB_TRAMPOLINE(oc::GridDecomposition, 2);

        void project(const ob::State *state, std::vector<double> &coord) const override
        {
            NB_OVERRIDE_PURE(project, state, coord);
        }
        void sampleFullState(const ob::StateSamplerPtr &sampler, const std::vector<double> &coord,
                             ob::State *s) const override
        {
            NB_OVERRIDE_PURE(sampleFullState, sampler, coord, s);
        }
    };

    nb::class_<oc::GridDecomposition, oc::Decomposition, PyGridDecomposition /* <-- trampoline */>(m, "GridDecompositio"
                                                                                                      "n")
        .def(nb::init<int, int, const ob::RealVectorBounds &>(), nb::arg("len"), nb::arg("dim"), nb::arg("bounds"))
        .def("getDimension", &oc::GridDecomposition::getDimension)
        .def("getBounds", &oc::GridDecomposition::getBounds, nb::rv_policy::reference_internal)
        .def("getNumRegions", &oc::GridDecomposition::getNumRegions)
        .def("getRegionVolume", &oc::GridDecomposition::getRegionVolume, nb::arg("region"))
        .def("getNeighbors", &oc::GridDecomposition::getNeighbors, nb::arg("rid"), nb::arg("neighbors"))
        .def("locateRegion", &oc::GridDecomposition::locateRegion, nb::arg("state"))
        .def("sampleFromRegion", &oc::GridDecomposition::sampleFromRegion, nb::arg("rid"), nb::arg("rng"),
             nb::arg("coord"));
}
