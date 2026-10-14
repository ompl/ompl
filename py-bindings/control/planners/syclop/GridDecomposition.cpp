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

        // The vector caster copies coord, so the override gets a list to fill in place (or returns the coordinates).
        void project(const ob::State *state, std::vector<double> &coord) const override
        {
            nb::detail::ticket nb_ticket(nb_trampoline, "project", true);
            nb::list out;
            for (int i = 0; i < getDimension(); ++i)
                out.append(0.0);
            nb::object result = nb_trampoline.base().attr(nb_ticket.key)(state, out);
            coord = nb::cast<std::vector<double>>(result.is_none() ? nb::handle(out) : nb::handle(result));
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
        .def(
            "getNeighbors",
            [](const oc::GridDecomposition &d, int rid)
            {
                std::vector<int> neighbors;
                d.getNeighbors(rid, neighbors);
                return neighbors;
            },
            nb::arg("rid"))
        .def("locateRegion", &oc::GridDecomposition::locateRegion, nb::arg("state"))
        .def(
            "sampleFromRegion",
            [](const oc::GridDecomposition &d, int rid, ompl::RNG &rng)
            {
                std::vector<double> coord;
                d.sampleFromRegion(rid, rng, coord);
                return coord;
            },
            nb::arg("rid"), nb::arg("rng"));
}
