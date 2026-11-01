#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "ompl/geometric/planners/xxl/XXLDecomposition.h"
#include "PyXXLDecomposition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    class PyXXLDecomposition : public ompl::binding::geometric::PyXXLDecompositionT<og::XXLDecomposition, 7>
    {
    public:
        int getNumRegions() const override
        {
            NB_OVERRIDE_PURE(getNumRegions);
        }

        int getDimension() const override
        {
            NB_OVERRIDE_PURE(getDimension);
        }

        int locateRegion(const ob::State *s) const override
        {
            NB_OVERRIDE_PURE(locateRegion, s);
        }

        int locateRegion(const std::vector<double> &coord) const override
        {
            NB_OVERRIDE_PURE_NAME("locateRegionCoord", locateRegion, coord);
        }

        void getNeighbors(int rid, std::vector<int> &neighbors) const override
        {
            neighbors = callPure<std::vector<int>>("getNeighbors", rid);
        }

        void getNeighborhood(int rid, std::vector<int> &neighborhood) const override
        {
            nb::detail::ticket nb_ticket(nb_trampoline, "getNeighborhood", false);
            if (!nb_ticket.key.is_valid())
                return og::XXLDecomposition::getNeighborhood(rid, neighborhood);
            neighborhood = nb::cast<std::vector<int>>(nb_trampoline.base().attr(nb_ticket.key)(rid));
        }

        double distanceHeuristic(int r1, int r2) const override
        {
            NB_OVERRIDE_PURE(distanceHeuristic, r1, r2);
        }
    };
}  // namespace

void ompl::binding::geometric::initPlannersXxl_XXLDecomposition(nb::module_ &m)
{
    nb::class_<og::XXLDecomposition, PyXXLDecomposition>(m, "XXLDecomposition")
        .def(nb::init<>())
        .def("getNumRegions", &og::XXLDecomposition::getNumRegions)
        .def("getDimension", &og::XXLDecomposition::getDimension)
        .def("numLayers", &og::XXLDecomposition::numLayers)
        .def("locateRegion", nb::overload_cast<const ob::State *>(&og::XXLDecomposition::locateRegion, nb::const_),
             nb::arg("s"))
        .def("locateRegionCoord",
             nb::overload_cast<const std::vector<double> &>(&og::XXLDecomposition::locateRegion, nb::const_),
             nb::arg("coord"))
        .def(
            "getNeighbors",
            [](const og::XXLDecomposition &d, int rid)
            {
                std::vector<int> neighbors;
                d.getNeighbors(rid, neighbors);
                return neighbors;
            },
            nb::arg("rid"))
        .def(
            "getNeighborhood",
            [](const og::XXLDecomposition &d, int rid)
            {
                std::vector<int> neighborhood;
                d.getNeighborhood(rid, neighborhood);
                return neighborhood;
            },
            nb::arg("rid"))
        .def("distanceHeuristic", &og::XXLDecomposition::distanceHeuristic, nb::arg("r1"), nb::arg("r2"))
        .def(
            "sampleFromRegion", [](const og::XXLDecomposition &d, int r, ob::State *s, const ob::State *seed)
            { return d.sampleFromRegion(r, s, seed); }, nb::arg("r"), nb::arg("s"), nb::arg("seed").none() = nb::none())
        .def(
            "sampleFromRegionLayer",
            [](const og::XXLDecomposition &d, int r, ob::State *s, const ob::State *seed, int layer)
            { return d.sampleFromRegion(r, s, seed, layer); }, nb::arg("r"), nb::arg("s"), nb::arg("seed").none(),
            nb::arg("layer"))
        .def(
            "project",
            [](const og::XXLDecomposition &d, const ob::State *s, int layer)
            {
                std::vector<double> coord;
                d.project(s, coord, layer);
                return coord;
            },
            nb::arg("s"), nb::arg("layer") = 0)
        .def(
            "projectLayers",
            [](const og::XXLDecomposition &d, const ob::State *s)
            {
                std::vector<int> layers;
                d.project(s, layers);
                return layers;
            },
            nb::arg("s"))
        .def("canSteer", &og::XXLDecomposition::canSteer);
}
