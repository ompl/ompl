#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/constraint/AtlasChart.h"
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesConstraint_AtlasChart(nb::module_ &m)
{

    // TODO [ob::AtlasChart][TEST]
    nb::class_<ob::AtlasChart>(m, "AtlasChart")
        // Constructor: (atlas_space, state)
        .def(
            "__init__",
            [](ob::AtlasChart &self, const ob::AtlasStateSpacePtr &atlas, const ob::State *state)
            {
                // Downcast the ompl::base::State* to the true StateType*
                auto stt = state->template as<ob::AtlasStateSpace::StateType>();
                new (&self) ob::AtlasChart(atlas.get(), stt);
            },
            nb::arg("atlas"), nb::arg("state"))
        // Destructor is implicit

        // Clear all polytope halfspaces
        .def("clear", &ob::AtlasChart::clear)

        // Accessors
        .def(
            "getOrigin", [](const ob::AtlasChart &self) -> const ob::State *
            { return static_cast<const ob::State *>(self.getOrigin()); }, nb::rv_policy::reference_internal)
        .def("getAmbientDimension", &ob::AtlasChart::getAmbientDimension)
        .def("getManifoldDimension", &ob::AtlasChart::getManifoldDimension)

        // Phi: map local coords u -> ambient
        .def(
            "phi",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &u)
            {
                Eigen::VectorXd out(self.getAmbientDimension());
                self.phi(u, out);
                return out;
            },
            nb::arg("u"))

        // Psi: project ambient u -> local coords
        .def(
            "psi",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &u)
            {
                Eigen::VectorXd out(self.getManifoldDimension());
                bool ok = self.psi(u, out);
                return std::make_pair(ok, out);
            },
            nb::arg("u"))

        // ψ⁻¹: inverse chart map
        .def(
            "psiInverse",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &x)
            {
                Eigen::VectorXd out(self.getAmbientDimension());
                self.psiInverse(x, out);
                return out;
            },
            nb::arg("x"))

        // inPolytope: test if local u lies inside the chart's polytope
        .def(
            "inPolytope",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &u) { return self.inPolytope(u, nullptr, nullptr); },
            nb::arg("u"))

        // borderCheck: update halfspaces to include v if it's near the boundary
        .def("borderCheck", &ob::AtlasChart::borderCheck, nb::arg("v"))

        // owningNeighbor: return pointer to adjacent chart owning x (or nullptr)
        .def("owningNeighbor", &ob::AtlasChart::owningNeighbor, nb::arg("x"), nb::rv_policy::reference_internal)

        // toPolygon: return the polytope as a list of boundary vertices
        .def(
            "toPolygon",
            [](const ob::AtlasChart &self)
            {
                std::vector<Eigen::VectorXd> verts;
                bool ok = self.toPolygon(verts);
                return std::make_pair(ok, verts);
            })

        // count of halfspaces (polytope facets)
        .def("getNeighborCount", &ob::AtlasChart::getNeighborCount)

        // Is this chart a frontier?
        .def("estimateIsFrontier", &ob::AtlasChart::estimateIsFrontier)

        // Static helper: construct halfspaces between two charts
        .def_static("generateHalfspace", &ob::AtlasChart::generateHalfspace, nb::arg("c1"), nb::arg("c2"));
}
