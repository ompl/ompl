#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/spaces/constraint/AtlasChart.h"
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesConstraint_AtlasChart(nb::module_ &m)
{
    // AtlasChart
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
            nb::arg("atlas"), nb::arg("state"), "Construct an AtlasChart(atlas: AtlasStateSpace, state: State)")
        // Destructor is implicit

        // Clear all polytope halfspaces
        .def("clear", &ob::AtlasChart::clear, "Remove all halfspaces (i.e. reset the chart region).")

        // Accessors
        .def(
            "getOrigin", [](const ob::AtlasChart &self) -> const ob::State *
            { return static_cast<const ob::State *>(self.getOrigin()); }, nb::rv_policy::reference_internal,
            "Return the chart center as an ompl.base.State")
        .def("getAmbientDimension", &ob::AtlasChart::getAmbientDimension, "Dimension of the ambient space.")
        .def("getManifoldDimension", &ob::AtlasChart::getManifoldDimension,
             "Intrinsic (manifold) dimension of this chart.")

        // Phi: map local coords u -> ambient
        .def(
            "phi",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &u)
            {
                Eigen::VectorXd out(self.getAmbientDimension());
                self.phi(u, out);
                return out;
            },
            nb::arg("u"), "Chart-to-ambient mapping φ(u).")

        // Psi: project ambient u -> local coords
        .def(
            "psi",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &u)
            {
                Eigen::VectorXd out(self.getManifoldDimension());
                bool ok = self.psi(u, out);
                return std::make_pair(ok, out);
            },
            nb::arg("u"), "Ambient-to-chart projection ψ(x) → (success, local coords).")

        // ψ⁻¹: inverse chart map
        .def(
            "psiInverse",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &x)
            {
                Eigen::VectorXd out(self.getAmbientDimension());
                self.psiInverse(x, out);
                return out;
            },
            nb::arg("x"), "Inverse of ψ (manifold coords → ambient).")

        // inPolytope: test if local u lies inside the chart's polytope
        .def(
            "inPolytope",
            [](const ob::AtlasChart &self, const Eigen::VectorXd &u) { return self.inPolytope(u, nullptr, nullptr); },
            nb::arg("u"), "Return True if u is inside the chart polytope.")

        // borderCheck: update halfspaces to include v if it's near the boundary
        .def("borderCheck", &ob::AtlasChart::borderCheck, nb::arg("v"),
             "Expand the polytope if v lies near its boundary.")

        // owningNeighbor: return pointer to adjacent chart owning x (or nullptr)
        .def("owningNeighbor", &ob::AtlasChart::owningNeighbor, nb::arg("x"), nb::rv_policy::reference_internal,
             "Return the neighboring AtlasChart that owns x (or None).")

        // toPolygon: return the polytope as a list of boundary vertices
        .def(
            "toPolygon",
            [](const ob::AtlasChart &self)
            {
                std::vector<Eigen::VectorXd> verts;
                bool ok = self.toPolygon(verts);
                return std::make_pair(ok, verts);
            },
            "Convert the chart's polytope to a list of vertices (u coordinates).")

        // count of halfspaces (polytope facets)
        .def("getNeighborCount", &ob::AtlasChart::getNeighborCount, "Number of neighboring charts / polytope facets.")

        // Is this chart a frontier?
        .def("estimateIsFrontier", &ob::AtlasChart::estimateIsFrontier,
             "Return True if this chart lies on the boundary of the manifold.")

        // Static helper: construct halfspaces between two charts
        .def_static("generateHalfspace", &ob::AtlasChart::generateHalfspace, nb::arg("c1"), nb::arg("c2"),
                    "Compute and link the Halfspace separating c1 and c2.");
}
