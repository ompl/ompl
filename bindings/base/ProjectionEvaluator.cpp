#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>
#include <nanobind/eigen/dense.h>
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpace.h"
#include "init.hh"
#include <sstream>

namespace nb = nanobind;

void ompl::binding::base::init_ProjectionEvaluator(nb::module_ &m)
{

    // Bind ProjectionMatrix
    // TODO [ompl::base::ProjectionMatrix][TEST]
    nb::class_<ompl::base::ProjectionMatrix>(m, "ProjectionMatrix")
        .def(nb::init<>())
        // return the class variable mat
        .def_rw("mat", &ompl::base::ProjectionMatrix::mat)
        .def_static("ComputeRandom", 
            nb::overload_cast<unsigned int, unsigned int, const std::vector<double>&>(&ompl::base::ProjectionMatrix::ComputeRandom), nb::arg("from"), nb::arg("to"), nb::arg("scale"))
        .def_static("ComputeRandom", 
            nb::overload_cast<unsigned int, unsigned int>(&ompl::base::ProjectionMatrix::ComputeRandom), nb::arg("from"), nb::arg("to"))
        .def("computeRandom", 
            nb::overload_cast<unsigned int, unsigned int, const std::vector<double>&>(&ompl::base::ProjectionMatrix::computeRandom), nb::arg("from"), nb::arg("to"), nb::arg("scale"))
        .def("computeRandom", 
            nb::overload_cast<unsigned int, unsigned int>(&ompl::base::ProjectionMatrix::computeRandom), nb::arg("from"), nb::arg("to"))
        .def("project", &ompl::base::ProjectionMatrix::project, nb::arg("from"), nb::arg("to"))
        .def("print", [](const ompl::base::ProjectionMatrix &pm) { pm.print(std::cout); });

    // Bind ProjectionEvaluator
    // TODO [ompl::base::ProjectionEvaluator][TEST]
    struct PyProjectionEvaluator : ompl::base::ProjectionEvaluator
    {
        NB_TRAMPOLINE(ompl::base::ProjectionEvaluator, 3);
        unsigned int getDimension() const override
        {
            NB_OVERRIDE_PURE(getDimension);
        }
        // virtual void 	project (const State *state, Eigen::Ref< Eigen::VectorXd > projection) const =0
        void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            NB_OVERRIDE_PURE(project, state, projection);
        }
        // virtual void 	setCellSizes (const std::vector< double > &cellSizes)
        void setCellSizes(const std::vector<double> &cellSizes) override
        {
            NB_OVERRIDE(setCellSizes, cellSizes);
        }
    };
    nb::class_<ompl::base::ProjectionEvaluator, PyProjectionEvaluator /* <-- trampoline */>(m, "ProjectionEvaluator")
        .def(nb::init<const ompl::base::StateSpacePtr &>(), nb::arg("space"))
        .def("setCellSizes", nb::overload_cast<unsigned int, double>(&ompl::base::ProjectionEvaluator::setCellSizes), nb::arg("dim"), nb::arg("cellSize"))
        .def("mulCellSizes", &ompl::base::ProjectionEvaluator::mulCellSizes, "Multiply the cell sizes by a factor", nb::arg("factor"))
        .def("userConfigured", &ompl::base::ProjectionEvaluator::userConfigured)
        .def("getCellSizes", nb::overload_cast<>(&ompl::base::ProjectionEvaluator::getCellSizes, nb::const_))
        .def("getCellSizes", nb::overload_cast<unsigned int>(&ompl::base::ProjectionEvaluator::getCellSizes, nb::const_), nb::arg("dim"))
        .def("checkCellSizes", &ompl::base::ProjectionEvaluator::checkCellSizes)
        .def("inferCellSizes", &ompl::base::ProjectionEvaluator::inferCellSizes)
        .def("checkBounds", &ompl::base::ProjectionEvaluator::checkBounds)
        .def("hasBounds", &ompl::base::ProjectionEvaluator::hasBounds)
        .def("setBounds", &ompl::base::ProjectionEvaluator::setBounds, nb::arg("bounds"))
        .def("getBounds", &ompl::base::ProjectionEvaluator::getBounds)
        .def("inferBounds", &ompl::base::ProjectionEvaluator::inferBounds)
        .def("computeCoordinates", 
             nb::overload_cast<const Eigen::Ref<Eigen::VectorXd>&, Eigen::Ref<Eigen::VectorXi>>(&ompl::base::ProjectionEvaluator::computeCoordinates, nb::const_), nb::arg("projection"), nb::arg("coord"))
        .def("computeCoordinates", 
             nb::overload_cast<const ompl::base::State*, Eigen::Ref<Eigen::VectorXi>>(&ompl::base::ProjectionEvaluator::computeCoordinates, nb::const_), nb::arg("state"), nb::arg("coord"))
        .def("params", 
             nb::overload_cast<>(&ompl::base::ProjectionEvaluator::params))
        .def("params", 
             nb::overload_cast<>(&ompl::base::ProjectionEvaluator::params, nb::const_));

    // Bind SubspaceProjectionEvaluator
    // TODO [ompl::base::SubspaceProjectionEvaluator][TEST]
    nb::class_<ompl::base::SubspaceProjectionEvaluator, ompl::base::ProjectionEvaluator>(m, "SubspaceProjectionEvaluator")
        .def(nb::init<const ompl::base::StateSpace*, unsigned int, ompl::base::ProjectionEvaluatorPtr>(), nb::arg("space"), nb::arg("index"), nb::arg("projToUse") = ompl::base::ProjectionEvaluatorPtr())
        .def("setup", &ompl::base::SubspaceProjectionEvaluator::setup)
        .def("getDimension", &ompl::base::SubspaceProjectionEvaluator::getDimension)
        .def("project", &ompl::base::SubspaceProjectionEvaluator::project, nb::arg("state"), nb::arg("projection"));
}