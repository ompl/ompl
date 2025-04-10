#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/eigen/dense.h>
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpace.h"
#include "init.hh"
#include <sstream>

namespace nb = nanobind;

void ompl::binding::base::init_ProjectionEvaluator(nb::module_ &m)
{
    // Bind ProjectionMatrix
    nb::class_<ompl::base::ProjectionMatrix>(m, "ProjectionMatrix",
        "A projection matrix allows multiplication of real vectors by a specified matrix.")
        .def(nb::init<>())
        // return the class variable mat
        .def_rw("mat", &ompl::base::ProjectionMatrix::mat)
        .def_static("ComputeRandom", 
            nb::overload_cast<unsigned int, unsigned int, const std::vector<double>&>(&ompl::base::ProjectionMatrix::ComputeRandom),
            "Compute a random projection matrix with scaling", nb::arg("from"), nb::arg("to"), nb::arg("scale"))
        .def_static("ComputeRandom", 
            nb::overload_cast<unsigned int, unsigned int>(&ompl::base::ProjectionMatrix::ComputeRandom),
            "Compute a random projection matrix", nb::arg("from"), nb::arg("to"))
        .def("computeRandom", 
            nb::overload_cast<unsigned int, unsigned int, const std::vector<double>&>(&ompl::base::ProjectionMatrix::computeRandom),
            "Compute a random projection matrix", nb::arg("from"), nb::arg("to"), nb::arg("scale"))
        .def("computeRandom", 
            nb::overload_cast<unsigned int, unsigned int>(&ompl::base::ProjectionMatrix::computeRandom),
            "Compute a random projection matrix", nb::arg("from"), nb::arg("to"))
        .def("project", &ompl::base::ProjectionMatrix::project, "Project the vector through the matrix", nb::arg("from"), nb::arg("to"))
        .def("print", [](const ompl::base::ProjectionMatrix &pm) { pm.print(std::cout); }, "Print the projection matrix");

    // Bind ProjectionEvaluator
    nb::class_<ompl::base::ProjectionEvaluator>(m, "ProjectionEvaluator",
        "Abstract definition for a class computing projections to R^n.")
        .def("setCellSizes", nb::overload_cast<unsigned int, double>(&ompl::base::ProjectionEvaluator::setCellSizes), 
             "Set the cell size for a particular dimension", nb::arg("dim"), nb::arg("cellSize"))
        .def("mulCellSizes", &ompl::base::ProjectionEvaluator::mulCellSizes, "Multiply the cell sizes by a factor", nb::arg("factor"))
        .def("userConfigured", &ompl::base::ProjectionEvaluator::userConfigured, "Check if user configuration was done")
        .def("getCellSizes", nb::overload_cast<>(&ompl::base::ProjectionEvaluator::getCellSizes, nb::const_), "Get the size of a grid cell")
        .def("getCellSizes", nb::overload_cast<unsigned int>(&ompl::base::ProjectionEvaluator::getCellSizes, nb::const_), "Get the size of a particular dimension", nb::arg("dim"))
        .def("checkCellSizes", &ompl::base::ProjectionEvaluator::checkCellSizes, "Check if cell dimensions match projection dimension")
        .def("inferCellSizes", &ompl::base::ProjectionEvaluator::inferCellSizes, "Infer cell sizes")
        .def("checkBounds", &ompl::base::ProjectionEvaluator::checkBounds, "Check if the projection dimension matches the dimension of the bounds")
        .def("hasBounds", &ompl::base::ProjectionEvaluator::hasBounds, "Check if bounds were specified")
        .def("setBounds", &ompl::base::ProjectionEvaluator::setBounds, "Set bounds on the projection", nb::arg("bounds"))
        .def("getBounds", &ompl::base::ProjectionEvaluator::getBounds, "Get the bounds for this projection")
        .def("inferBounds", &ompl::base::ProjectionEvaluator::inferBounds, "Compute an approximation of the bounds for this projection")
        .def("computeCoordinates", 
             nb::overload_cast<const Eigen::Ref<Eigen::VectorXd>&, Eigen::Ref<Eigen::VectorXi>>(&ompl::base::ProjectionEvaluator::computeCoordinates, nb::const_),
             "Compute integer coordinates for a projection", nb::arg("projection"), nb::arg("coord"))
        .def("computeCoordinates", 
             nb::overload_cast<const ompl::base::State*, Eigen::Ref<Eigen::VectorXi>>(&ompl::base::ProjectionEvaluator::computeCoordinates, nb::const_),
             "Compute integer coordinates for a state", nb::arg("state"), nb::arg("coord"))
        .def("params", 
             nb::overload_cast<>(&ompl::base::ProjectionEvaluator::params),
             "Get the parameters for this projection")
        .def("params", 
             nb::overload_cast<>(&ompl::base::ProjectionEvaluator::params, nb::const_),
             "Get the parameters for this projection");

    // Bind SubspaceProjectionEvaluator
    // TODO: Not tested 
    nb::class_<ompl::base::SubspaceProjectionEvaluator, ompl::base::ProjectionEvaluator>(m, "SubspaceProjectionEvaluator",
        "If the projection for a CompoundStateSpace is supposed to be the same as one of its included subspaces.")
        .def(nb::init<const ompl::base::StateSpace*, unsigned int, ompl::base::ProjectionEvaluatorPtr>(), 
             "Constructor for subspace projection evaluator", nb::arg("space"), nb::arg("index"), nb::arg("projToUse") = ompl::base::ProjectionEvaluatorPtr())
        .def("setup", &ompl::base::SubspaceProjectionEvaluator::setup, "Setup the projection evaluator")
        .def("getDimension", &ompl::base::SubspaceProjectionEvaluator::getDimension, "Get the dimension of the projection")
        .def("project", &ompl::base::SubspaceProjectionEvaluator::project, "Project the state to a subspace", nb::arg("state"), nb::arg("projection"));
}