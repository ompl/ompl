#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>
#include "ompl/base/spaces/RealVectorStateProjections.h"
#include "../init.hh"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_RealVectorStateProjections(nb::module_& m)
{
    nb::class_<ompl::base::RealVectorLinearProjectionEvaluator, ompl::base::ProjectionEvaluator>(m, "RealVectorLinearProjectionEvaluator")
        .def(nb::init<const ompl::base::StateSpace*, const std::vector<double>&, const ompl::base::ProjectionMatrix::Matrix&>())
        .def(nb::init<const ompl::base::StateSpacePtr&, const std::vector<double>&, const ompl::base::ProjectionMatrix::Matrix&>())
        .def(nb::init<const ompl::base::StateSpace*, const ompl::base::ProjectionMatrix::Matrix&>())
        .def(nb::init<const ompl::base::StateSpacePtr&, const ompl::base::ProjectionMatrix::Matrix&>())
        .def("getDimension", &ompl::base::RealVectorLinearProjectionEvaluator::getDimension)
        .def("project", &ompl::base::RealVectorLinearProjectionEvaluator::project);
      
    nb::class_<ompl::base::RealVectorRandomLinearProjectionEvaluator, ompl::base::RealVectorLinearProjectionEvaluator>(m, "RealVectorRandomLinearProjectionEvaluator");

    nb::class_<ompl::base::RealVectorOrthogonalProjectionEvaluator, ompl::base::ProjectionEvaluator>(m, "RealVectorOrthogonalProjectionEvaluator");

    nb::class_<ompl::base::RealVectorIdentityProjectionEvaluator, ompl::base::ProjectionEvaluator>(m, "RealVectorIdentityProjectionEvaluator");

}
