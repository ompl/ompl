#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "ompl/base/spaces/RealVectorStateProjections.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_RealVectorStateProjections(nb::module_ &m)
{
     // RealVectorLinearProjectionEvaluator
     // TODO [ob::RealVectorLinearProjectionEvaluator][TEST]
     nb::class_<ob::RealVectorLinearProjectionEvaluator, ob::ProjectionEvaluator>(
          m, "RealVectorLinearProjectionEvaluator")
          .def(nb::init<const ob::StateSpace *, const std::vector<double> &, const ob::ProjectionMatrix::Matrix &>(),
               nb::arg("space"), nb::arg("cellSizes"), nb::arg("projection"))
          .def(nb::init<const ob::StateSpacePtr &, const std::vector<double> &, const ob::ProjectionMatrix::Matrix &>(),
               nb::arg("space"), nb::arg("cellSizes"), nb::arg("projection"))
          .def(nb::init<const ob::StateSpace *, const ob::ProjectionMatrix::Matrix &>(),
               nb::arg("space"), nb::arg("projection"))
          .def(nb::init<const ob::StateSpacePtr &, const ob::ProjectionMatrix::Matrix &>(),
               nb::arg("space"), nb::arg("projection"))
          .def("getDimension", &ob::RealVectorLinearProjectionEvaluator::getDimension)
          .def("project", &ob::RealVectorLinearProjectionEvaluator::project);

     // RealVectorRandomLinearProjectionEvaluator
     // TODO [ob::RealVectorRandomLinearProjectionEvaluator][TEST]
     nb::class_<ob::RealVectorRandomLinearProjectionEvaluator, ob::RealVectorLinearProjectionEvaluator>(
          m, "RealVectorRandomLinearProjectionEvaluator")
          .def(nb::init<const ob::StateSpace *, const std::vector<double> &>(),
               nb::arg("space"), nb::arg("cellSizes"))
          .def(nb::init<const ob::StateSpacePtr &, const std::vector<double> &>(),
               nb::arg("space"), nb::arg("cellSizes"))
          .def(nb::init<const ob::StateSpace *, unsigned int>(),
               nb::arg("space"), nb::arg("dim"))
          .def(nb::init<const ob::StateSpacePtr &, unsigned int>(),
               nb::arg("space"), nb::arg("dim"));

     // RealVectorOrthogonalProjectionEvaluator
     // TODO [ob::RealVectorOrthogonalProjectionEvaluator][TEST]
     nb::class_<ob::RealVectorOrthogonalProjectionEvaluator, ob::ProjectionEvaluator>(
          m, "RealVectorOrthogonalProjectionEvaluator")
          .def(nb::init<const ob::StateSpace *, const std::vector<double> &, std::vector<unsigned int>>(),
               nb::arg("space"), nb::arg("cellSizes"), nb::arg("components"))
          .def(nb::init<const ob::StateSpacePtr &, const std::vector<double> &, std::vector<unsigned int>>(),
               nb::arg("space"), nb::arg("cellSizes"), nb::arg("components"))
          .def(nb::init<const ob::StateSpace *, std::vector<unsigned int>>(),
               nb::arg("space"), nb::arg("components"))
          .def(nb::init<const ob::StateSpacePtr &, std::vector<unsigned int>>(),
               nb::arg("space"), nb::arg("components"))
          .def("getDimension", &ob::RealVectorOrthogonalProjectionEvaluator::getDimension)
          .def("defaultCellSizes", &ob::RealVectorOrthogonalProjectionEvaluator::defaultCellSizes)
          .def("project", &ob::RealVectorOrthogonalProjectionEvaluator::project);

     // RealVectorIdentityProjectionEvaluator
     // TODO [ob::RealVectorIdentityProjectionEvaluator][TEST]
     nb::class_<ob::RealVectorIdentityProjectionEvaluator, ob::ProjectionEvaluator>(
          m, "RealVectorIdentityProjectionEvaluator")
          .def(nb::init<const ob::StateSpace *, const std::vector<double> &>(),
               nb::arg("space"), nb::arg("cellSizes"))
          .def(nb::init<const ob::StateSpacePtr &, const std::vector<double> &>(),
               nb::arg("space"), nb::arg("cellSizes"))
          .def(nb::init<const ob::StateSpace *>(), nb::arg("space"))
          .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("space"))
          .def("getDimension", &ob::RealVectorIdentityProjectionEvaluator::getDimension)
          .def("defaultCellSizes", &ob::RealVectorIdentityProjectionEvaluator::defaultCellSizes)
          .def("setup", &ob::RealVectorIdentityProjectionEvaluator::setup)
          .def("project", &ob::RealVectorIdentityProjectionEvaluator::project);
}
