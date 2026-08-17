#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/multilevel/datastructures/ProjectionFactory.h"
#include "ompl/multilevel/datastructures/Projection.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::init_ProjectionFactory(nb::module_ &m)
{
    nb::class_<ml::ProjectionFactory>(m, "ProjectionFactory")
        .def(nb::init<>())
        .def("makeProjection",
             nb::overload_cast<const ob::SpaceInformationPtr &, const ob::SpaceInformationPtr &>(
                 &ml::ProjectionFactory::makeProjection),
             nb::arg("bundle"), nb::arg("base"))
        .def("makeProjection",
             nb::overload_cast<const ob::SpaceInformationPtr &>(&ml::ProjectionFactory::makeProjection),
             nb::arg("bundle"));
}
