#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <sstream>

#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "ompl/multilevel/datastructures/Projection.h"
#include "ompl/multilevel/datastructures/projections/FiberedProjection.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::init_Projection(nb::module_ &m)
{
    nb::class_<ml::Projection>(m, "Projection")
        .def("project", &ml::Projection::project, nb::arg("xBundle"), nb::arg("xBase"))
        .def("lift", &ml::Projection::lift, nb::arg("xBase"), nb::arg("xBundle"))
        .def("isFibered", &ml::Projection::isFibered)
        .def("isAdmissible", &ml::Projection::isAdmissible)
        .def("getCoDimension", &ml::Projection::getCoDimension)
        .def("getDimension", &ml::Projection::getDimension)
        .def("getBaseDimension", &ml::Projection::getBaseDimension)
        .def("getBundle", &ml::Projection::getBundle)
        .def("getBase", &ml::Projection::getBase)
        .def("getType", &ml::Projection::getType)
        .def("setType", &ml::Projection::setType, nb::arg("type"))
        .def("getTypeAsString", &ml::Projection::getTypeAsString)
        .def("getBundleTypeAsString", &ml::Projection::getBundleTypeAsString)
        .def("getBaseTypeAsString", &ml::Projection::getBaseTypeAsString)
        .def("__repr__",
             [](const ml::Projection &p)
             {
                 std::ostringstream oss;
                 p.print(oss);
                 return oss.str();
             });

    nb::class_<ml::FiberedProjection, ml::Projection>(m, "FiberedProjection")
        // Both overloads, since binding the fiber one alone would hide Projection.lift.
        .def("lift", nb::overload_cast<const ob::State *, ob::State *>(&ml::FiberedProjection::lift, nb::const_),
             nb::arg("xBase"), nb::arg("xBundle"))
        .def("lift",
             nb::overload_cast<const ob::State *, const ob::State *, ob::State *>(&ml::FiberedProjection::lift,
                                                                                  nb::const_),
             nb::arg("xBase"), nb::arg("xFiber"), nb::arg("xBundle"))
        .def("projectFiber", &ml::FiberedProjection::projectFiber, nb::arg("xBundle"), nb::arg("xFiber"))
        .def("getFiberSpace", &ml::FiberedProjection::getFiberSpace)
        .def("getFiberSamplerPtr", &ml::FiberedProjection::getFiberSamplerPtr)
        .def("getFiberDimension", &ml::FiberedProjection::getFiberDimension)
        .def("getFiberTypeAsString", &ml::FiberedProjection::getFiberTypeAsString);
}
