#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/export.hpp>

#include "ompl/control/PlannerData.h"
#include "ompl/control/PlannerDataStorage.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

// OMPL leaves this export to the application; without it every store() fails on the derived edge type.
BOOST_CLASS_EXPORT(ompl::control::PlannerDataEdgeControl)

void ompl::binding::control::init_PlannerDataStorage(nb::module_ &m)
{
    // Upstream static_casts its base::PlannerData argument to control::PlannerData, so only accept that type.
    nb::class_<oc::PlannerDataStorage, ob::PlannerDataStorage>(m, "PlannerDataStorage")
        .def(nb::init<>())
        .def(
            "store", [](oc::PlannerDataStorage &storage, const oc::PlannerData &pd, const char *filename)
            { return storage.store(pd, filename); }, nb::arg("pd"), nb::arg("filename"))
        .def(
            "load", [](oc::PlannerDataStorage &storage, const char *filename, oc::PlannerData &pd)
            { return storage.load(filename, pd); }, nb::arg("filename"), nb::arg("pd"));
}
