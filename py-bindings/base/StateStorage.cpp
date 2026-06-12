#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/base/StateStorage.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_StateStorage(nb::module_ &m)
{
    nb::class_<ob::StateStorage>(m, "StateStorage")
        .def(nb::init<ob::StateSpacePtr>(), nb::arg("space"))
        .def("getStateSpace", &ob::StateStorage::getStateSpace)
        .def("load", nb::overload_cast<const char *>(&ob::StateStorage::load), nb::arg("filename"))
        .def("store", nb::overload_cast<const char *>(&ob::StateStorage::store), nb::arg("filename"))
        .def("addState", &ob::StateStorage::addState, nb::arg("state"))
        .def("generateSamples", &ob::StateStorage::generateSamples, nb::arg("count"))
        .def("clear", &ob::StateStorage::clear)
        .def("size", &ob::StateStorage::size)
        .def("getStates", &ob::StateStorage::getStates, nb::rv_policy::reference_internal)
        .def("getState", nb::overload_cast<unsigned int>(&ob::StateStorage::getState, nb::const_), nb::arg("index"),
             nb::rv_policy::reference)
        .def("hasMetadata", &ob::StateStorage::hasMetadata)
        .def(
            "sort",
            [](ob::StateStorage &storage, std::function<bool(const ob::State *, const ob::State *)> op)
            {
                nb::gil_scoped_acquire gil;
                storage.sort(op);
            },
            nb::arg("op"))
        .def("getStateSamplerAllocator", &ob::StateStorage::getStateSamplerAllocator)
        .def("getStateSamplerAllocatorRangeUntil", &ob::StateStorage::getStateSamplerAllocatorRangeUntil,
             nb::arg("until"))
        .def("getStateSamplerAllocatorRangeAfter", &ob::StateStorage::getStateSamplerAllocatorRangeAfter,
             nb::arg("after"))
        .def("getStateSamplerAllocatorRange", &ob::StateStorage::getStateSamplerAllocatorRange, nb::arg("from"),
             nb::arg("to"))
        .def(
            "print",
            [](const ob::StateStorage &storage)
            {
                std::ostringstream oss;
                storage.print(oss);
                return oss.str();
            });

    nb::class_<ob::GraphStateStorage, ob::StateStorage>(m, "GraphStateStorage")
        .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("space"))
        .def(
            "addState",
            nb::overload_cast<const ob::State *, const std::vector<std::size_t> &>(&ob::GraphStateStorage::addState),
            nb::arg("state"), nb::arg("metadata"))
        .def("getMetadata", nb::overload_cast<unsigned int>(&ob::GraphStateStorage::getMetadata, nb::const_),
             nb::arg("index"), nb::rv_policy::reference_internal);
}
