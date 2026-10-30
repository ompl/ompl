#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <memory>
#include <sstream>
#include <utility>

#include "ompl/base/StateSampler.h"
#include "ompl/base/StateStorage.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    std::shared_ptr<ob::State> ownedClone(const ob::StateStorage &storage, const ob::State *state)
    {
        const ob::StateSpacePtr &space = storage.getStateSpace();
        return {space->cloneState(state), [space](ob::State *s) { space->freeState(s); }};
    }

    void checkIndex(const ob::StateStorage &storage, unsigned int index)
    {
        if (index >= storage.size())
            throw nb::index_error("state index out of range");
    }

    /// OMPL's allocators, and the samplers they make, point into the storage without owning it.
    ob::StateSamplerAllocator owningAllocator(std::shared_ptr<ob::StateStorage> storage,
                                              ob::StateSamplerAllocator allocator)
    {
        return [storage = std::move(storage), allocator = std::move(allocator)](const ob::StateSpace *space)
        {
            auto owner = std::make_shared<std::pair<std::shared_ptr<ob::StateStorage>, ob::StateSamplerPtr>>(
                storage, allocator(space));
            return ob::StateSamplerPtr(owner, owner->second.get());
        };
    }
}  // namespace

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
        // Copies: clear() and load() free the stored states, which would leave views dangling.
        .def("getStates",
             [](const ob::StateStorage &storage)
             {
                 std::vector<std::shared_ptr<ob::State>> states;
                 states.reserve(storage.size());
                 for (const ob::State *state : storage.getStates())
                     states.push_back(ownedClone(storage, state));
                 return states;
             })
        .def(
            "getState",
            [](const ob::StateStorage &storage, unsigned int index)
            {
                checkIndex(storage, index);
                return ownedClone(storage, storage.getState(index));
            },
            nb::arg("index"))
        .def("hasMetadata", &ob::StateStorage::hasMetadata)
        .def("sort", &ob::StateStorage::sort, nb::arg("op"))
        .def("getStateSamplerAllocator", [](const std::shared_ptr<ob::StateStorage> &storage)
             { return owningAllocator(storage, storage->getStateSamplerAllocator()); })
        .def(
            "getStateSamplerAllocatorRangeUntil",
            [](const std::shared_ptr<ob::StateStorage> &storage, std::size_t until)
            { return owningAllocator(storage, storage->getStateSamplerAllocatorRangeUntil(until)); }, nb::arg("until"))
        .def(
            "getStateSamplerAllocatorRangeAfter",
            [](const std::shared_ptr<ob::StateStorage> &storage, std::size_t after)
            { return owningAllocator(storage, storage->getStateSamplerAllocatorRangeAfter(after)); }, nb::arg("after"))
        .def(
            "getStateSamplerAllocatorRange",
            [](const std::shared_ptr<ob::StateStorage> &storage, std::size_t from, std::size_t to)
            { return owningAllocator(storage, storage->getStateSamplerAllocatorRange(from, to)); }, nb::arg("from"),
            nb::arg("to"))
        .def("__repr__",
             [](const ob::StateStorage &storage)
             {
                 std::ostringstream oss;
                 storage.print(oss);
                 return oss.str();
             });

    nb::class_<ob::GraphStateStorage, ob::StateStorage>(m, "GraphStateStorage")
        .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("space"))
        .def("addState", nb::overload_cast<const ob::State *>(&ob::GraphStateStorage::addState), nb::arg("state"))
        .def("addState",
             nb::overload_cast<const ob::State *, const std::vector<std::size_t> &>(&ob::GraphStateStorage::addState),
             nb::arg("state"), nb::arg("metadata"))
        .def(
            "getMetadata",
            [](const ob::GraphStateStorage &storage, unsigned int index)
            {
                checkIndex(storage, index);
                return storage.getMetadata(index);
            },
            nb::arg("index"));
}
