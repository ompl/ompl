#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/samplers/ConditionalStateSampler.h"
#include "ompl/base/spaces/SpaceTimeStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ValidStateSampler.h"
#include "base/samplers/OwnerRef.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace gc = ompl::binding::gc;

namespace
{
    using Motion = ob::ConditionalStateSampler::Motion;

    // OMPL keeps raw Motion pointers; the Python motions they point into are held next to them.
    struct PyConditionalStateSampler : ob::ConditionalStateSampler
    {
        Motion *startMotionPtr{nullptr};
        std::vector<Motion *> goalMotionsStorage;
        std::vector<Motion *> newBatchGoalMotionsStorage;
        bool sampleOldBatchFlag{false};

        nb::object startMotion;
        nb::list goalMotions;
        nb::list newBatchGoalMotions;

        explicit PyConditionalStateSampler(const ob::SpaceInformation *si)
          : ob::ConditionalStateSampler(si, startMotionPtr, goalMotionsStorage, newBatchGoalMotionsStorage,
                                        sampleOldBatchFlag)
        {
        }

        bool sample(ob::State *state) override
        {
            checkSampleable();
            return ob::ConditionalStateSampler::sample(state);
        }

        void checkSampleable() const
        {
            if (dynamic_cast<const ob::SpaceTimeStateSpace *>(si_->getStateSpace().get()) == nullptr)
                throw nb::type_error("ConditionalStateSampler.sample(): the state space must be a SpaceTimeStateSpace");
            if (startMotionPtr == nullptr || startMotionPtr->state == nullptr)
                throw nb::value_error("ConditionalStateSampler.sample(): set a start motion with a state first");
            for (const auto *goals : {&goalMotionsStorage, &newBatchGoalMotionsStorage})
                for (const Motion *goal : *goals)
                    if (goal->state == nullptr)
                        throw nb::value_error("ConditionalStateSampler.sample(): every goal motion needs a state");
        }

        void clearMotions()
        {
            startMotionPtr = nullptr;
            goalMotionsStorage.clear();
            newBatchGoalMotionsStorage.clear();
            startMotion.reset();
            goalMotions.reset();
            newBatchGoalMotions.reset();
        }
    };

    PyConditionalStateSampler &storage(ob::ConditionalStateSampler &self)
    {
        auto *py = dynamic_cast<PyConditionalStateSampler *>(&self);
        if (py == nullptr)
            throw nb::type_error("this ConditionalStateSampler was not created from Python");
        return *py;
    }

    void assignMotions(std::vector<Motion *> &target, nb::list &keep,
                       const std::vector<nb::pointer_and_handle<Motion>> &motions)
    {
        nb::list held;
        std::vector<Motion *> pointers;
        for (const auto &motion : motions)
        {
            held.append(motion.h);
            pointers.push_back(motion.p);
        }
        target = std::move(pointers);
        keep = std::move(held);
    }

    PyConditionalStateSampler *fromInstance(PyObject *self)
    {
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return nullptr;
        return dynamic_cast<PyConditionalStateSampler *>(nb::inst_ptr<ob::ConditionalStateSampler>(self));
    }

    int conditionalTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        PyConditionalStateSampler *sampler = fromInstance(self);
        if (sampler == nullptr)
            return 0;
        for (nb::handle member : {nb::handle(sampler->startMotion), nb::handle(sampler->goalMotions),
                                  nb::handle(sampler->newBatchGoalMotions)})
            if (member.is_valid())
                Py_VISIT(member.ptr());
        if (const auto *ref = dynamic_cast<const ompl::binding::OwnerRef *>(sampler))
            return gc::visitHeld(ref->owner, visit, arg);
        return 0;
    }

    // Only the motions are dropped; the owner must outlive the sampler (see OwnerRef.h).
    int conditionalClear(PyObject *self)
    {
        if (PyConditionalStateSampler *sampler = fromInstance(self))
            sampler->clearMotions();
        return 0;
    }

    PyType_Slot conditionalSlots[] = {{Py_tp_traverse, (void *)conditionalTraverse},
                                      {Py_tp_clear, (void *)conditionalClear},
                                      {0, 0}};
}  // namespace

void ompl::binding::base::initSamplers_ConditionalStateSampler(nb::module_ &m)
{
    // A motion owns the state it is given through __dict__, so the raw pointer OMPL reads stays valid.
    nb::class_<Motion>(m, "ConditionalStateSamplerMotion", nb::dynamic_attr())
        .def(nb::init<>())
        .def(
            "__init__",
            [](nb::pointer_and_handle<Motion> self, const ob::SpaceInformationPtr &si)
            {
                new (self.p) Motion();
                std::shared_ptr<ob::State> state(si->allocState(),
                                                 [space = si->getStateSpace()](ob::State *s) { space->freeState(s); });
                self.p->state = state.get();
                nb::setattr(self.h, "_state", nb::cast(std::move(state)));
            },
            nb::arg("si"))
        .def_rw("root", &Motion::root)
        .def_prop_rw(
            "state", [](const Motion &motion) { return motion.state; },
            [](nb::pointer_and_handle<Motion> self, nb::pointer_and_handle<ob::State> state)
            {
                self.p->state = state.p;
                nb::setattr(self.h, "_state", state.h);
            },
            nb::rv_policy::reference)
        .def_rw("parent", &Motion::parent)
        .def_rw("connectionPoint", &Motion::connectionPoint)
        .def_rw("numConnections", &Motion::numConnections);

    nb::class_<ob::ConditionalStateSampler, ob::ValidStateSampler, WithOwner<PyConditionalStateSampler>>(
        m, "ConditionalStateSampler", nb::type_slots(conditionalSlots))
        .def("__init__",
             initWithOwnerAs<ob::ConditionalStateSampler, PyConditionalStateSampler, ob::SpaceInformation>(),
             nb::arg("si"))
        .def(
            "startMotionPtr", [](ob::ConditionalStateSampler &self) { return storage(self).startMotionPtr; },
            nb::rv_policy::reference)
        .def(
            "setStartMotionPtr",
            [](ob::ConditionalStateSampler &self, nb::pointer_and_handle<Motion> motion)
            {
                PyConditionalStateSampler &py = storage(self);
                py.startMotionPtr = motion.p;
                py.startMotion = nb::borrow(motion.h);
            },
            nb::arg("motion"))
        .def(
            "goalMotions", [](ob::ConditionalStateSampler &self) { return storage(self).goalMotionsStorage; },
            nb::rv_policy::reference)
        .def(
            "setGoalMotions",
            [](ob::ConditionalStateSampler &self, const std::vector<nb::pointer_and_handle<Motion>> &motions)
            {
                PyConditionalStateSampler &py = storage(self);
                assignMotions(py.goalMotionsStorage, py.goalMotions, motions);
            },
            nb::arg("motions"))
        .def(
            "newBatchGoalMotions", [](ob::ConditionalStateSampler &self)
            { return storage(self).newBatchGoalMotionsStorage; }, nb::rv_policy::reference)
        .def(
            "setNewBatchGoalMotions",
            [](ob::ConditionalStateSampler &self, const std::vector<nb::pointer_and_handle<Motion>> &motions)
            {
                PyConditionalStateSampler &py = storage(self);
                assignMotions(py.newBatchGoalMotionsStorage, py.newBatchGoalMotions, motions);
            },
            nb::arg("motions"))
        .def("sampleOldBatchFlag", [](ob::ConditionalStateSampler &self) { return storage(self).sampleOldBatchFlag; })
        .def(
            "setSampleOldBatchFlag", [](ob::ConditionalStateSampler &self, bool value)
            { storage(self).sampleOldBatchFlag = value; }, nb::arg("value"))
        .def("sample", &ob::ConditionalStateSampler::sample, nb::arg("state"))
        .def("sampleNear", &ob::ConditionalStateSampler::sampleNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"));
}
