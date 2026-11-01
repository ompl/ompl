#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/MotionValidator.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/StateSpace.h"
#include "PyGC.h"

namespace ompl::binding::base
{
    /** \brief Validate the \e reals argument of a StateSpace::copyFromReals() call before handing it to C++. */
    inline void checkRealsSize(const ompl::base::StateSpace &space, const std::vector<double> &reals,
                               const char *function)
    {
        const std::size_t expected = space.getValueLocations().size();
        if (reals.size() == expected)
            return;

        std::string message = std::string(function) + "(): state space '" + space.getName() + "' expects " +
                              std::to_string(expected) + " real value(s), but " + std::to_string(reals.size()) +
                              " were given.";
        if (expected == 0)
            message += " This space has no known value locations; call setup() (or computeLocations()) on it first.";
        throw std::invalid_argument(message);
    }

    /// MotionValidator keeps only a raw SpaceInformation pointer, so validators built from Python own it here.
    struct OwnsSpaceInformation
    {
        ompl::base::SpaceInformationPtr ownedSi;
    };

    template <typename Validator>
    struct SiOwningMotionValidator : Validator, OwnsSpaceInformation
    {
        explicit SiOwningMotionValidator(const ompl::base::SpaceInformationPtr &si)
          : Validator(si), OwnsSpaceInformation{si}
        {
        }
    };

    inline OwnsSpaceInformation *siOwner(PyObject *self)
    {
        if (!nanobind::inst_ready(self) || !gc::pythonOwned(self))
            return nullptr;
        return dynamic_cast<OwnsSpaceInformation *>(nanobind::inst_ptr<ompl::base::MotionValidator>(self));
    }

    inline int motionValidatorTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (OwnsSpaceInformation *owner = siOwner(self))
            return gc::visitHeld(owner->ownedSi, visit, arg);
        return 0;
    }

    inline int motionValidatorClear(PyObject *self)
    {
        if (OwnsSpaceInformation *owner = siOwner(self); owner && gc::holdsPython(owner->ownedSi))
            owner->ownedSi.reset();
        return 0;
    }

    inline const PyType_Slot motionValidatorSlots[] = {{Py_tp_traverse, (void *)motionValidatorTraverse},
                                                       {Py_tp_clear, (void *)motionValidatorClear},
                                                       {0, 0}};

    /// Binds a MotionValidator type whose constructor takes the SpaceInformation.
    template <typename Validator>
    void bindSiOwningMotionValidator(nanobind::module_ &m, const char *name)
    {
        nanobind::class_<Validator, ompl::base::MotionValidator, SiOwningMotionValidator<Validator>>(
            m, name, nanobind::type_slots(motionValidatorSlots))
            .def(
                "__init__", [](Validator *self, const ompl::base::SpaceInformationPtr &si)
                { new (self) SiOwningMotionValidator<Validator>(si); }, nanobind::arg("si"));
    }
}  // namespace ompl::binding::base
