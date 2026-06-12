#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/SubspaceStateSpace.h>
#include <ompl/base/spaces/WrapperStateSpace.h>
#include <ompl/util/Exception.h>

#include <ompl/vamp/Utils.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <vamp/collision/environment.hh>
#include <vamp/planning/validate.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    //==========================================================================
    // VAMP Subgroup State Validity Checker for OMPL
    //
    // Adapter that bridges a general ``ompl::base::SubspaceStateSpace`` into
    // VAMP's SIMD collision validator. The subspace handles the reduced-DOF
    // bookkeeping (active indices, frozen ambient pose); this checker simply
    // expands each reduced-DOF state to a full Robot::Configuration and
    // forwards to ``vamp::planning::validate_motion``. Works for both
    // unconstrained planning (SpaceInformation directly over the subspace)
    // and constrained planning (the subspace wrapped by a
    // ConstrainedStateSpace / ProjectedStateSpace) — the wrapper walk in
    // ``resolveSubspace`` peels both layers.
    //
    // Performance notes:
    //  * The subspace pointer and the "is this SI's state space wrapped?" flag
    //    are resolved once at construction. The per-call hot path then skips
    //    the ``dynamic_cast`` for the common unconstrained case.
    //  * The frozen pose is cached as a pre-cast single-precision array on the
    //    checker itself and refreshed lazily when the subspace's frozen
    //    version counter advances. The per-call hot path then writes the
    //    frozen part as a single SIMD-friendly memcpy and only casts the (few)
    //    active DOFs.
    //  * Active indices are mirrored into a fixed-size array of length
    //    Robot::dimension so the per-call loop reads from contiguous, cache-
    //    resident storage instead of chasing a vector indirection.
    //  * The subspace's ambient dimension must equal ``Robot::dimension``; we
    //    verify that at construction so misuse fails loudly rather than
    //    producing silently-wrong collision checks.
    //==========================================================================

    template <typename Robot, std::size_t rake = ::vamp::FloatVectorWidth>
    class VampSubgroupStateValidityChecker : public ob::StateValidityChecker
    {
    public:
        using Environment = ::vamp::collision::Environment<::vamp::FloatVector<rake>>;
        using Configuration = typename Robot::Configuration;

        VampSubgroupStateValidityChecker(ob::SpaceInformation *si, const Environment &env)
          : ob::StateValidityChecker(si)
          , env_(env)
          , subspace_(resolveSubspace(si->getStateSpace().get()))
          , wrapped_(static_cast<const ob::StateSpace *>(subspace_) != si->getStateSpace().get())
        {
            primeCaches();
        }

        VampSubgroupStateValidityChecker(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::StateValidityChecker(si)
          , env_(env)
          , subspace_(resolveSubspace(si->getStateSpace().get()))
          , wrapped_(static_cast<const ob::StateSpace *>(subspace_) != si->getStateSpace().get())
        {
            primeCaches();
        }

        auto isValid(const ob::State *state) const -> bool override
        {
            const auto *rv = wrapped_ ? extract_real_state(state) : state->as<ob::RealVectorStateSpace::StateType>();
            auto configuration = expand(rv);
            return ::vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_);
        }

    private:
        // Lift a reduced-DOF state into a full VAMP Configuration using the
        // cached frozen-float buffer and the cached active-index array. Lazily
        // refreshes the frozen cache when the subspace reports a new version
        // so ``setFrozenValues`` is picked up live without per-call rebuilds.
        auto expand(const ob::RealVectorStateSpace::StateType *rv) const -> Configuration
        {
            const std::size_t v = subspace_->getFrozenVersion();
            if (v != frozen_version_)
                refreshFrozenCache(v);

            alignas(Configuration::S::Alignment) std::array<float, Configuration::num_scalars> buf;
            std::memcpy(buf.data(), frozen_float_.data(), sizeof(float) * Robot::dimension);
            for (std::size_t i = 0; i < active_count_; ++i)
                buf[active_indices_[i]] = static_cast<float>(rv->values[i]);
            return Configuration(buf.data());
        }

        void primeCaches()
        {
            const auto &active = subspace_->getActiveIndices();
            active_count_ = active.size();
            for (std::size_t i = 0; i < active_count_; ++i)
                active_indices_[i] = active[i];
            refreshFrozenCache(subspace_->getFrozenVersion());
        }

        void refreshFrozenCache(std::size_t version) const
        {
            const auto &frozen = subspace_->getFrozenValues();
            for (std::size_t i = 0; i < Robot::dimension; ++i)
                frozen_float_[i] = static_cast<float>(frozen[i]);
            frozen_version_ = version;
        }

        static auto resolveSubspace(ob::StateSpace *space) -> const ob::SubspaceStateSpace *
        {
            ob::StateSpace *current = space;
            while (current != nullptr)
            {
                if (auto *sub = dynamic_cast<const ob::SubspaceStateSpace *>(current))
                {
                    if (sub->getAmbientDimension() != Robot::dimension)
                    {
                        throw ompl::Exception("VampSubgroupStateValidityChecker: SubspaceStateSpace ambient "
                                              "dimension does not match Robot::dimension");
                    }
                    return sub;
                }
                if (auto *wrapper = dynamic_cast<ob::WrapperStateSpace *>(current))
                {
                    current = wrapper->getSpace().get();
                    continue;
                }
                break;
            }
            throw ompl::Exception(
                "VampSubgroupStateValidityChecker: SpaceInformation's state space is not (or does not wrap) a "
                "SubspaceStateSpace");
        }

        const Environment &env_;
        const ob::SubspaceStateSpace *subspace_;
        bool wrapped_;
        std::size_t active_count_{0};
        std::array<std::size_t, Robot::dimension> active_indices_{};
        mutable std::array<float, Robot::dimension> frozen_float_{};
        mutable std::size_t frozen_version_{static_cast<std::size_t>(-1)};
    };

}  // namespace ompl::vamp
