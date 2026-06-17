#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SubspaceStateSpace.h>
#include <ompl/util/Exception.h>

#include <ompl/vamp/Utils.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <utility>
#include <vamp/collision/environment.hh>
#include <vamp/planning/validate.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    //==========================================================================
    // VAMP Subgroup Motion Validator for OMPL
    //
    // Companion to VampSubgroupStateValidityChecker. Bridges a general
    // ``ompl::base::SubspaceStateSpace`` into VAMP's SIMD collision validator:
    // VAMP's swept-edge ``validate_motion`` linearly interpolates the two
    // endpoints at ``Robot::resolution`` and runs ``rake``-wide SIMD FK+CC
    // over the interpolated samples.
    //
    // Same caching strategy as the state validity checker for the hot path:
    //   * Frozen ambient pose cached as a pre-cast single-precision array,
    //     refreshed lazily on subspace version bumps so ``setFrozenValues``
    //     stays live-updatable.
    //   * Active indices mirrored into a fixed-size array so the per-edge
    //     expansion stays cache-resident.
    //==========================================================================

    template <typename Robot, std::size_t rake = ::vamp::FloatVectorWidth>
    class VampSubgroupMotionValidator : public ob::MotionValidator
    {
    public:
        using Environment = ::vamp::collision::Environment<::vamp::FloatVector<rake>>;
        using Configuration = typename Robot::Configuration;

        VampSubgroupMotionValidator(ob::SpaceInformation *si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
            subspace_ = resolve(si->getStateSpace().get());
            primeCaches();
        }

        VampSubgroupMotionValidator(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
            subspace_ = resolve(si->getStateSpace().get());
            primeCaches();
        }

        auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
        {
            refreshFrozenIfStale();
            return ::vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
                expand(s1->as<ob::RealVectorStateSpace::StateType>()),
                expand(s2->as<ob::RealVectorStateSpace::StateType>()), env_);
        }

        auto checkMotion(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &last_valid) const
            -> bool override
        {
            // VAMP's swept-edge validator returns only a single bool, not the last
            // valid fraction along the motion, so we leave last_valid empty.
            last_valid.first = nullptr;
            last_valid.second = 0.0;
            return checkMotion(s1, s2);
        }

    private:
        auto expand(const ob::RealVectorStateSpace::StateType *rv) const -> Configuration
        {
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

        void refreshFrozenIfStale() const
        {
            const std::size_t v = subspace_->getFrozenVersion();
            if (v != frozen_version_)
                refreshFrozenCache(v);
        }

        void refreshFrozenCache(std::size_t version) const
        {
            const auto &frozen = subspace_->getFrozenValues();
            for (std::size_t i = 0; i < Robot::dimension; ++i)
                frozen_float_[i] = static_cast<float>(frozen[i]);
            frozen_version_ = version;
        }

        static auto resolve(ob::StateSpace *space) -> const ob::SubspaceStateSpace *
        {
            auto *sub = dynamic_cast<const ob::SubspaceStateSpace *>(space);
            if (sub == nullptr)
            {
                throw ompl::Exception(
                    "VampSubgroupMotionValidator: SpaceInformation's state space is not a SubspaceStateSpace");
            }
            if (sub->getAmbientDimension() != Robot::dimension)
            {
                throw ompl::Exception("VampSubgroupMotionValidator: SubspaceStateSpace ambient dimension "
                                      "does not match Robot::dimension");
            }
            return sub;
        }

        const Environment &env_;
        const ob::SubspaceStateSpace *subspace_{nullptr};
        std::size_t active_count_{0};
        std::array<std::size_t, Robot::dimension> active_indices_{};
        mutable std::array<float, Robot::dimension> frozen_float_{};
        mutable std::size_t frozen_version_{static_cast<std::size_t>(-1)};
    };

}  // namespace ompl::vamp
