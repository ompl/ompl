#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/IndexedSubStateSpace.h>
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

    /** \brief Motion validator that bridges an ompl::base::IndexedSubStateSpace
     * into VAMP's SIMD swept-edge collision check.
     *
     * Companion to VampSubgroupStateValidityChecker. VAMP's swept-edge
     * \c validate_motion linearly interpolates the two endpoints at
     * \c Robot::resolution and runs \c rake-wide SIMD forward-kinematics and
     * collision checking over the interpolated samples.
     *
     * \tparam Robot VAMP robot model supplying \c Configuration, \c dimension,
     *         \c resolution, and the SIMD kernels.
     * \tparam rake SIMD lane width used for collision checking.
     *
     * \par Performance notes
     * Uses the same caching strategy as VampSubgroupStateValidityChecker on the
     * hot path:
     * - The frozen ambient pose is cached as a pre-cast single-precision array,
     *   refreshed lazily on subspace version bumps so
     *   ompl::base::IndexedSubStateSpace::setFrozenValues stays live-updatable.
     * - Active indices are mirrored into a fixed-size array so the per-edge
     *   expansion stays cache-resident. */
    template <typename Robot, std::size_t rake = ::vamp::FloatVectorWidth>
    class VampSubgroupMotionValidator : public ob::MotionValidator
    {
    public:
        using Environment = ::vamp::collision::Environment<::vamp::FloatVector<rake>>;
        using Configuration = typename Robot::Configuration;

        /** \brief Construct from a raw SpaceInformation.
         *
         * \param si Space information whose state space must be an
         *        ompl::base::IndexedSubStateSpace of ambient dimension
         *        \c Robot::dimension.
         * \param env VAMP collision environment used for all checks; stored by
         *        reference and must outlive the validator. */
        VampSubgroupMotionValidator(ob::SpaceInformation *si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
            subspace_ = resolve(si->getStateSpace().get());
            primeCaches();
        }

        /** \brief Construct from a SpaceInformationPtr; \a si must own an
         * ompl::base::IndexedSubStateSpace of ambient dimension
         * \c Robot::dimension, and \a env must outlive the validator. */
        VampSubgroupMotionValidator(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
            subspace_ = resolve(si->getStateSpace().get());
            primeCaches();
        }

        /** \brief Report whether the motion from \a s1 to \a s2 is collision-free,
         * expanding both endpoints to full configurations and delegating to
         * VAMP's swept-edge \c validate_motion. */
        auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
        {
            refreshFrozenIfStale();
            return ::vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
                expand(s1->as<ob::RealVectorStateSpace::StateType>()),
                expand(s2->as<ob::RealVectorStateSpace::StateType>()), env_);
        }

        /** \brief Last-valid-state overload of checkMotion().
         *
         * VAMP's swept-edge validator returns only a single bool, not the last
         * valid fraction along the motion, so \a last_valid is cleared and the
         * plain checkMotion() result is returned. */
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
        /** \brief Lift a reduced-DOF state into a full VAMP \c Configuration by
         * writing the cached frozen pose and overwriting the active DOFs from
         * \a rv. Assumes the frozen cache is current (see refreshFrozenIfStale()). */
        auto expand(const ob::RealVectorStateSpace::StateType *rv) const -> Configuration
        {
            alignas(Configuration::S::Alignment) std::array<float, Configuration::num_scalars> buf;
            std::memcpy(buf.data(), frozen_float_.data(), sizeof(float) * Robot::dimension);
            for (std::size_t i = 0; i < active_count_; ++i)
                buf[active_indices_[i]] = static_cast<float>(rv->values[i]);
            return Configuration(buf.data());
        }

        /** \brief Cache the active-index array and frozen pose from the subspace
         * at construction. */
        void primeCaches()
        {
            const auto &active = subspace_->getActiveIndices();
            active_count_ = active.size();
            for (std::size_t i = 0; i < active_count_; ++i)
                active_indices_[i] = active[i];
            refreshFrozenCache(subspace_->getFrozenVersion());
        }

        /** \brief Refresh the frozen cache if the subspace's frozen version has
         * advanced since the last check. */
        void refreshFrozenIfStale() const
        {
            const std::size_t v = subspace_->getFrozenVersion();
            if (v != frozen_version_)
                refreshFrozenCache(v);
        }

        /** \brief Re-cast the subspace's frozen ambient pose into the
         * single-precision cache and record \a version as current. */
        void refreshFrozenCache(std::size_t version) const
        {
            const auto &frozen = subspace_->getFrozenValues();
            for (std::size_t i = 0; i < Robot::dimension; ++i)
                frozen_float_[i] = static_cast<float>(frozen[i]);
            frozen_version_ = version;
        }

        /** \brief Return \a space cast to ompl::base::IndexedSubStateSpace,
         * throwing if it is not one or if its ambient dimension does not match
         * \c Robot::dimension. */
        static auto resolve(ob::StateSpace *space) -> const ob::IndexedSubStateSpace *
        {
            auto *sub = dynamic_cast<const ob::IndexedSubStateSpace *>(space);
            if (sub == nullptr)
            {
                throw ompl::Exception(
                    "VampSubgroupMotionValidator: SpaceInformation's state space is not an IndexedSubStateSpace");
            }
            if (sub->getAmbientDimension() != Robot::dimension)
            {
                throw ompl::Exception("VampSubgroupMotionValidator: IndexedSubStateSpace ambient dimension "
                                      "does not match Robot::dimension");
            }
            return sub;
        }

        /** \brief VAMP collision environment used for every check (not owned). */
        const Environment &env_;
        /** \brief Subspace queried for the active indices and frozen ambient pose. */
        const ob::IndexedSubStateSpace *subspace_{nullptr};
        /** \brief Number of active (planner-varied) DOFs. */
        std::size_t active_count_{0};
        /** \brief Ambient index of each active DOF (first \c active_count_ entries used). */
        std::array<std::size_t, Robot::dimension> active_indices_{};
        /** \brief Frozen ambient pose pre-cast to single precision for the SIMD hot path. */
        mutable std::array<float, Robot::dimension> frozen_float_{};
        /** \brief Frozen-pose version mirrored by \c frozen_float_; starts stale to force the first refresh. */
        mutable std::size_t frozen_version_{static_cast<std::size_t>(-1)};
    };

}  // namespace ompl::vamp
