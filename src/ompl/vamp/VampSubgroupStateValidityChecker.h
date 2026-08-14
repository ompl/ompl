#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/IndexedSubStateSpace.h>
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

    /** \brief State validity checker that bridges an
     * ompl::base::IndexedSubStateSpace into VAMP's SIMD collision checker.
     *
     * The subspace owns the reduced-DOF bookkeeping (active indices and frozen
     * ambient pose); this checker expands each reduced-DOF state to a full
     * \c Robot::Configuration and forwards to \c vamp::planning::validate_motion.
     *
     * \tparam Robot VAMP robot model supplying \c Configuration, \c dimension,
     *         and the SIMD forward-kinematics/collision kernels.
     * \tparam rake SIMD lane width used for collision checking.
     *
     * \par Performance notes
     * - The subspace pointer is resolved once at construction.
     * - The frozen pose is cached as a pre-cast single-precision array on the
     *   checker itself and refreshed lazily when the subspace's frozen version
     *   counter advances. The per-call hot path then writes the frozen part as a
     *   single SIMD-friendly memcpy and only casts the (few) active DOFs.
     * - Active indices are mirrored into a fixed-size array of length
     *   \c Robot::dimension so the per-call loop reads from contiguous,
     *   cache-resident storage instead of chasing a vector indirection.
     * - The subspace's ambient dimension must equal \c Robot::dimension; this is
     *   verified at construction so misuse fails loudly rather than producing
     *   silently-wrong collision checks. */
    template <typename Robot, std::size_t rake = ::vamp::FloatVectorWidth>
    class VampSubgroupStateValidityChecker : public ob::StateValidityChecker
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
         *        reference and must outlive the checker. */
        VampSubgroupStateValidityChecker(ob::SpaceInformation *si, const Environment &env)
          : ob::StateValidityChecker(si), env_(env), subspace_(resolveSubspace(si->getStateSpace().get()))
        {
            primeCaches();
        }

        /** \brief Construct from a SpaceInformationPtr; \a si must own an
         * ompl::base::IndexedSubStateSpace of ambient dimension
         * \c Robot::dimension, and \a env must outlive the checker. */
        VampSubgroupStateValidityChecker(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::StateValidityChecker(si), env_(env), subspace_(resolveSubspace(si->getStateSpace().get()))
        {
            primeCaches();
        }

        /** \brief Expand \a state to a full configuration and report whether it
         * is collision-free. */
        auto isValid(const ob::State *state) const -> bool override
        {
            const auto *rv = state->as<ob::RealVectorStateSpace::StateType>();
            auto configuration = expand(rv);
            return ::vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_);
        }

    private:
        /** \brief Lift a reduced-DOF state into a full VAMP \c Configuration.
         *
         * Writes the cached frozen ambient pose, then overwrites the active DOFs
         * from \a rv. Lazily refreshes the frozen cache when the subspace reports
         * a new version, so ompl::base::IndexedSubStateSpace::setFrozenValues is
         * picked up live without per-call rebuilds. */
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
        static auto resolveSubspace(ob::StateSpace *space) -> const ob::IndexedSubStateSpace *
        {
            auto *sub = dynamic_cast<const ob::IndexedSubStateSpace *>(space);
            if (sub == nullptr)
            {
                throw ompl::Exception(
                    "VampSubgroupStateValidityChecker: SpaceInformation's state space is not an IndexedSubStateSpace");
            }
            if (sub->getAmbientDimension() != Robot::dimension)
            {
                throw ompl::Exception("VampSubgroupStateValidityChecker: IndexedSubStateSpace ambient "
                                      "dimension does not match Robot::dimension");
            }
            return sub;
        }

        /** \brief VAMP collision environment used for every check (not owned). */
        const Environment &env_;
        /** \brief Subspace queried for the active indices and frozen ambient pose. */
        const ob::IndexedSubStateSpace *subspace_;
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
