#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/SubspaceStateSpace.h>
#include <ompl/base/spaces/WrapperStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/util/Exception.h>

#include <ompl/vamp/Utils.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <utility>
#include <vector>
#include <vamp/collision/environment.hh>
#include <vamp/planning/validate.hh>
#include <vamp/vector.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    //==========================================================================
    // VAMP Subgroup Motion Validator for OMPL
    //
    // Companion to VampSubgroupStateValidityChecker. Bridges a general
    // ``ompl::base::SubspaceStateSpace`` (possibly wrapped by a
    // ``ConstrainedStateSpace``) into VAMP's SIMD collision validator and
    // chooses the right strategy automatically at construction:
    //
    //   * Unconstrained: VAMP's swept-edge ``validate_motion`` linearly
    //     interpolates the two endpoints at ``Robot::resolution`` and runs
    //     ``rake``-wide SIMD FK+CC over the interpolated samples.
    //
    //   * Constrained: we walk the manifold via the constrained state
    //     space's own ``discreteGeodesic`` (which mirrors the discretization
    //     used by OMPL's default ``ConstrainedMotionValidator``), then pack
    //     the geodesic samples into a ``Robot::ConfigurationBlock<rake>``
    //     and validate each block with ``Robot::fkcc<rake>``. This recovers
    //     the rake-wide SIMD batching that the default constrained
    //     validator forfeits by calling per-state ``isValid``.
    //
    // Same caching strategy as the state validity checker for the hot path:
    //   * Wrapped-state detection cached at construction.
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
        using Block = typename Robot::template ConfigurationBlock<rake>;

        VampSubgroupMotionValidator(ob::SpaceInformation *si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
            Resolved r = resolve(si->getStateSpace().get());
            subspace_ = r.subspace;
            constrained_ = r.constrained;
            wrapped_ = static_cast<const ob::StateSpace *>(subspace_) != si->getStateSpace().get();
            primeCaches();
        }

        VampSubgroupMotionValidator(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
            Resolved r = resolve(si->getStateSpace().get());
            subspace_ = r.subspace;
            constrained_ = r.constrained;
            wrapped_ = static_cast<const ob::StateSpace *>(subspace_) != si->getStateSpace().get();
            primeCaches();
        }

        auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
        {
            refreshFrozenIfStale();
            if (constrained_ != nullptr)
                return checkMotionConstrained(s1, s2);
            return ::vamp::planning::validate_motion<Robot, rake, Robot::resolution>(expand(unwrap(s1)),
                                                                                     expand(unwrap(s2)), env_);
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
        struct Resolved
        {
            const ob::SubspaceStateSpace *subspace{nullptr};
            ob::ConstrainedStateSpace *constrained{nullptr};
        };

        auto unwrap(const ob::State *state) const -> const ob::RealVectorStateSpace::StateType *
        {
            return wrapped_ ? extract_real_state(state) : state->as<ob::RealVectorStateSpace::StateType>();
        }

        auto expand(const ob::RealVectorStateSpace::StateType *rv) const -> Configuration
        {
            alignas(Configuration::S::Alignment) std::array<float, Configuration::num_scalars> buf;
            std::memcpy(buf.data(), frozen_float_.data(), sizeof(float) * Robot::dimension);
            for (std::size_t i = 0; i < active_count_; ++i)
                buf[active_indices_[i]] = static_cast<float>(rv->values[i]);
            return Configuration(buf.data());
        }

        // Manifold-aware checkMotion: ask the constrained state space to walk
        // its own geodesic, then batch-validate the resulting on-manifold
        // samples through Robot::fkcc<rake>. Samples are produced with
        // ``interpolate=true`` so the walker doesn't bail early on a per-state
        // svc check — we do the collision verification ourselves in rake-wide
        // batches.
        auto checkMotionConstrained(const ob::State *s1, const ob::State *s2) const -> bool
        {
            // Goal must lie on the manifold (cheap test). Matches what OMPL's
            // default ConstrainedMotionValidator does before walking.
            if (!constrained_->getConstraint()->isSatisfied(s2))
                return false;

            std::vector<ob::State *> samples;
            const bool reached = constrained_->discreteGeodesic(s1, s2, /*interpolate=*/true, &samples);
            const bool ok = reached && validateSamples(samples);
            for (auto *s : samples)
                constrained_->freeState(s);
            return ok;
        }

        // Pack samples [1, n) (i.e., skip the start state, which the planner
        // has already validated) into ConfigurationBlock<rake>'s, validating
        // each filled block via fkcc<rake>. The final partial block is padded
        // by duplicating an already-included lane so the "all valid" semantics
        // hold without per-lane fallbacks.
        auto validateSamples(const std::vector<ob::State *> &samples) const -> bool
        {
            if (samples.size() < 2)
                return true;

            alignas(::vamp::FloatVectorAlignment) std::array<float, Robot::dimension * rake> blk{};
            std::size_t cursor = 0;
            for (std::size_t i = 1; i < samples.size(); ++i)
            {
                writeLane(blk, cursor, samples[i]);
                if (++cursor == rake)
                {
                    Block block(blk.data());
                    if (!Robot::template fkcc<rake>(env_, block))
                        return false;
                    cursor = 0;
                }
            }
            if (cursor > 0)
            {
                for (std::size_t lane = cursor; lane < rake; ++lane)
                    for (std::size_t d = 0; d < Robot::dimension; ++d)
                        blk[d * rake + lane] = blk[d * rake + 0];
                Block block(blk.data());
                if (!Robot::template fkcc<rake>(env_, block))
                    return false;
            }
            return true;
        }

        // Lift one OMPL state into one lane of a ConfigurationBlock<rake>.
        // Layout matches VAMP's pack expectations: blk[d * rake + lane].
        void writeLane(std::array<float, Robot::dimension * rake> &blk, std::size_t lane, const ob::State *s) const
        {
            const auto *rv = unwrap(s);
            for (std::size_t d = 0; d < Robot::dimension; ++d)
                blk[d * rake + lane] = frozen_float_[d];
            for (std::size_t i = 0; i < active_count_; ++i)
                blk[active_indices_[i] * rake + lane] = static_cast<float>(rv->values[i]);
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

        static auto resolve(ob::StateSpace *space) -> Resolved
        {
            Resolved r;
            ob::StateSpace *current = space;
            while (current != nullptr)
            {
                if (r.constrained == nullptr)
                {
                    if (auto *css = dynamic_cast<ob::ConstrainedStateSpace *>(current))
                        r.constrained = css;
                }
                if (auto *sub = dynamic_cast<const ob::SubspaceStateSpace *>(current))
                {
                    if (sub->getAmbientDimension() != Robot::dimension)
                    {
                        throw ompl::Exception("VampSubgroupMotionValidator: SubspaceStateSpace ambient dimension "
                                              "does not match Robot::dimension");
                    }
                    r.subspace = sub;
                    return r;
                }
                if (auto *wrapper = dynamic_cast<ob::WrapperStateSpace *>(current))
                {
                    current = wrapper->getSpace().get();
                    continue;
                }
                break;
            }
            throw ompl::Exception("VampSubgroupMotionValidator: SpaceInformation's state space is not (or does not "
                                  "wrap) a SubspaceStateSpace");
        }

        const Environment &env_;
        const ob::SubspaceStateSpace *subspace_{nullptr};
        ob::ConstrainedStateSpace *constrained_{nullptr};
        bool wrapped_{false};
        std::size_t active_count_{0};
        std::array<std::size_t, Robot::dimension> active_indices_{};
        mutable std::array<float, Robot::dimension> frozen_float_{};
        mutable std::size_t frozen_version_{static_cast<std::size_t>(-1)};
    };

}  // namespace ompl::vamp
