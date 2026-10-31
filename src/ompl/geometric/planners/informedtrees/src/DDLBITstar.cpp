/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Open Motion Planning Library contributors
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Elias (OMPL contribution) */
#include "ompl/geometric/planners/informedtrees/DDLBITstar.h"
#include "ompl/geometric/planners/informedtrees/bitstar/ImplicitGraph.h"
#include "ompl/geometric/planners/informedtrees/bitstar/Vertex.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/Exception.h"
#include "ompl/util/RandomNumbers.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace ompl
{
    namespace geometric
    {
        /** \brief Perception, configuration and owned state for DDL-BIT*. */
        struct DDLBITstar::Impl
        {
            /** \brief Signed Euclidean grid coordinates. */
            using Key = std::vector<std::int64_t>;
            /** \brief Hash all coordinates of a spatial grid cell. */
            struct Hash
            {
                /** \brief Combine all dimensions of a cell into a hash. */
                std::size_t operator()(const Key &key) const
                {
                    std::size_t hash = 0;
                    for (auto coordinate : key)
                        hash ^= std::hash<std::int64_t>{}(coordinate) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
                    return hash;
                }
            };

            /** \brief Retain allocation and validity services for planner states. */
            explicit Impl(const base::SpaceInformationPtr &spaceInformation) : si_(spaceInformation)
            {
            }

            /** \brief Map a bounded real-vector state to a floor-based cell. */
            Key key(const base::State *state, double resolution) const
            {
                const auto *values = state->as<base::RealVectorStateSpace::StateType>()->values;
                Key result(dimension_);
                for (unsigned int i = 0; i < dimension_; ++i)
                    result[i] = static_cast<std::int64_t>(std::floor(values[i] / resolution));
                return result;
            }

            /** \brief Release perception data and reset counters while retaining settings. */
            void reset()
            {
                boundaries_.clear();
                boundaryCells_.clear();
                occupancy_.clear();
                boundarySamples_ = 0;
                densityRejections_ = 0;
                gridRebuilds_ = 0;
                densityCost_ = base::Cost(std::numeric_limits<double>::infinity());
                initialized_ = false;
            }

            /** \brief Query the active solve's termination condition, if present. */
            bool terminated() const
            {
                return termination_ && (*termination_)();
            }

            /** \brief Discover free-side obstacle frontiers from retained graph states. */
            void scout(const BITstar::VertexPtrVector &states)
            {
                if (states.empty())
                    return;
                const auto *space = si_->getStateSpace()->as<base::RealVectorStateSpace>();
                const auto &bounds = space->getBounds();
                base::ScopedState<> endpoint(si_), candidate(si_), lastValid(si_);
                std::vector<double> direction(dimension_);
                const double angleOffset = rng_.uniformReal(0.0, 2.0 * std::acos(-1.0));
                const double step = space->getLongestValidSegmentLength() / space->getValidSegmentCountFactor();
                for (unsigned int ray = 0; ray < rays_ && !terminated(); ++ray)
                {
                    const auto *origin = states[rng_.uniformInt(0, static_cast<int>(states.size() - 1))]->state();
                    if (dimension_ == 2)
                    {
                        const double angle = angleOffset + 2.0 * std::acos(-1.0) * ray / rays_;
                        direction[0] = std::cos(angle);
                        direction[1] = std::sin(angle);
                    }
                    else
                    {
                        double squaredNorm = 0.0;
                        for (double &coordinate : direction)
                        {
                            coordinate = rng_.gaussian01();
                            squaredNorm += coordinate * coordinate;
                        }
                        if (squaredNorm == 0.0)
                            continue;
                        for (double &coordinate : direction)
                            coordinate /= std::sqrt(squaredNorm);
                    }
                    const auto *values = origin->as<base::RealVectorStateSpace::StateType>()->values;
                    // Bound work without skipping collision-checking intervals on very large domains.
                    double length = 4096.0 * step;
                    for (unsigned int i = 0; i < dimension_; ++i)
                    {
                        if (direction[i] > 0.0)
                            length = std::min(length, (bounds.high[i] - values[i]) / direction[i]);
                        else if (direction[i] < 0.0)
                            length = std::min(length, (bounds.low[i] - values[i]) / direction[i]);
                    }
                    if (!(length > 0.0))
                        continue;
                    for (unsigned int i = 0; i < dimension_; ++i)
                        endpoint[i] = values[i] + length * direction[i];
                    space->enforceBounds(endpoint.get());
                    const unsigned int segments = std::max(1u, static_cast<unsigned int>(std::ceil(length / step)));
                    double previous = 0.0;
                    si_->copyState(lastValid.get(), origin);
                    for (unsigned int segment = 1; segment <= segments && !terminated(); ++segment)
                    {
                        double upper = static_cast<double>(segment) / segments;
                        space->interpolate(origin, endpoint.get(), upper, candidate.get());
                        if (si_->isValid(candidate.get()))
                        {
                            previous = upper;
                            si_->copyState(lastValid.get(), candidate.get());
                            continue;
                        }
                        // Keep the free side of the first detected obstacle transition.
                        double lower = previous;
                        for (unsigned int iteration = 0; iteration < 12; ++iteration)
                        {
                            if (terminated())
                                return;
                            const double middle = 0.5 * (lower + upper);
                            space->interpolate(origin, endpoint.get(), middle, candidate.get());
                            if (si_->isValid(candidate.get()))
                            {
                                lower = middle;
                                si_->copyState(lastValid.get(), candidate.get());
                            }
                            else
                                upper = middle;
                        }
                        if (boundaryCells_.insert(key(lastValid.get(), boundaryResolution_)).second)
                            boundaries_.emplace_back(si_->getStateSpace(), lastValid.get());
                        break;
                    }
                }
            }

            /** \brief Apply hybrid proposals and density admission around an informed sampler. */
            class Sampler : public base::InformedSampler
            {
            public:
                /** \brief Wrap the objective sampler with shared perception state. */
                Sampler(const base::ProblemDefinitionPtr &problem, unsigned int maxCalls, Impl &implementation,
                        const base::InformedSamplerPtr &uniformSampler)
                  : base::InformedSampler(problem, maxCalls), impl_(implementation), uniform_(uniformSampler)
                {
                }

                /** \brief Attempt one admitted proposal below the cost bound. */
                bool sampleUniform(base::State *state, const base::Cost &maxCost) override
                {
                    return sampleUniform(state, opt_->identityCost(), maxCost);
                }

                /** \brief Attempt one bounded hybrid proposal in the cost interval. */
                bool sampleUniform(base::State *state, const base::Cost &minCost, const base::Cost &maxCost) override
                {
                    if (impl_.terminated())
                        return false;
                    const double bias =
                        std::isfinite(maxCost.value()) ? impl_.optimizationBias_ : impl_.explorationBias_;
                    const bool boundary = !impl_.boundaries_.empty() && impl_.rng_.uniform01() < bias;
                    if (boundary)
                    {
                        const auto &mean =
                            impl_.boundaries_[impl_.rng_.uniformInt(0, static_cast<int>(impl_.boundaries_.size() - 1))];
                        auto *values = state->as<base::RealVectorStateSpace::StateType>()->values;
                        for (unsigned int i = 0; i < impl_.dimension_; ++i)
                            values[i] = impl_.rng_.gaussian(mean[i], impl_.standardDeviation_);
                        if (!space_->satisfiesBounds(state))
                            return false;
                        const auto cost = uniform_->heuristicSolnCost(state);
                        if (opt_->isCostBetterThan(cost, minCost) || !opt_->isCostBetterThan(cost, maxCost))
                            return false;
                    }
                    else if (!uniform_->sampleUniform(state, minCost, maxCost))
                        return false;

                    // Count only valid admitted states, never rejected or colliding proposals.
                    if (!space_->satisfiesBounds(state) || !impl_.si_->isValid(state) || impl_.terminated())
                        return false;
                    const auto cell = impl_.key(state, impl_.gridResolution_);
                    const auto found = impl_.occupancy_.find(cell);
                    const std::size_t count = found == impl_.occupancy_.end() ? 0u : found->second;
                    const unsigned int capacity = boundary ? impl_.boundaryCapacity_ : impl_.openCapacity_;
                    const double excess = std::max(0.0, static_cast<double>(count) - capacity + 1.0);
                    const double probability = 1.0 / (1.0 + 2.0 * excess);
                    if (impl_.rng_.uniform01() >= probability)
                    {
                        ++impl_.densityRejections_;
                        return false;
                    }
                    ++impl_.occupancy_[cell];
                    if (boundary)
                        ++impl_.boundarySamples_;
                    return true;
                }

                /** \brief Query whether the objective sampler knows the informed measure. */
                bool hasInformedMeasure() const override
                {
                    return uniform_->hasInformedMeasure();
                }
                /** \brief Delegate the measure of an informed set. */
                double getInformedMeasure(const base::Cost &cost) const override
                {
                    return uniform_->getInformedMeasure(cost);
                }
                /** \brief Delegate the measure of an informed cost slice. */
                double getInformedMeasure(const base::Cost &minCost, const base::Cost &maxCost) const override
                {
                    return uniform_->getInformedMeasure(minCost, maxCost);
                }
                /** \brief Delegate the objective's admissible solution-cost estimate. */
                base::Cost heuristicSolnCost(const base::State *state) const override
                {
                    return uniform_->heuristicSolnCost(state);
                }

            private:
                /** \brief Perception state retained by the owning planner. */
                Impl &impl_;
                /** \brief Objective-specific informed proposal generator. */
                base::InformedSamplerPtr uniform_;
            };

            /** \brief State allocation, bounds and validity services. */
            base::SpaceInformationPtr si_;
            /** \brief Copied condition present only during solve(). */
            std::shared_ptr<const base::PlannerTerminationCondition> termination_;
            /** \brief Real-vector dimension established during setup. */
            unsigned int dimension_{0};
            /** \brief Occupancy cell width in state-space units. */
            double gridResolution_{15.0};
            /** \brief Boundary deduplication cell width in state-space units. */
            double boundaryResolution_{2.0};
            /** \brief Isotropic boundary Gaussian standard deviation. */
            double standardDeviation_{10.0};
            /** \brief Boundary proposal probability before an exact solution. */
            double explorationBias_{0.2};
            /** \brief Boundary proposal probability after an exact solution. */
            double optimizationBias_{0.5};
            /** \brief Soft capacity attributed to uniform proposals. */
            unsigned int openCapacity_{1};
            /** \brief Higher soft capacity attributed to boundary proposals. */
            unsigned int boundaryCapacity_{3};
            /** \brief Scouting rays attempted in each batch. */
            unsigned int rays_{18};
            /** \brief Randomness used for proposals, ray directions and admission. */
            RNG rng_;
            /** \brief Owned free-side boundary representatives. */
            std::vector<base::ScopedState<>> boundaries_;
            /** \brief Cells represented in the boundary library. */
            std::unordered_set<Key, Hash> boundaryCells_;
            /** \brief Counts of retained or admitted states by cell. */
            std::unordered_map<Key, std::size_t, Hash> occupancy_;
            /** \brief Best cost at the last occupancy reconstruction. */
            base::Cost densityCost_{std::numeric_limits<double>::infinity()};
            /** \brief Whether initial occupancy has been reconstructed. */
            bool initialized_{false};
            /** \brief Accepted valid Gaussian proposals since clear(). */
            std::size_t boundarySamples_{0};
            /** \brief Valid proposals rejected by the soft lock since clear(). */
            std::size_t densityRejections_{0};
            /** \brief Occupancy reconstructions after improvements since clear(). */
            std::size_t gridRebuilds_{0};
        };

        DDLBITstar::DDLBITstar(const base::SpaceInformationPtr &spaceInformation)
          : BITstar(spaceInformation, "DDLBITstar"), impl_(new Impl(spaceInformation))
        {
            setSamplesPerBatch(50);
            setRewireFactor(1.1);
            declareParam<double>("ddl_grid_resolution", this, &DDLBITstar::setGridResolution,
                                 &DDLBITstar::getGridResolution);
            declareParam<double>("ddl_boundary_resolution", this, &DDLBITstar::setBoundaryResolution,
                                 &DDLBITstar::getBoundaryResolution);
            declareParam<double>("ddl_boundary_stddev", this, &DDLBITstar::setBoundaryStandardDeviation,
                                 &DDLBITstar::getBoundaryStandardDeviation);
            declareParam<double>("ddl_exploration_bias", this, &DDLBITstar::setExplorationBias,
                                 &DDLBITstar::getExplorationBias);
            declareParam<double>("ddl_optimization_bias", this, &DDLBITstar::setOptimizationBias,
                                 &DDLBITstar::getOptimizationBias);
            declareParam<unsigned int>("ddl_open_capacity", this, &DDLBITstar::setOpenCellCapacity,
                                       &DDLBITstar::getOpenCellCapacity);
            declareParam<unsigned int>("ddl_boundary_capacity", this, &DDLBITstar::setBoundaryCellCapacity,
                                       &DDLBITstar::getBoundaryCellCapacity);
            declareParam<unsigned int>("ddl_scouting_rays", this, &DDLBITstar::setScoutingRays,
                                       &DDLBITstar::getScoutingRays);
            addPlannerProgressProperty("DDL boundary points INTEGER",
                                       [this] { return std::to_string(numBoundaryPoints()); });
            addPlannerProgressProperty("DDL density rejections INTEGER",
                                       [this] { return std::to_string(numDensityRejections()); });
        }

        DDLBITstar::~DDLBITstar() = default;

        void DDLBITstar::setup()
        {
            const auto *space = dynamic_cast<const base::RealVectorStateSpace *>(si_->getStateSpace().get());
            if (!space || space->getDimension() == 0)
                throw Exception("DDLBITstar requires a positive-dimensional RealVectorStateSpace");
            if (getJustInTimeSampling())
                throw Exception("DDLBITstar does not support just-in-time sampling");
            const auto &bounds = space->getBounds();
            const double resolution = std::min(impl_->gridResolution_, impl_->boundaryResolution_);
            for (unsigned int i = 0; i < space->getDimension(); ++i)
            {
                if (!std::isfinite(bounds.low[i]) || !std::isfinite(bounds.high[i]) ||
                    bounds.low[i] >= bounds.high[i] || std::abs(bounds.low[i] / resolution) >= std::ldexp(1.0, 62) ||
                    std::abs(bounds.high[i] / resolution) >= std::ldexp(1.0, 62))
                    throw Exception("DDLBITstar requires finite bounds representable at its grid resolutions");
            }
            if (!std::isfinite(space->getMaximumExtent()))
                throw Exception("DDLBITstar requires a finite maximum extent");
            impl_->dimension_ = space->getDimension();
            BITstar::setup();
        }

        void DDLBITstar::clear()
        {
            BITstar::clear();
            impl_->reset();
        }

        base::PlannerStatus DDLBITstar::solve(const base::PlannerTerminationCondition &terminationCondition)
        {
            if (getJustInTimeSampling())
                throw Exception("DDLBITstar does not support just-in-time sampling");
            impl_->termination_ = std::make_shared<base::PlannerTerminationCondition>(terminationCondition);
            /** \brief Clear the active termination condition on every exit path. */
            struct ResetTermination
            {
                /** \brief Perception state retained by the owning planner. */
                Impl &impl_;
                /** \brief Release the copied condition on return or exception. */
                ~ResetTermination()
                {
                    impl_.termination_.reset();
                }
            } resetTermination{*impl_};
            return BITstar::solve(terminationCondition);
        }

        base::InformedSamplerPtr DDLBITstar::allocInformedStateSampler(const base::ProblemDefinitionPtr &problem,
                                                                       unsigned int maxCalls)
        {
            // Each outer BIT* trial gets one bounded proposal; BIT* owns the batch attempt budget.
            auto uniformSampler = BITstar::allocInformedStateSampler(problem, 1u);
            return std::make_shared<Impl::Sampler>(problem, maxCalls, *impl_, uniformSampler);
        }

        void DDLBITstar::onNewBatch(const ImplicitGraph &graph)
        {
            const auto states = graph.getCopyOfSamples();
            if (!impl_->initialized_ ||
                pdef_->getOptimizationObjective()->isCostBetterThan(bestCost(), impl_->densityCost_))
            {
                impl_->occupancy_.clear();
                for (const auto &state : states)
                    ++impl_->occupancy_[impl_->key(state->state(), impl_->gridResolution_)];
                if (impl_->initialized_)
                    ++impl_->gridRebuilds_;
                impl_->densityCost_ = bestCost();
                impl_->initialized_ = true;
            }
            impl_->scout(states);
        }

        void DDLBITstar::checkConfigurationChange() const
        {
            if (isSetup())
                throw Exception("Call clear() before changing DDLBITstar sampling parameters");
        }

        namespace
        {
            /** \brief Validate a finite positive spatial parameter. */
            void positive(double value)
            {
                if (!std::isfinite(value) || value <= 0.0)
                    throw Exception("DDLBITstar spatial parameters must be finite and positive");
            }
            /** \brief Validate a finite mixture probability strictly below one. */
            void probability(double value)
            {
                if (!std::isfinite(value) || value < 0.0 || value >= 1.0)
                    throw Exception("DDLBITstar boundary biases must be finite and in [0,1)");
            }
        }  // namespace

        void DDLBITstar::setGridResolution(double value)
        {
            checkConfigurationChange();
            positive(value);
            impl_->gridResolution_ = value;
        }
        double DDLBITstar::getGridResolution() const
        {
            return impl_->gridResolution_;
        }
        void DDLBITstar::setBoundaryResolution(double value)
        {
            checkConfigurationChange();
            positive(value);
            impl_->boundaryResolution_ = value;
        }
        double DDLBITstar::getBoundaryResolution() const
        {
            return impl_->boundaryResolution_;
        }
        void DDLBITstar::setBoundaryStandardDeviation(double value)
        {
            checkConfigurationChange();
            positive(value);
            impl_->standardDeviation_ = value;
        }
        double DDLBITstar::getBoundaryStandardDeviation() const
        {
            return impl_->standardDeviation_;
        }
        void DDLBITstar::setExplorationBias(double value)
        {
            checkConfigurationChange();
            probability(value);
            impl_->explorationBias_ = value;
        }
        double DDLBITstar::getExplorationBias() const
        {
            return impl_->explorationBias_;
        }
        void DDLBITstar::setOptimizationBias(double value)
        {
            checkConfigurationChange();
            probability(value);
            impl_->optimizationBias_ = value;
        }
        double DDLBITstar::getOptimizationBias() const
        {
            return impl_->optimizationBias_;
        }
        void DDLBITstar::setOpenCellCapacity(unsigned int value)
        {
            checkConfigurationChange();
            if (value == 0 || value >= impl_->boundaryCapacity_)
                throw Exception("DDLBITstar open capacity must be positive and below boundary capacity");
            impl_->openCapacity_ = value;
        }
        unsigned int DDLBITstar::getOpenCellCapacity() const
        {
            return impl_->openCapacity_;
        }
        void DDLBITstar::setBoundaryCellCapacity(unsigned int value)
        {
            checkConfigurationChange();
            if (value <= impl_->openCapacity_)
                throw Exception("DDLBITstar boundary capacity must exceed open capacity");
            impl_->boundaryCapacity_ = value;
        }
        unsigned int DDLBITstar::getBoundaryCellCapacity() const
        {
            return impl_->boundaryCapacity_;
        }
        void DDLBITstar::setScoutingRays(unsigned int value)
        {
            checkConfigurationChange();
            if (value == 0)
                throw Exception("DDLBITstar requires at least one scouting ray");
            impl_->rays_ = value;
        }
        unsigned int DDLBITstar::getScoutingRays() const
        {
            return impl_->rays_;
        }
        std::size_t DDLBITstar::numBoundaryPoints() const
        {
            return impl_->boundaries_.size();
        }
        std::size_t DDLBITstar::numBoundarySamples() const
        {
            return impl_->boundarySamples_;
        }
        std::size_t DDLBITstar::numDensityRejections() const
        {
            return impl_->densityRejections_;
        }
        std::size_t DDLBITstar::numGridRebuilds() const
        {
            return impl_->gridRebuilds_;
        }
    }  // namespace geometric
}  // namespace ompl
