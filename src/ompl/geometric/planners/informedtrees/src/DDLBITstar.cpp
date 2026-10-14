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
        struct DDLBITstar::Impl
        {
            using Key = std::vector<std::int64_t>;
            struct Hash
            {
                std::size_t operator()(const Key &key) const
                {
                    std::size_t hash = 0;
                    for (auto coordinate : key)
                        hash ^= std::hash<std::int64_t>{}(coordinate) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
                    return hash;
                }
            };

            explicit Impl(const base::SpaceInformationPtr &si) : si(si)
            {
            }

            Key key(const base::State *state, double resolution) const
            {
                const auto *values = state->as<base::RealVectorStateSpace::StateType>()->values;
                Key result(dimension);
                for (unsigned int i = 0; i < dimension; ++i)
                    result[i] = static_cast<std::int64_t>(std::floor(values[i] / resolution));
                return result;
            }

            void reset()
            {
                boundaries.clear();
                boundaryCells.clear();
                occupancy.clear();
                boundarySamples = 0;
                densityRejections = 0;
                gridRebuilds = 0;
                densityCost = base::Cost(std::numeric_limits<double>::infinity());
                initialized = false;
            }

            bool terminated() const
            {
                return termination && (*termination)();
            }

            void scout(const BITstar::VertexPtrVector &states)
            {
                if (states.empty())
                    return;
                const auto *space = si->getStateSpace()->as<base::RealVectorStateSpace>();
                const auto &bounds = space->getBounds();
                base::ScopedState<> endpoint(si), candidate(si), lastValid(si);
                std::vector<double> direction(dimension);
                const double angleOffset = rng.uniformReal(0.0, 2.0 * std::acos(-1.0));
                const double step = space->getLongestValidSegmentLength() / space->getValidSegmentCountFactor();
                for (unsigned int ray = 0; ray < rays && !terminated(); ++ray)
                {
                    const auto *origin = states[rng.uniformInt(0, static_cast<int>(states.size() - 1))]->state();
                    if (dimension == 2)
                    {
                        const double angle = angleOffset + 2.0 * std::acos(-1.0) * ray / rays;
                        direction[0] = std::cos(angle);
                        direction[1] = std::sin(angle);
                    }
                    else
                    {
                        double squaredNorm = 0.0;
                        for (double &coordinate : direction)
                        {
                            coordinate = rng.gaussian01();
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
                    for (unsigned int i = 0; i < dimension; ++i)
                    {
                        if (direction[i] > 0.0)
                            length = std::min(length, (bounds.high[i] - values[i]) / direction[i]);
                        else if (direction[i] < 0.0)
                            length = std::min(length, (bounds.low[i] - values[i]) / direction[i]);
                    }
                    if (!(length > 0.0))
                        continue;
                    for (unsigned int i = 0; i < dimension; ++i)
                        endpoint[i] = values[i] + length * direction[i];
                    space->enforceBounds(endpoint.get());
                    const unsigned int segments = std::max(1u, static_cast<unsigned int>(std::ceil(length / step)));
                    double previous = 0.0;
                    si->copyState(lastValid.get(), origin);
                    for (unsigned int segment = 1; segment <= segments && !terminated(); ++segment)
                    {
                        double upper = static_cast<double>(segment) / segments;
                        space->interpolate(origin, endpoint.get(), upper, candidate.get());
                        if (si->isValid(candidate.get()))
                        {
                            previous = upper;
                            si->copyState(lastValid.get(), candidate.get());
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
                            if (si->isValid(candidate.get()))
                            {
                                lower = middle;
                                si->copyState(lastValid.get(), candidate.get());
                            }
                            else
                                upper = middle;
                        }
                        if (boundaryCells.insert(key(lastValid.get(), boundaryResolution)).second)
                            boundaries.emplace_back(si->getStateSpace(), lastValid.get());
                        break;
                    }
                }
            }

            class Sampler : public base::InformedSampler
            {
            public:
                Sampler(const base::ProblemDefinitionPtr &problem, unsigned int maxCalls, Impl &impl,
                        const base::InformedSamplerPtr &uniform)
                  : base::InformedSampler(problem, maxCalls), impl(impl), uniform(uniform)
                {
                }

                bool sampleUniform(base::State *state, const base::Cost &maxCost) override
                {
                    return sampleUniform(state, opt_->identityCost(), maxCost);
                }

                bool sampleUniform(base::State *state, const base::Cost &minCost, const base::Cost &maxCost) override
                {
                    if (impl.terminated())
                        return false;
                    const double bias = std::isfinite(maxCost.value()) ? impl.optimizationBias : impl.explorationBias;
                    const bool boundary = !impl.boundaries.empty() && impl.rng.uniform01() < bias;
                    if (boundary)
                    {
                        const auto &mean =
                            impl.boundaries[impl.rng.uniformInt(0, static_cast<int>(impl.boundaries.size() - 1))];
                        auto *values = state->as<base::RealVectorStateSpace::StateType>()->values;
                        for (unsigned int i = 0; i < impl.dimension; ++i)
                            values[i] = impl.rng.gaussian(mean[i], impl.standardDeviation);
                        if (!space_->satisfiesBounds(state))
                            return false;
                        const auto cost = uniform->heuristicSolnCost(state);
                        if (opt_->isCostBetterThan(cost, minCost) || !opt_->isCostBetterThan(cost, maxCost))
                            return false;
                    }
                    else if (!uniform->sampleUniform(state, minCost, maxCost))
                        return false;

                    // Count only valid admitted states, never rejected or colliding proposals.
                    if (!space_->satisfiesBounds(state) || !impl.si->isValid(state) || impl.terminated())
                        return false;
                    const auto cell = impl.key(state, impl.gridResolution);
                    const auto found = impl.occupancy.find(cell);
                    const std::size_t count = found == impl.occupancy.end() ? 0u : found->second;
                    const unsigned int capacity = boundary ? impl.boundaryCapacity : impl.openCapacity;
                    const double excess = std::max(0.0, static_cast<double>(count) - capacity + 1.0);
                    const double probability = 1.0 / (1.0 + 2.0 * excess);
                    if (impl.rng.uniform01() >= probability)
                    {
                        ++impl.densityRejections;
                        return false;
                    }
                    ++impl.occupancy[cell];
                    if (boundary)
                        ++impl.boundarySamples;
                    return true;
                }

                bool hasInformedMeasure() const override
                {
                    return uniform->hasInformedMeasure();
                }
                double getInformedMeasure(const base::Cost &cost) const override
                {
                    return uniform->getInformedMeasure(cost);
                }
                double getInformedMeasure(const base::Cost &minCost, const base::Cost &maxCost) const override
                {
                    return uniform->getInformedMeasure(minCost, maxCost);
                }
                base::Cost heuristicSolnCost(const base::State *state) const override
                {
                    return uniform->heuristicSolnCost(state);
                }

            private:
                Impl &impl;
                base::InformedSamplerPtr uniform;
            };

            base::SpaceInformationPtr si;
            std::shared_ptr<const base::PlannerTerminationCondition> termination;
            unsigned int dimension{0};
            double gridResolution{15.0};
            double boundaryResolution{2.0};
            double standardDeviation{10.0};
            double explorationBias{0.2};
            double optimizationBias{0.5};
            unsigned int openCapacity{1};
            unsigned int boundaryCapacity{3};
            unsigned int rays{18};
            RNG rng;
            std::vector<base::ScopedState<>> boundaries;
            std::unordered_set<Key, Hash> boundaryCells;
            std::unordered_map<Key, std::size_t, Hash> occupancy;
            base::Cost densityCost{std::numeric_limits<double>::infinity()};
            bool initialized{false};
            std::size_t boundarySamples{0};
            std::size_t densityRejections{0};
            std::size_t gridRebuilds{0};
        };

        DDLBITstar::DDLBITstar(const base::SpaceInformationPtr &si) : BITstar(si, "DDLBITstar"), impl_(new Impl(si))
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
            const double resolution = std::min(impl_->gridResolution, impl_->boundaryResolution);
            for (unsigned int i = 0; i < space->getDimension(); ++i)
            {
                if (!std::isfinite(bounds.low[i]) || !std::isfinite(bounds.high[i]) ||
                    bounds.low[i] >= bounds.high[i] || std::abs(bounds.low[i] / resolution) >= std::ldexp(1.0, 62) ||
                    std::abs(bounds.high[i] / resolution) >= std::ldexp(1.0, 62))
                    throw Exception("DDLBITstar requires finite bounds representable at its grid resolutions");
            }
            if (!std::isfinite(space->getMaximumExtent()))
                throw Exception("DDLBITstar requires a finite maximum extent");
            impl_->dimension = space->getDimension();
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
            impl_->termination = std::make_shared<base::PlannerTerminationCondition>(terminationCondition);
            struct ResetTermination
            {
                Impl &impl;
                ~ResetTermination()
                {
                    impl.termination.reset();
                }
            } resetTermination{*impl_};
            return BITstar::solve(terminationCondition);
        }

        base::InformedSamplerPtr DDLBITstar::allocInformedStateSampler(const base::ProblemDefinitionPtr &problem,
                                                                       unsigned int maxCalls)
        {
            // Each outer BIT* trial gets one bounded proposal; BIT* owns the batch attempt budget.
            auto uniform = BITstar::allocInformedStateSampler(problem, 1u);
            return std::make_shared<Impl::Sampler>(problem, maxCalls, *impl_, uniform);
        }

        void DDLBITstar::onNewBatch(const ImplicitGraph &graph)
        {
            const auto states = graph.getCopyOfSamples();
            if (!impl_->initialized ||
                pdef_->getOptimizationObjective()->isCostBetterThan(bestCost(), impl_->densityCost))
            {
                impl_->occupancy.clear();
                for (const auto &state : states)
                    ++impl_->occupancy[impl_->key(state->state(), impl_->gridResolution)];
                if (impl_->initialized)
                    ++impl_->gridRebuilds;
                impl_->densityCost = bestCost();
                impl_->initialized = true;
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
            void positive(double value)
            {
                if (!std::isfinite(value) || value <= 0.0)
                    throw Exception("DDLBITstar spatial parameters must be finite and positive");
            }
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
            impl_->gridResolution = value;
        }
        double DDLBITstar::getGridResolution() const
        {
            return impl_->gridResolution;
        }
        void DDLBITstar::setBoundaryResolution(double value)
        {
            checkConfigurationChange();
            positive(value);
            impl_->boundaryResolution = value;
        }
        double DDLBITstar::getBoundaryResolution() const
        {
            return impl_->boundaryResolution;
        }
        void DDLBITstar::setBoundaryStandardDeviation(double value)
        {
            checkConfigurationChange();
            positive(value);
            impl_->standardDeviation = value;
        }
        double DDLBITstar::getBoundaryStandardDeviation() const
        {
            return impl_->standardDeviation;
        }
        void DDLBITstar::setExplorationBias(double value)
        {
            checkConfigurationChange();
            probability(value);
            impl_->explorationBias = value;
        }
        double DDLBITstar::getExplorationBias() const
        {
            return impl_->explorationBias;
        }
        void DDLBITstar::setOptimizationBias(double value)
        {
            checkConfigurationChange();
            probability(value);
            impl_->optimizationBias = value;
        }
        double DDLBITstar::getOptimizationBias() const
        {
            return impl_->optimizationBias;
        }
        void DDLBITstar::setOpenCellCapacity(unsigned int value)
        {
            checkConfigurationChange();
            if (value == 0 || value >= impl_->boundaryCapacity)
                throw Exception("DDLBITstar open capacity must be positive and below boundary capacity");
            impl_->openCapacity = value;
        }
        unsigned int DDLBITstar::getOpenCellCapacity() const
        {
            return impl_->openCapacity;
        }
        void DDLBITstar::setBoundaryCellCapacity(unsigned int value)
        {
            checkConfigurationChange();
            if (value <= impl_->openCapacity)
                throw Exception("DDLBITstar boundary capacity must exceed open capacity");
            impl_->boundaryCapacity = value;
        }
        unsigned int DDLBITstar::getBoundaryCellCapacity() const
        {
            return impl_->boundaryCapacity;
        }
        void DDLBITstar::setScoutingRays(unsigned int value)
        {
            checkConfigurationChange();
            if (value == 0)
                throw Exception("DDLBITstar requires at least one scouting ray");
            impl_->rays = value;
        }
        unsigned int DDLBITstar::getScoutingRays() const
        {
            return impl_->rays;
        }
        std::size_t DDLBITstar::numBoundaryPoints() const
        {
            return impl_->boundaries.size();
        }
        std::size_t DDLBITstar::numBoundarySamples() const
        {
            return impl_->boundarySamples;
        }
        std::size_t DDLBITstar::numDensityRejections() const
        {
            return impl_->densityRejections;
        }
        std::size_t DDLBITstar::numGridRebuilds() const
        {
            return impl_->gridRebuilds;
        }
    }  // namespace geometric
}  // namespace ompl
