/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Zachary Kingston, Caleb Voss */

#ifndef OMPL_BASE_SPACES_ATLAS_STATE_SPACE_
#define OMPL_BASE_SPACES_ATLAS_STATE_SPACE_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/PDF.h"

#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

#include <eigen3/Eigen/Core>

namespace ompl
{
    namespace magic
    {
        static const unsigned int ATLAS_STATE_SAMPLER_TRIES = 100;
        static const double ATLAS_STATE_SPACE_EPSILON = 0.2;
        static const double ATLAS_STATE_SPACE_LAMBDA = 2.0;
        static const double ATLAS_STATE_SPACE_RHO = 0.5;
        static const double ATLAS_STATE_SPACE_ALPHA = M_PI / 8.0;
        static const double ATLAS_STATE_SPACE_EXPLORATION = 0.5;
        static const unsigned int ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION = 200;
    }

    namespace base
    {
        class AtlasChart;
        class AtlasStateSpace;

        typedef std::function<double(AtlasChart *)> AtlasChartBiasFunction;

        /** \brief StateSampler for use on an atlas. */
        class AtlasStateSampler : public StateSampler
        {
        public:
            AtlasStateSampler(const AtlasStateSpace &space);

            /** \brief Sample a state uniformly from the charted regions of the
             * manifold. Return sample in \a state. */
            virtual void sampleUniform(State *state) override;

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            virtual void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev. Return sample in \a state. */
            virtual void sampleGaussian(State *state, const State *mean, double stdDev) override;

        private:
            /** \brief Atlas on which to sample. */
            const AtlasStateSpace &atlas_;

            /** \brief Random number generator. */
            mutable RNG rng_;
        };

        /** \brief ValidStateSampler for use on an atlas. */
        class AtlasValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Create a valid state sampler for the specifed space
             * information \a si. */
            AtlasValidStateSampler(const SpaceInformation *si);

            /** \brief Sample a valid state uniformly from the charted regions
             * of the manifold. Return sample in \a state. */
            virtual bool sample(State *state) override;

            /** \brief Sample a valid state uniformly from the ball with center
             * \a near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            virtual bool sampleNear(State *state, const State *near, double distance) override;

        private:
            /** \brief Underlying ordinary atlas state sampler. */
            AtlasStateSampler sampler_;
        };

        /** \brief State space encapsulating a planner-agnostic atlas algorithm
         * for planning on a constraint manifold. */
        class AtlasStateSpace : public ConstrainedStateSpace
        {
        public:
            /** \brief A state in an atlas represented as a real vector in
             * ambient space and a chart that it belongs to. */
            class StateType : public ConstrainedStateSpace::StateType
            {
            public:
                /** \brief Construct state of size \a n. */
                StateType(State *state, const unsigned int n) : ConstrainedStateSpace::StateType(state, n)
                {
                }

                /** \brief Get the chart this state is on. */
                AtlasChart *getChart(void) const
                {
                    return chart_;
                }

                /** \brief Set the chart \a c for the state. */
                void setChart(AtlasChart *c) const
                {
                    chart_ = c;
                }

            private:
                /** \brief Chart owning the real vector. */
                mutable AtlasChart *chart_ = nullptr;
            };

            // Store an AtlasChart's xorigin and index in the charts_ array.
            typedef std::pair<const Eigen::VectorXd *, std::size_t> NNElement;

            /** \brief Construct an atlas with the specified dimensions. */
            AtlasStateSpace(const StateSpacePtr ambientSpace, const ConstraintPtr constraint, bool lazy = false, bool bias = false, bool separate = true);

            /** \brief Destructor. */
            virtual ~AtlasStateSpace();

            /** @name Setup and tuning of atlas parameters
             * @{ */

            /** \brief Check that the space referred to by the space information
             * \a si is, in fact, an AtlasStateSpace. */
            static void checkSpace(const SpaceInformation *si);

            /** \brief Reset the space (except for anchor charts). */
            void clear();

            /** \brief Set \a epsilon, the maximum permissible distance between
             * a point in the validity region of a chart and its projection onto
             * the manifold. Default 0.1. */
            void setEpsilon(const double epsilon)
            {
                if (epsilon <= 0)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setEpsilon(): "
                                          "epsilon must be positive.");
                epsilon_ = epsilon;
            }

            /** \brief Set \a rho, the maximum radius for which a chart is
             * valid. Default 0.1. */
            void setRho(const double rho)
            {
                if (rho <= 0)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setRho(): "
                                          "rho must be positive.");
                rho_ = rho;
                rho_s_ = rho_ / std::pow(1 - exploration_, 1.0 / k_);
            }

            /** \brief Set \a alpha, the maximum permissible angle between the
             * chart and the manifold inside the validity region of the
             * chart. Must be within the range (0, pi/2). Default pi/16. */
            void setAlpha(const double alpha)
            {
                if (alpha <= 0 || alpha >= M_PI_2)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setAlpha(): "
                                          "alpha must be in (0, pi/2).");
                cos_alpha_ = std::cos(alpha);
            }

            /** \brief Set the \a exploration parameter, which tunes the balance
             * of refinement (sampling within known regions) and exploration
             * (sampling on the frontier). Valid values are in the range [0,1),
             * where 0 is all refinement, and 1 is all exploration. Default
             * 0.5. */
            void setExploration(const double exploration)
            {
                if (exploration >= 1)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setExploration(): "
                                          "exploration must be in [0, 1).");
                exploration_ = exploration;

                // Update sampling radius
                setRho(rho_);
            }

            /** \brief Set \a lambda, where lambda * ||x-y|| is the maximum
             * distance that can be accumulated while traversing the manifold
             * from x to y before the algorithm stops. Must be greater than 1.
             * Default 2. */
            void setLambda(const double lambda)
            {
                if (lambda <= 1)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setLambda(): "
                                          "lambda must be > 1.");
                lambda_ = lambda;
            }

            /** \brief Sometimes manifold traversal creates many charts. This
             * parameter limits the number of charts that can be created during
             * one traversal. Default 200. */
            void setMaxChartsPerExtension(const unsigned int charts)
            {
                maxChartsPerExtension_ = charts;
            }

            bool getLazy() const
            {
                return lazy_;
            }

            /** \brief Get epsilon. */
            double getEpsilon() const
            {
                return epsilon_;
            }

            /** \brief Get rho. */
            double getRho() const
            {
                return rho_;
            }

            /** \brief Get alpha. */
            double getAlpha() const
            {
                return std::acos(cos_alpha_);
            }

            /** \brief Get the exploration parameter. */
            double getExploration() const
            {
                return exploration_;
            }

            /** \brief Get lambda. */
            double getLambda() const
            {
                return lambda_;
            }

            /** \brief Get the sampling radius. */
            double getRho_s() const
            {
                return rho_s_;
            }

            /** \brief Get the maximum number of charts to create in one pass. */
            unsigned int getMaxChartsPerExtension() const
            {
                return maxChartsPerExtension_;
            }
            /** @} */

            /** @name Manifold and chart operations
             *  @{ */

            /** \brief Wrapper for newChart(). Charts created this way will
             * persist through calls to clear().
             * \throws ompl::Exception if manifold seems degenerate here. */
            AtlasChart *anchorChart(const Eigen::VectorXd &xorigin) const;

            /** \brief Create a new chart for the atlas, centered at \a xorigin,
             * which should be on the manifold. Returns nullptr upon failure. */
            AtlasChart *newChart(const Eigen::VectorXd &xorigin) const;

            /** \brief Pick a chart at random. */
            AtlasChart *sampleChart(void) const;

            /** \brief Find the chart to which \a x belongs. Returns nullptr if
             * no chart found. Assumes \a x is already on the manifold. */
            AtlasChart *owningChart(const Eigen::VectorXd &x) const;

            /** \brief Wrapper to return chart \a state belongs to. Will attempt
             * to initialize new chart if \a state does not belong to one. */
            AtlasChart *getChart(const StateType *state, bool force = false, bool *created = nullptr) const;

            /** \brief Return the number of charts currently in the atlas. */
            std::size_t getChartCount(void) const;

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a stateList
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a stateList. */
            bool traverseManifold(const State *from, const State *to, bool interpolate = false,
                                  std::vector<State *> *stateList = nullptr) const;


            State *piecewiseInterpolate(const std::vector<State *> &stateList, double t) const override;
            /** @} */

            /** @name Interpolation and state management
             * @{ */

            /** \brief Return an instance of the AtlasStateSampler. */
            StateSamplerPtr allocDefaultStateSampler() const
            {
                return StateSamplerPtr(new AtlasStateSampler(*this));
            }

            void copyState(State *destination, const State *source) const
            {
                ConstrainedStateSpace::copyState(destination, source);
                destination->as<StateType>()->setChart(source->as<StateType>()->getChart());
            }

            /** \brief Allocate a new state in this space. */
            State *allocState() const
            {
                StateType *state = new StateType(space_->allocState(), n_);
                state->setValues(space_->getValueAddressAtIndex(state->getState(), 0));
                return state;
            }

            /** @} */

            /** @name Visualization and debug
             * @{ */

            /** \brief Estimate what percentage of atlas charts do not have
             * fully formed polytope boundaries, and are therefore on the
             * frontier. */
            double estimateFrontierPercent() const;

            /** \brief Write a mesh representation of the atlas to a stream. */
            void printPLY(std::ostream &out) const;

            void setBiasFunction(const AtlasChartBiasFunction &biasFunction)
            {
                biasFunction_ = biasFunction;
            }

            /** @} */

        protected:
            /** \brief Set of charts, sampleable by weight. */
            mutable std::vector<AtlasChart *> charts_;

            /** \brief Set of chart centers and indices, accessible by
             * nearest-neighbor queries to the chart centers. */
            mutable NearestNeighborsGNAT<NNElement> chartNN_;

            mutable PDF<AtlasChart *> chartPDF_;

        private:
            /** \brief Maximum distance between a chart and the manifold inside its validity region. */
            double epsilon_;

            /** \brief Maximum radius of chart validity region. */
            double rho_;

            /** \brief Cosine of the maximum angle between a chart and the manifold inside its validity region. */
            double cos_alpha_;

            /** \brief Balance between explorationa and refinement. */
            double exploration_;

            /** \brief Manifold traversal from x to y is stopped if accumulated distance is greater than d(x,y) times
             * this. */
            double lambda_;

            /** \brief Enable or disable halfspace separation of the charts. */
            const bool lazy_;

            const bool bias_;

            const bool separate_;

            /** \brief Sampling radius within a chart. Inferred from rho and exploration parameters. */
            mutable double rho_s_;

            /** \brief Maximum number of charts that can be created in one manifold traversal. */
            unsigned int maxChartsPerExtension_;

            /** \brief Random number generator. */
            mutable RNG rng_;

            AtlasChartBiasFunction biasFunction_;
        };
    }
}

#endif
