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

#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/PDF.h"

#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

#include <boost/math/constants/constants.hpp>

namespace ompl
{
    namespace magic
    {
        static const unsigned int ATLAS_STATE_SPACE_SAMPLES = 50;
        static const double ATLAS_STATE_SPACE_EPSILON = 0.05;
        static const double ATLAS_STATE_SPACE_RHO_MULTIPLIER = 5;
        static const double ATLAS_STATE_SPACE_ALPHA = boost::math::constants::pi<double>() / 8.0;
        static const double ATLAS_STATE_SPACE_EXPLORATION = 0.75;
        static const unsigned int ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION = 200;
        static const double ATLAS_STATE_SPACE_BACKOFF = 0.75;
    }

    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::AtlasChart */
        OMPL_CLASS_FORWARD(AtlasChart);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::AtlasStateSpace */
        OMPL_CLASS_FORWARD(AtlasStateSpace);
        /// @endcond

        /** \brief StateSampler for use on an atlas. */
        class AtlasStateSampler : public StateSampler
        {
        public:
            AtlasStateSampler(const AtlasStateSpace *space);

            /** \brief Sample a state uniformly from the charted regions of the
             * manifold. Return sample in \a state. */
            void sampleUniform(State *state) override;

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance. Return sample in \a state. \note
             * rho_s_ is a good choice for \a distance. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev. Return sample in \a state. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        private:
            /** \brief Atlas on which to sample. */
            const AtlasStateSpace *atlas_;

            /** \brief Random number generator. */
            mutable RNG rng_;
        };

        /**
           @anchor gAtlas
           @par Short Description
           AtlasStateSpace implements an atlas-based methodology for constrained sampling-based planning,
           where the underlying constraint manifold is locally parameterized by \e charts (AtlasChart). The underlying
           constraint manifold can then be sampled and explored using the collection of these charts (an \e atlas).

           @par External Documentation
           This state space is inspired by the work on AtlasRRT. One reference of which is the following:

           L. Jaillet and J. M. Porta, "Path Planning Under Kinematic Constraints by Rapidly Exploring Manifolds," in
           IEEE Transactions on Robotics, vol. 29, no. 1, pp. 105-117, Feb. 2013. DOI: <a
           href="http://dx.doi.org/10.1109/TRO.2012.2222272">10.1109/TRO.2012.2222272</a>.

           For more information on constrained sampling-based planning using atlas-based methods, see the following
           review paper. The section on atlas-based methods cites most of the relevant literature.

           Z. Kingston, M. Moll, and L. E. Kavraki, “Sampling-Based Methods for Motion Planning with Constraints,”
           Annual Review of Control, Robotics, and Autonomous Systems, 2018. DOI:
           <a href="http://dx.doi.org/10.1146/annurev-control-060117-105226">10.1146/annurev-control-060117-105226</a>
           <a href="http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf">[PDF]</a>.
        */

        /** \brief ConstrainedStateSpace encapsulating a planner-agnostic atlas
         * algorithm for planning on a constraint manifold. */
        class AtlasStateSpace : public ConstrainedStateSpace
        {
        public:
            using AtlasChartBiasFunction = std::function<double(AtlasChart *)>;
            using NNElement = std::pair<const StateType *, std::size_t>;

            /** \brief A state in an atlas represented as a real vector in
             * ambient space and a chart that it belongs to. */
            class StateType : public ConstrainedStateSpace::StateType
            {
            public:
                /** \brief Construct state of size \a n. */
                StateType(const ConstrainedStateSpace *space) : ConstrainedStateSpace::StateType(space)
                {
                }

                /** \brief Get the chart this state is on. */
                AtlasChart *getChart() const
                {
                    return chart_;
                }

                /** \brief Set the chart \a c for the state. */
                void setChart(AtlasChart *c) const
                {
                    chart_ = c;
                }

            private:
                /** \brief Chart owning the state. */
                mutable AtlasChart *chart_{nullptr};
            };

            /** \brief Construct an atlas with the specified dimensions. */
            AtlasStateSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint, bool separate = true);

            /** \brief Destructor. */
            ~AtlasStateSpace() override;

            /** \brief Reset the space (except for anchor charts). */
            void clear() override;

            /** \brief Allocate the default state sampler for this space. */
            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<AtlasStateSampler>(this);
            }

            /** \brief Allocate the previously set state sampler for this space. */
            StateSamplerPtr allocStateSampler() const override
            {
                return std::make_shared<AtlasStateSampler>(this);
            }

            void copyState(State *destination, const State *source) const override
            {
                ConstrainedStateSpace::copyState(destination, source);
                destination->as<StateType>()->setChart(source->as<StateType>()->getChart());
            }

            /** \brief Allocate a new state in this space. */
            State *allocState() const override
            {
                return new StateType(this);
            }

            /** @name Getters and Setters
                @{ */

            /** \brief Set \a epsilon, the maximum permissible distance between
             * a point in the validity region of a chart and its projection onto
             * the manifold. Default 0.1. */
            void setEpsilon(double epsilon)
            {
                if (epsilon <= 0)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setEpsilon(): "
                                          "epsilon must be positive.");
                epsilon_ = epsilon;
            }

            /** \brief Set \a rho, the maximum radius for which a chart is
             * valid. Default 0.1. */
            void setRho(double rho)
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
            void setAlpha(double alpha)
            {
                if (alpha <= 0 || alpha >= boost::math::constants::pi<double>() / 2.)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setAlpha(): "
                                          "alpha must be in (0, pi/2).");
                cos_alpha_ = std::cos(alpha);
            }

            /** \brief Set the \a exploration parameter, which tunes the balance
             * of refinement (sampling within known regions) and exploration
             * (sampling on the frontier). Valid values are in the range [0,1),
             * where 0 is all refinement, and 1 is all exploration. Default
             * 0.5. */
            void setExploration(double exploration)
            {
                if (exploration >= 1)
                    throw ompl::Exception("ompl::base::AtlasStateSpace::setExploration(): "
                                          "exploration must be in [0, 1).");
                exploration_ = exploration;

                // Update sampling radius
                setRho(rho_);
            }

            /** \brief Sometimes manifold traversal creates many charts. This
             * parameter limits the number of charts that can be created during
             * one traversal. Default 200. */
            void setMaxChartsPerExtension(unsigned int charts)
            {
                maxChartsPerExtension_ = charts;
            }

            /** \brief Sets bias function for sampling. */
            void setBiasFunction(const AtlasChartBiasFunction &biasFunction)
            {
                biasFunction_ = biasFunction;
            }

            /** \brief Sets whether the atlas should separate charts or not. */
            void setSeparated(bool separate)
            {
                separate_ = separate;
            }

            /** \brief Sets the backoff factor in manifold traversal factor. */
            void setBackoff(double backoff)
            {
                backoff_ = backoff;
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

            /** \brief Returns whether the atlas is separating charts or not. */
            bool isSeparated() const
            {
                return separate_;
            }

            /** \brief Return the number of charts currently in the atlas. */
            std::size_t getChartCount() const
            {
                return charts_.size();
            }

            /** \brief Returns the current backoff factor used in manifold traversal. */
            double getBackoff() const
            {
                return backoff_;
            }

            /** @} */

            /** @name Atlas Chart Management
                @{ */

            /** \brief Create a new chart for the atlas, centered at \a xorigin,
             * which should be on the manifold. Returns nullptr upon failure. */
            AtlasChart *newChart(const StateType *state) const;

            /** \brief Pick a chart at random. */
            AtlasChart *sampleChart() const;

            /** \brief Find the chart to which \a x belongs. Returns nullptr if
             * no chart found. Assumes \a x is already on the manifold. */
            AtlasChart *owningChart(const StateType *state) const;

            /** \brief Wrapper to return chart \a state belongs to. Will attempt
             * to initialize new chart if \a state does not belong to one. If \a
             * force is true, this routine will reinitialize the chart that the
             * state should be on. If \a created is not null, it will be set to
             * true if a new chart is created. */
            AtlasChart *getChart(const StateType *state, bool force = false, bool *created = nullptr) const;

            /** @} */

            /** @name Constrained Planning
                @{ */

            /** \brief Wrapper for newChart(). Charts created this way will
             * persist through calls to clear().
             * \throws ompl::Exception if manifold seems degenerate here. */
            AtlasChart *anchorChart(const State *state) const;

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a geodesic
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a geodesic.*/
            bool discreteGeodesic(const State *from, const State *to, bool interpolate = false,
                                  std::vector<State *> *geodesic = nullptr) const override;

            /** @} */

            /** @name Statistics and Visualization
                @{ */

            /** \brief Estimate what percentage of atlas charts do not have
             * fully formed polytope boundaries, and are therefore on the
             * frontier. */
            double estimateFrontierPercent() const;

            /** \brief Write a mesh representation of the atlas to a stream. */
            void printPLY(std::ostream &out) const;

            /** @} */

        protected:
            /** \brief Set of states on which there are anchored charts. */
            mutable std::vector<StateType *> anchors_;

            /** \brief Set of charts. */
            mutable std::vector<AtlasChart *> charts_;

            /** \brief PDF of charts according to a bias function. */
            mutable PDF<AtlasChart *> chartPDF_;

            /** \brief Set of chart centers and indices, accessible by
             * nearest-neighbor queries to the chart centers. */
            mutable NearestNeighborsGNAT<NNElement> chartNN_;

            /** @name Tunable Parameters
                @{ */

            /** \brief Maximum distance between a chart and the manifold inside its validity region. */
            double epsilon_{ompl::magic::ATLAS_STATE_SPACE_EPSILON};

            /** \brief Maximum radius of chart validity region. */
            double rho_;

            /** \brief Cosine of the maximum angle between a chart and the manifold inside its validity region. */
            double cos_alpha_;

            /** \brief Balance between explorationa and refinement. */
            double exploration_;

            /** \brief Sampling radius within a chart. Inferred from rho and exploration parameters. */
            mutable double rho_s_;

            /** \brief Step size reduction factor during manifold traversal. */
            double backoff_{ompl::magic::ATLAS_STATE_SPACE_BACKOFF};

            /** \brief Maximum number of charts that can be created in one manifold traversal. */
            unsigned int maxChartsPerExtension_{ompl::magic::ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION};

            /** @} */

            /** \brief Function to bias chart sampling */
            AtlasChartBiasFunction biasFunction_;

            /** \brief Enable or disable halfspace separation of the charts. */
            bool separate_;

            /** \brief Random number generator. */
            mutable RNG rng_;
        };
    }
}

#endif
