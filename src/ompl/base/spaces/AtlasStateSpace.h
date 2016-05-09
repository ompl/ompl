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

/* Author: Caleb Voss */

#ifndef OMPL_BASE_SPACES_ATLAS_STATE_SPACE_
#define OMPL_BASE_SPACES_ATLAS_STATE_SPACE_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/geometric/PathGeometric.h"

#include <eigen3/Eigen/Core>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::AtlasChart and
         * ompl::base::AtlasStateSpace. */
        class AtlasChart;
        OMPL_CLASS_FORWARD(AtlasStateSpace);
        /// @endcond
        
        /** \class ompl::base::AtlasStateSpacePtr
         * \brief A boost shared pointer wrapper for
         * ompl::base::AtlasStateSpace. */
        
        /** \brief StateSampler for use on an atlas. */
        class AtlasStateSampler : public StateSampler
        {
        public:
            
            /** \brief Create a sampler for the specified space information.
             * \note The underlying state space must be an AtlasStateSpace. */
            AtlasStateSampler (const SpaceInformation *si);
            
            /** \brief Create a sampler for the specified \a atlas space. */
            AtlasStateSampler (const AtlasStateSpace &atlas);

            /** \brief Sample a state uniformly from the charted regions of the
             * manifold. Return sample in \a state. */
            virtual void sampleUniform (State *state);
            
            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            virtual void sampleUniformNear (State *state, const State *near,
                                            const double distance);
            
            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev. Return sample in \a state. */
            virtual void sampleGaussian (State *state, const State *mean,
                                         const double stdDev);
            
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
            AtlasValidStateSampler (const SpaceInformation *si);
            
            /** \brief Sample a valid state uniformly from the charted regions
             * of the manifold. Return sample in \a state. */
            virtual bool sample (State *state);
            
            /** \brief Sample a valid state uniformly from the ball with center
             * \a near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            virtual bool sampleNear (State *state, const State *near,
                                     const double distance);
            
        private:
            
            /** \brief Underlying ordinary atlas state sampler. */
            AtlasStateSampler sampler_;
        };
        
        /** \brief Atlas-specific implementation of checkMotion(). */
        class AtlasMotionValidator : public MotionValidator
        {
        public:
            
            /** \brief Constructor. */
            AtlasMotionValidator (SpaceInformation *si);
            
            /** \brief Constructor. */
            AtlasMotionValidator (const SpaceInformationPtr &si);
            
            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. */
            bool checkMotion (const State *s1, const State *s2) const;
            
            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. If not, return the last valid state
             * and its interpolation parameter in \a lastValid.
             * \note The interpolation parameter will not likely reproduce the
             * last valid state if used in interpolation since the distance
             * between the last valid state and \a s2 is estimated using the
             * ambient metric. */
            bool checkMotion (const State *s1, const State *s2,
                              std::pair<State *, double> &lastValid) const;
            
        private:
            
            /** \brief Atlas on which we check motion. */
            const AtlasStateSpace &atlas_;
        };
        
        /** \brief State space encapsulating a planner-agnostic atlas algorithm
         * for planning on a constraint manifold. */
        class AtlasStateSpace : public RealVectorStateSpace
        {
        public:
            
            /** \brief A state in an atlas represented as a real vector in
             * ambient space and a chart that it belongs to. */
            class StateType : public RealVectorStateSpace::StateType
            {
            public:
                
                /** \brief Construct state of size \a dimension. */
                StateType (const unsigned int &dimension);
                
                /** \brief Destructor. */
                virtual ~StateType(void);

                /** \brief Set this state to be identical to \a source.
                 * \note Assumes source has the same size as this state. */
                void copyFrom (const StateType *source);
                
                /** \brief Set this state to \a x and make it belong to \a c.
                 * \note Assumes \a x has the same size as the state. */
                void setRealState (const Eigen::VectorXd &x, AtlasChart *c);
                
                /** \brief View this state as a vector. */
                Eigen::Map<Eigen::VectorXd> vectorView (void) const;
                
                /** \brief View this state as a const vector. */
                Eigen::Map<const Eigen::VectorXd> constVectorView (void) const;
                
                /** \brief Get the chart this state is on. */
                AtlasChart *getChart (void) const;
                
                /** \brief Set the chart \a c for the state. */
                void setChart (AtlasChart *c) const;

            private:
                
                /** \brief Dimension of the real vector. */
                const unsigned int &dimension_;

                /** \brief Chart owning the real vector. */
                mutable AtlasChart *chart_ = nullptr;
            };

            // Store an AtlasChart's xorigin and index in the charts_ array.
            typedef std::pair<const Eigen::VectorXd *, std::size_t> NNElement;
            
            /** \brief Construct an atlas with the specified dimensions. */
            AtlasStateSpace (const unsigned int ambientDimension,
                             const unsigned int manifoldDimension);
            
            /** \brief Destructor. */
            virtual ~AtlasStateSpace (void);
            
            /** @name Setup and tuning of atlas parameters
             * @{ */

            /** \brief Compute the constraint function at \a x. Result is
             * returned in \a out, which should be allocated to size n_. */
            virtual void constraintFunction (
                const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const = 0;
            
            /** \brief Compute the Jacobian of the constraint function at \a
             * x. Result is returned in \a out, which should be allocated to
             * size (n_-k_) by n_. Default implementation performs the
             * differentiation numerically, which may be slower and/or
             * more inaccurate than an explicit formula. */
            virtual void jacobianFunction (
                const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const;
            
            /** \brief Behave exactly like the underlying RealVectorStateSpace
             * for all overridden functions. */
            void stopBeingAnAtlas (const bool yes);
            
            /** \brief Final setup for the space. */
            void setup (void);

            /** \brief Check that the space referred to by the space information
             * \a si is, in fact, an AtlasStateSpace. */
            static void checkSpace (const SpaceInformation *si);

            /** \brief Reset the space (except for anchor charts). */
            void clear (void);
            
            /** \brief Associate \a si with this space. Requires that \a si was
             * constructed from this AtlasStateSpace. */
            void setSpaceInformation (const SpaceInformationPtr &si);
            
            /** \brief Set \a delta, the step size for traversing the manifold
             * and collision checking. Default 0.02. */
            void setDelta (const double delta);
            
            /** \brief Set \a epsilon, the maximum permissible distance between
             * a point in the validity region of a chart and its projection onto
             * the manifold. Default 0.1. */
            void setEpsilon (const double epsilon);
            
            /** \brief Set \a rho, the maximum radius for which a chart is
             * valid. Default 0.1. */
            void setRho (const double rho);
            
            /** \brief Set \a alpha, the maximum permissible angle between the
             * chart and the manifold inside the validity region of the
             * chart. Must be within the range (0, pi/2). Default pi/16. */
            void setAlpha (const double alpha);
            
            /** \brief Set the \a exploration parameter, which tunes the balance
             * of refinement (sampling within known regions) and exploration
             * (sampling on the frontier). Valid values are in the range [0,1),
             * where 0 is all refinement, and 1 is all exploration. Default
             * 0.5. */
            void setExploration (const double exploration);
            
            /** \brief Set \a lambda, where lambda * ||x-y|| is the maximum
             * distance that can be accumulated while traversing the manifold
             * from x to y before the algorithm stops. Must be greater than 1.
             * Default 2. */
            void setLambda (const double lambda);
            
            /** \brief Projection from a chart to the manifold will stop if the
             * norm of the error is less than \a tolerance. Default 1e-8. */
            void setProjectionTolerance (const double tolerance);
            
            /** \brief Projection from a chart to the manifold will stop after
             * at most \a iterations iterations. Default 50. */
            void setProjectionMaxIterations (const unsigned int iterations);
            
            /** \brief Sometimes manifold traversal creates many charts. This
             * parameter limits the number of charts that can be created during
             * one traversal. Default 200. */
            void setMaxChartsPerExtension (const unsigned int charts);
            
            /** \brief Get delta. */
            double getDelta (void) const;
            
            /** \brief Get epsilon. */
            double getEpsilon (void) const;
            
            /** \brief Get rho. */
            double getRho (void) const;
            
            /** \brief Get alpha. */
            double getAlpha (void) const;
            
            /** \brief Get the exploration parameter. */
            double getExploration (void) const;
            
            /** \brief Get lambda. */
            double getLambda (void) const;
            
            /** \brief Get the sampling radius. */
            double getRho_s (void) const;
            
            /** \brief Get the projection tolerance. */
            double getProjectionTolerance (void) const;
            
            /** \brief Get the maximum number of projection iterations. */
            unsigned int getProjectionMaxIterations (void) const;
            
            /** \brief Get the maximum number of charts to create in one pass. */
            unsigned int getMaxChartsPerExtension (void) const;
            
            /** \brief Get the dimension of the ambient space. */
            unsigned int getAmbientDimension (void) const;
            
            /** \brief Get the dimension of the constraint manifold. */
            unsigned int getManifoldDimension (void) const;
            
            /** @} */
            
            /** @name Manifold and chart operations
             *  @{ */
                        
            /** \brief Wrapper for newChart(). Charts created this way will
             * persist through calls to clear().
             * \throws ompl::Exception if manifold seems degenerate here. */
            AtlasChart &anchorChart (const Eigen::VectorXd &xorigin) const;
            
            /** \brief Create a new chart for the atlas, centered at \a xorigin,
             * which should be on the manifold.
             * \throws ompl::Exception if manifold seems degenerate here. */
            AtlasChart &newChart (const Eigen::VectorXd &xorigin) const;

            /** \brief Pick a chart at random. */
            AtlasChart &sampleChart (void) const;
            
            /** \brief Find the chart to which \a x belongs. Returns nullptr if
             * no chart found. Assumes \a x is already on the manifold. */
            AtlasChart *owningChart (const Eigen::VectorXd &x) const;
            
            /** Compute the distance between two charts represented by
             * nearest-neighbor elements. */
            static double chartNNDistanceFunction (const NNElement &e1, const NNElement &e2);
            
            /** \brief Search for the border of chart \a c between \a xinside,
             * which is inside the polytope of \a c, and \a xoutside, which is
             * outside. The computed point lies inside the border at a distance
             * no farther than half the distance of \a xinside to the
             * border. The output is stored to \a out, which should be allocated
             * to size n_. */
            void dichotomicSearch (const AtlasChart &c,
                                   const Eigen::VectorXd &xinside,
                                   const Eigen::VectorXd &xoutside,
                                   Eigen::Ref<Eigen::VectorXd> out) const;
            
            /** \brief Return the number of charts currently in the atlas. */
            std::size_t getChartCount (void) const;
            
            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a stateList
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a stateList. */
            bool traverseManifold (const StateType *from, const StateType *to,
                                   const bool interpolate = false,
                                   std::vector<StateType *> *stateList = nullptr) const;

            /** @} */

            /** @name Interpolation and state management
             * @{ */
            
            /** \brief Try to project the vector \a x onto the manifold, without
             * using charts. Result returned in \a x. Returns whether the
             * projection succeeded. */
            bool project (Eigen::Ref<Eigen::VectorXd> x) const;
            
            /** \brief Find the state between \a from and \a to at time \a t,
             * where \a t = 0 is \a from, and \a t = 1 is the final state
             * reached by followManifold(\a from, \a to, true, ...), which may
             * not be \a to. State returned in \a state. */
            void interpolate (const State *from, const State *to,
                              const double t, State *state) const;
            
            /** \brief Like interpolate(...), but uses the information about
             * intermediate states already supplied in \a stateList from a
             * previous call to followManifold(..., true, \a stateList). The
             * 'from' and 'to' states are the first and last elements \a
             * stateList. Assumes \a stateList contains at least two
             * elements. */
            void piecewiseInterpolate (const std::vector<StateType *> &stateList,
                                       const double t, State *state) const;
            
            /** \brief Whether interpolation is symmetric. (Yes.) */
            bool hasSymmetricInterpolate (void) const;
            
            /** \brief Copy \a source to \a destination. The memory for
             * these two states should not overlap. Assumes they are of type
             * AtlasStateSpace::StateType. */
            void copyState (State *destination, const State *source) const;
            
            /** \brief Return an instance of the AtlasStateSampler. */
            StateSamplerPtr allocDefaultStateSampler (void) const;
            
            /** \brief Allocate a new state in this space. */
            State *allocState (void) const;
            
            /** \brief Free \a state. Assumes \a state is of type
             * AtlasStateSpace::StateType.  state. */
            void freeState (State *state) const;
            
            /** @} */
            
            /** @name Visualization and debug
             * @{ */
            
            /** \brief Estimate what percentage of atlas charts do not have
             * fully formed polytope boundaries, and are therefore on the
             * frontier. */
            int estimateFrontierPercent () const;
            
            /** \brief Write a mesh representation of the atlas to a stream. */
            void dumpMesh (std::ostream &out) const;
            
            /** \brief Write a mesh of the planner graph to a stream. Insert
             * additional vertices to project the edges along the manifold if \a
             * asIs == true. */
            void dumpGraph (const PlannerData::Graph &graph, std::ostream &out,
                            const bool asIs = false) const;
            
            /** \brief Write a mesh of a path on the atlas to stream. Insert
             * additional vertices to project the edges along the manifold if \a
             * asIs == true. */
            void dumpPath (ompl::geometric::PathGeometric &path,
                           std::ostream &out, const bool asIs = false) const;
            
            /** @} */

        protected:
            
            /** \brief SpaceInformation associated with this space. */
            SpaceInformation *si_;
            
            /** \brief Set of charts, sampleable by weight. */
            mutable std::vector<AtlasChart *> charts_;
            
            /** \brief Set of chart centers and indices, accessible by
             * nearest-neighbor queries to the chart centers. */
            mutable NearestNeighborsGNAT<NNElement> chartNN_;
            
        private:
            
            /** \brief Ambient space dimension. */
            const unsigned int n_;
            
            /** \brief Manifold dimension. */
            const unsigned int k_;
            
            /** \brief Step size when traversing the manifold and collision checking. */
            double delta_;
            
            /** \brief Maximum distance between a chart and the manifold inside its validity region. */
            double epsilon_;
            
            /** \brief Maximum radius of chart validity region. */
            double rho_;
            
            /** \brief Cosine of the maximum angle between a chart and the manifold inside its validity region. */
            double cos_alpha_;
            
            /** \brief Balance between explorationa and refinement. */
            double exploration_;
            
            /** \brief Manifold traversal from x to y is stopped if accumulated distance is greater than d(x,y) times this. */
            double lambda_;
            
            /** \brief Sampling radius within a chart. Inferred from rho and exploration parameters. */
            mutable double rho_s_;
            
            /** \brief Tolerance for Newton method used in projection onto manifold. */
            double projectionTolerance_;
            
            /** \brief Maximum number of iterations for Newton method used in projection onto manifold. */
            unsigned int projectionMaxIterations_;
            
            /** \brief Maximum number of charts that can be created in one manifold traversal. */
            unsigned int maxChartsPerExtension_;
            
            /** \brief Whether setup() has been called. */
            bool setup_;
            
            /** \brief Whether we are not being an atlas. */
            bool noAtlas_;
            
            /** \brief Random number generator. */
            mutable RNG rng_;
        };
    }
}

#endif
