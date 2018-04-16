/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Bryant Gipson, Mark Moll */

#ifndef OMPL_GEOMETRIC_PLANNERS_STRIDE_STRIDE_
#define OMPL_GEOMETRIC_PLANNERS_STRIDE_STRIDE_

#include "ompl/datastructures/Grid.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/PDF.h"
#include <unordered_map>
#include <boost/scoped_ptr.hpp>
#include <vector>

namespace ompl
{
    template <typename _T>
    class NearestNeighborsGNAT;

    namespace geometric
    {
        /**
          @anchor gSTRIDE

          @par Short description
          STRIDE (Search Tree with Resolution Independent Density Estimation)
          is a tree-based motion planner that attempts to detect
          the less explored area of the space through the use of a
          GNAT nearest-neighbor data structure. It is similar to EST,
          but unlike the EST implementation in OMPL does not require
          a projection. However, in case the state space has many
          dimensions, a projection can be specified and the GNAT
          can be built using distances in the projected space. This
          has the advantage over the EST implementation that no grid
          cell sizes have to be specified.

          @par External documentation
          B. Gipson, M. Moll, and L.E. Kavraki, Resolution independent density
          estimation for motion planning in high-dimensional spaces, in
          <em>IEEE Intl. Conf. on Robotics and Automation</em>, pp. 2429-2435, 2013.
          [[PDF]](http://dx.doi.org/10.1109/ICRA.2013.6630908)
        */

        /** \brief Search Tree with Resolution Independent Density Estimation */
        class STRIDE : public base::Planner
        {
        public:
            /** \brief Constructor */
            STRIDE(const base::SpaceInformationPtr &si, bool useProjectedDistance = false, unsigned int degree = 16,
                   unsigned int minDegree = 12, unsigned int maxDegree = 18, unsigned int maxNumPtsPerLeaf = 6,
                   double estimatedDimension = 0.0);
            ~STRIDE() override;

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief In the process of randomly selecting states in
              the state space to attempt to go towards, the
              algorithm may in fact choose the actual goal state, if
              it knows it, with some probability. This probability
              is a real number between 0.0 and 1.0; its value should
              usually be around 0.05 and should not be too large. It
              is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set whether nearest neighbors are computed based
              on distances in a _projection_ of the state rather distances
              in the state space itself. */
            void setUseProjectedDistance(bool useProjectedDistance)
            {
                useProjectedDistance_ = useProjectedDistance;
            }
            /** \brief Return whether nearest neighbors are computed based
              on distances in a _projection_ of the state rather distances
              in the state space itself. */
            bool getUseProjectedDistance() const
            {
                return useProjectedDistance_;
            }

            /** \brief Set desired degree of a node in the GNAT. */
            void setDegree(unsigned int degree)
            {
                degree_ = degree;
            }
            /** \brief Get desired degree of a node in the GNAT. */
            unsigned int getDegree() const
            {
                return degree_;
            }
            /** \brief Set minimum degree of a node in the GNAT. */
            void setMinDegree(unsigned int minDegree)
            {
                minDegree_ = minDegree;
            }
            /** \brief Get minimum degree of a node in the GNAT. */
            unsigned int getMinDegree() const
            {
                return minDegree_;
            }
            /** \brief Set maximum degree of a node in the GNAT. */
            void setMaxDegree(unsigned int maxDegree)
            {
                maxDegree_ = maxDegree;
            }
            /** \brief Set maximum degree of a node in the GNAT. */
            unsigned int getMaxDegree() const
            {
                return maxDegree_;
            }
            /** \brief Set maximum number of elements stored in a leaf
              node of the GNAT. */
            void setMaxNumPtsPerLeaf(unsigned int maxNumPtsPerLeaf)
            {
                maxNumPtsPerLeaf_ = maxNumPtsPerLeaf;
            }
            /** \brief Get maximum number of elements stored in a leaf
              node of the GNAT. */
            unsigned int getMaxNumPtsPerLeaf() const
            {
                return maxNumPtsPerLeaf_;
            }
            /** \brief Set estimated dimension of the free space, which
              is needed to compute the sampling weight for a node in the
              GNAT. */
            void setEstimatedDimension(double estimatedDimension)
            {
                estimatedDimension_ = estimatedDimension;
            }
            /** \brief Get estimated dimension of the free space, which
              is needed to compute the sampling weight for a node in the
              GNAT. */
            double getEstimatedDimension() const
            {
                return estimatedDimension_;
            }

            /** \brief Set the range the planner is supposed to use.

              This parameter greatly influences the runtime of the
              algorithm. It represents the maximum length of a
              motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }
            /** \brief When extending a motion, the planner can decide
              to keep the first valid part of it, even if invalid
              states are found, as long as the valid part represents
              a sufficiently large fraction from the original
              motion. This function sets the minimum acceptable
              fraction (between 0 and 1). */
            void setMinValidPathFraction(double fraction)
            {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction() const
            {
                return minValidPathFraction_;
            }
            /** \brief Set the projection evaluator. This class is
              able to compute the projection of a given state.  */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
              the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator */
            const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            /** \brief The definition of a motion */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Initialize GNAT data structure */
            void setupTree();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief Compute distance between motions (actually distance between projections of contained states) */
            double projectedDistanceFunction(const Motion *a, const Motion *b) const
            {
                unsigned int num_dims = projectionEvaluator_->getDimension();
                Eigen::VectorXd aproj(num_dims), bproj(num_dims);
                projectionEvaluator_->project(a->state, aproj);
                projectionEvaluator_->project(b->state, bproj);
                return (aproj - bproj).norm();
            }

            /** \brief Add a motion to the exploration tree */
            void addMotion(Motion *motion);

            /** \brief Select a motion to continue the expansion of the tree from */
            Motion *selectMotion();

            /** \brief Valid state sampler */
            base::ValidStateSamplerPtr sampler_;

            /** \brief This algorithm can optionally use a projection to guide the exploration. */
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            /** \brief The exploration tree constructed by this algorithm */
            boost::scoped_ptr<NearestNeighborsGNAT<Motion *>> tree_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Whether to use distance in the projection (instead of distance in the state space) for the GNAT
             */
            bool useProjectedDistance_;
            /** \brief Desired degree of an internal node in the GNAT */
            unsigned int degree_;
            /** \brief Minimum degree of an internal node in the GNAT */
            unsigned int minDegree_;
            /** \brief Maximum degree of an internal node in the GNAT */
            unsigned int maxDegree_;
            /** \brief Maximum number of points stored in a leaf node in the GNAT */
            unsigned int maxNumPtsPerLeaf_;
            /** \brief Estimate of the local dimensionality of the free space around a state */
            double estimatedDimension_;
            /** \brief When extending a motion, the planner can decide
                 to keep the first valid part of it, even if invalid
                 states are found, as long as the valid part represents
                 a sufficiently large fraction from the original
                 motion. This is used only when extendWhileValid_ is true. */
            double minValidPathFraction_{.2};

            /** \brief The random number generator */
            RNG rng_;
        };
    }  // namespace geometric
}  // namespace ompl

#endif
