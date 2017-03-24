/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_
#define OMPL_GEOMETRIC_PLANNERS_KPIECE_LBKPIECE1_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/planners/kpiece/Discretization.h"

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gLBKPIECE1
           @par Short description
           KPIECE is a tree-based planner that uses a discretization
           (multiple levels, in general) to guide the exploration of
           the continuous space. This implementation is a simplified
           one, using a single level of discretization: one grid. The
           grid is imposed on a projection of the state space. When
           exploring the space, preference is given to the boundary of
           this grid. The boundary is computed to be the set of grid
           cells that have less than 2n non-diagonal neighbors in an
           n-dimensional projection space.
           It is important to set the projection the algorithm uses
           (setProjectionEvaluator() function). If no projection is
           set, the planner will attempt to use the default projection
           associated to the state space. An exception is thrown if
           no default projection is available either.
           This variant of the implementation use two trees of
           exploration with lazy collision checking, hence the LB
           prefix.
           @par External documentation
           - I.A. Şucan and L.E. Kavraki, Kinodynamic motion planning by interior-exterior cell exploration,
           in <em>Workshop on the Algorithmic Foundations of Robotics</em>, Dec. 2008.<br>
           [[PDF]](http://ioan.sucan.ro/files/pubs/wafr2008.pdf)
           - R. Bohlin and L.E. Kavraki, Path planning using lazy PRM, in <em>Proc. 2000 IEEE Intl. Conf. on Robotics
           and Automation</em>, pp. 521–528, 2000. DOI:
           [10.1109/ROBOT.2000.844107](http://dx.doi.org/10.1109/ROBOT.2000.844107)<br>
           [[PDF]](http://ieeexplore.ieee.org/ielx5/6794/18235/00844107.pdf?tp=&arnumber=844107&isnumber=18235)
        */

        /** \brief Lazy Bi-directional KPIECE with one level of discretization */
        class LBKPIECE1 : public base::Planner
        {
        public:
            /** \brief Constructor */
            LBKPIECE1(const base::SpaceInformationPtr &si);

            ~LBKPIECE1() override;

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state. */
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

            /** \brief Get the projection evaluator. */
            const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const
            {
                return projectionEvaluator_;
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
            /** \brief Set the fraction of time for focusing on the
                border (between 0 and 1). This is the minimum fraction
                used to select cells that are exterior (minimum
                because if 95% of cells are on the border, they will
                be selected with 95% chance, even if this fraction is
                set to 90%)*/
            void setBorderFraction(double bp)
            {
                dStart_.setBorderFraction(bp);
                dGoal_.setBorderFraction(bp);
            }

            /** \brief Get the fraction of time to focus exploration
                on boundary */
            double getBorderFraction() const
            {
                return dStart_.getBorderFraction();
            }

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion. This function sets the minimum acceptable
                fraction. */
            void setMinValidPathFraction(double fraction)
            {
                minValidPathFraction_ = fraction;
            }

            /** \brief Get the value of the fraction set by setMinValidPathFraction() */
            double getMinValidPathFraction() const
            {
                return minValidPathFraction_;
            }

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void clear() override;

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            /** \brief Representation of a motion for this algorithm */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si)
                  : state(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The root state (start state) that leads to this motion */
                const base::State *root{nullptr};

                /** \brief The state contained by this motion */
                base::State *state{nullptr};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                /** \brief Flag indicating whether this motion has been checked for validity. */
                bool valid{false};

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion *> children;
            };

            /** \brief Free the memory for a motion */
            void freeMotion(Motion *motion);

            /** \brief Remove a motion from a tree of motions */
            void removeMotion(Discretization<Motion> &disc, Motion *motion);

            /** \brief Since solutions are computed in a lazy fashion,
                once trees are connected, the solution found needs to
                be checked for validity. This function checks whether
                the reverse path from a given motion to a root is
                valid. If this is not the case, invalid motions are removed  */
            bool isPathValid(Discretization<Motion> &disc, Motion *motion, base::State *temp);

            /** \brief The employed state sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The employed projection evaluator */
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            /** \brief The start tree */
            Discretization<Motion> dStart_;

            /** \brief The goal tree */
            Discretization<Motion> dGoal_;

            /** \brief When extending a motion, the planner can decide
                to keep the first valid part of it, even if invalid
                states are found, as long as the valid part represents
                a sufficiently large fraction from the original
                motion */
            double minValidPathFraction_{0.5};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<base::State *, base::State *> connectionPoint_{nullptr, nullptr};
        };
    }
}

#endif
