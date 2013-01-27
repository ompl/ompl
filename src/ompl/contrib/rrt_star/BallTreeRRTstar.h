/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Authors: Alejandro Perez, Sertac Karaman, Ioan Sucan */

#ifndef OMPL_CONTRIB_RRT_STAR_BTRRTSTAR_
#define OMPL_CONTRIB_RRT_STAR_BTRRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/objectives/AccumulativeOptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <limits>
#include <vector>


namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gBallTreeRRTstar
           @par Short description
           Implementation of \ref gRRTstar "RRT*" that incorporates Ball Trees
           to approximate connected regions of free space with volumes in
           configuration space instead of points. Every vertex added to the tree has an initial volume
           of an infinite radius associated with it. This radius is gradually reduced as collisions are found.
           All samples within any of the existing volumes are discarded. However, discarded samples are
           collision checked. If a collision is found, the nearest volume is trimmed at the collision point.
           Information from all collision checking procedures within iterations is also used to trim volumes
           accordingly. The radii of volumes are considered when computing nearest/near neighbors. This
           implementation is suited for high-dimensional planning problems with narrow collision boundary passages.
           @par External documentation
           A. Perez, S. Karaman, M. Walter, A. Shkolnik, E. Frazzoli, S. Teller,
           Asymptotically-optimal Manipulation Planning using Incremental Sampling-based Algorithms,
           IEEE/RSJ International Conference on Intelligent Robots and Systems, 2011.
           \n\n S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research (to appear), 2011.
           <a href="http://arxiv.org/abs/1105.1186">http://arxiv.org/abs/1105.1186</a>


        */

        /** \brief Optimal Rapidly-exploring Random Trees with Ball Trees */
        class BallTreeRRTstar : public base::Planner
        {
        public:

            BallTreeRRTstar(const base::SpaceInformationPtr &si);

            virtual ~BallTreeRRTstar(void);

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear(void);

            /** \brief Set the goal bias

                In the process of randomly selecting states in
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
            double getGoalBias(void) const
            {
                return goalBias_;
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
            double getRange(void) const
            {
                return maxDistance_;
            }

            /** \brief When the planner attempts to rewire the tree,
                it does so by looking at some of the neighbors within
                a computed radius. The computation of that radius
                depends on the multiplicative factor set here.
                Set this parameter should be set at least to the side
                length of the (bounded) state space. E.g., if the state
                space is a box with side length L, then this parameter
                should be set to at least L for rapid and efficient
                convergence in trajectory space. */
            void setBallRadiusConstant(double ballRadiusConstant)
            {
                ballRadiusConst_ = ballRadiusConstant;
            }

            /** \brief Get the multiplicative factor used in the
                computation of the radius whithin which tree rewiring
                is done. */
            double getBallRadiusConstant(void) const
            {
                return ballRadiusConst_;
            }

            /** \brief When the planner attempts to rewire the tree,
                it does so by looking at some of the neighbors within
                a computed radius. That radius is bounded by the value
                set here. This parameter should ideally be equal longest
                straight line from the initial state to anywhere in the
                state space. In other words, this parameter should be
                "sqrt(d) L", where d is the dimensionality of space
                and L is the side length of a box containing the obstacle free space.*/
            void setMaxBallRadius(double maxBallRadius)
            {
                ballRadiusMax_ = maxBallRadius;
            }

            /** \brief Get the maximum radius the planner uses in the
                tree rewiring step */
            double getMaxBallRadius(void) const
            {
                return ballRadiusMax_;
            }

            /** \brief Each vertex in the tree has a volume (hyper-sphere) associated with it.
                The radius of said volume is decreased when collisions are found within it. Samples found
                within any of the existing volumes are rejected. The value set here is the initial radius assigned to volumes added
                to the tree.*/
            void setInitialVolumeRadius(double rO)
            {
                rO_ = rO;
            }

            /** \brief Get the initial volume radius */
            double getInitialVolumeRadius(void) const
            {
                return rO_;
            }

            /** \brief Verify if a state is inside an existing volume */
            bool inVolume(base::State *state)
            {
                for (std::size_t i = 0 ; i < motions_.size() ; ++i)
                {
                    if ((si_->distance(motions_[i]->state, state) <= motions_[i]->volRadius))
                        return true;
                }
                return false;
            }


            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }

            /** \brief Option that delays collision checking procedures.
                 When it is enabled, all neighbors are sorted by cost. The
                 planner then goes through this list, starting with the lowest
                 cost, checking for collisions in order to find a parent. The planner
                 stops iterating through the list when a collision free parent is found.
                 This prevents the planner from collsion checking each neighbor, reducing
                 computation time in scenarios where collision checking procedures are expensive.*/
            void setDelayCC(bool delayCC)
            {
                delayCC_ = delayCC;
            }

            /** \brief Get the state of the delayed collision checking option */
            bool getDelayCC(void) const
            {
                return delayCC_;
            }

            virtual void setup(void);

        protected:

	    typedef base::AccumulativeOptimizationObjective Objective;

            /** \brief Representation of a motion */
            class Motion
            {
            public:

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si, const base::OptimizationObjectivePtr& obj, double rO) : state(si->allocState()), parent(NULL), cost(obj->allocCost()), incCost(obj->allocCost()), volRadius(rO)

                {
                }

                ~Motion(void)
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

                /** \brief The cost of this motion */
		base::Cost        *cost;

		/** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
		base::Cost        *incCost;

                /** \brief The radius of the volume  associated to this motion */
                double             volRadius;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Add a motion to the tree */
            void addMotion(Motion* m)
            {
                nn_->add(m);
                motions_.push_back(m);
            }

            /** \brief Functor which allows us to sort a set of costs and maintain the original, unsorted indices into the sequence*/
	    typedef std::pair<std::size_t, base::Cost*> indexCostPair;
	    struct CostCompare
	    {
		CostCompare(const Objective& optObj) : optObj_(optObj) {}
		bool operator()(const indexCostPair& a, const indexCostPair& b)
		{
		    return optObj_.compareCost(a.second, b.second);
		}

		const Objective& optObj_;
	    };

            /** \brief Distance calculation considering volumes */
            double distanceFunction(const Motion* a, const Motion* b) const
            {
                return (si_->distance(a->state, b->state)) - a->volRadius;
            }

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief A copy of the list of motions in the tree used for faster verification of samples */
            std::vector<Motion*>                           motions_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief Shrink rate of radius the planner uses to find near neighbors and rewire */
            double                                         ballRadiusConst_;

            /** \brief Maximum radius the planner uses to find near neighbors and rewire */
            double                                         ballRadiusMax_;

            /** \brief Option to delay and reduce collision checking within iterations */
            bool                                           delayCC_;

            /** \brief Initial radius of volumes assigned to new vertices in the tree */
            double                                         rO_;

	    /** \brief Objective we're optimizing (currently can OptimizationObjectives which are subclasses of AccumulativeOptimizationObjective) */
            boost::shared_ptr<Objective> opt_;
        };

    }
}

#endif
