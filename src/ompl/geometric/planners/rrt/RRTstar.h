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

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan */
/* CForest Authors: Javier V Gomez */

#ifndef OMPL_CONTRIB_RRT_STAR_RRTSTAR_
#define OMPL_CONTRIB_RRT_STAR_RRTSTAR_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <limits>
#include <vector>
#include <utility>
#include <boost/thread/mutex.hpp>


namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gRRTstar
           @par Short description
           \ref gRRTstar "RRT*" (optimal RRT) is an asymptotically-optimal incremental
           sampling-based motion planning algorithm. \ref gRRTstar "RRT*" algorithm is
           guaranteed to converge to an optimal solution, while its
           running time is guaranteed to be a constant factor of the
           running time of the \ref gRRT "RRT". The notion of optimality is with
           respect to the distance function defined on the state space
           we are operating on. See ompl::base::Goal::setMaximumPathLength() for
           how to set the maximally allowed path length to reach the goal.
           If a solution path that is shorter than ompl::base::Goal::getMaximumPathLength() is
           found, the algorithm terminates before the elapsed time.
           @par External documentation
           S. Karaman and E. Frazzoli, Sampling-based
           Algorithms for Optimal Motion Planning, International Journal of Robotics
           Research (to appear), 2011.
           <a href="http://arxiv.org/abs/1105.1186">http://arxiv.org/abs/1105.1186</a>
        */

        /** \brief Optimal Rapidly-exploring Random Trees */
        class RRTstar : public base::Planner
        {
        public:

            RRTstar(const base::SpaceInformationPtr &si);

            virtual ~RRTstar();

            virtual void getPlannerData(base::PlannerData &data) const;

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc)
            {
                return  base::PlannerStatus::UNKNOWN;
            }

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, const int idx);

            virtual void clear();

            virtual void includeValidPath(const std::vector<const base::State *> &states, const base::Cost cost);

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
            double getGoalBias() const
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
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors()
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
            bool getDelayCC() const
            {
                return delayCC_;
            }

            /** \brief Set the percentage threshold (between 0 and 1) for pruning the tree. If best cost is improved
                over this percentage, the  tree will be pruned. */
            void setPruneCostImprovementThreshold (const double pp)
            {
                pruneCostThreshold_ = pp;
            }

            /** \brief Get the current prune cost percentage threshold parameter. */
            
            double getPruneCostImprovementThreshold() const
            {
                return pruneCostThreshold_;
            }

            /** \brief Set the percentage threshold (between 0 and 1) for pruning the tree. If the new tree has removed
                at least this percentage of states, the tree will be finally pruned. */
            void setPruneStatesImprovementThreshold(const double pp)
            {
                pruneStatesThreshold_ = pp;
            }

            /** \brief Get the current prune states percentage threshold parameter. */
            double getPruneStatesImprovementThreshold () const
            {
                return pruneStatesThreshold_;
            }

            virtual void setup();

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const;

            std::string getCollisionCheckCount() const;

            std::string getBestCost() const;
            ///////////////////////////////////////

            /** \brief TO BE REMOVED in the final version. */
            void saveTree(const char * filename);

            /** \brief When called, the CForest parallelization framework is activated. */
            virtual void activateCForest() 
            {
                isCForest_ = true;
            }

            /** \brief When called, the CForest parallelization framework is deactivated. */
            virtual void deactivateCForest() 
            {
                isCForest_ = false;
            }

        protected:

            /** \brief Representation of a motion */
            class Motion
            {
            public:
                /** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
                Motion(const base::SpaceInformationPtr &si) :
                    state(si->allocState()),
                    parent(NULL)
                {
                }

                ~Motion()
                {
                }

                /** \brief The state contained by the motion */
                base::State       *state;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;

                /** \brief The cost up to this motion */
                base::Cost        cost;

                /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
                base::Cost        incCost;

                /** \brief The set of motions descending from the current motion */
                std::vector<Motion*> children;
            };

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost>& costs,
                                 const base::OptimizationObjective &opt) :
                    costs_(costs), opt_(opt)
                {}
                bool operator()(unsigned i, unsigned j)
                {
                    return opt_.isCostBetterThan(costs_[i],costs_[j]);
                }
                const std::vector<base::Cost>& costs_;
                const base::OptimizationObjective &opt_;
            };

            enum DistanceDirection { FROM_NEIGHBORS, TO_NEIGHBORS };
            
            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                switch (distanceDirection_)
                {
                case FROM_NEIGHBORS:
                    return si_->distance(a->state, b->state);
                case TO_NEIGHBORS:
                    return si_->distance(b->state, a->state);
                }
                return 0; // remove warning
            }

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);
            
            /** \brief If CForest is activated, this function will prune the tree when the best cost is improved over a threshold prunePercentage_. 
                returns the number of motions pruned. */
            int pruneTree();
            
            /** \brief If CForest is activated, this function will delete the pruned motions at the end of the solve() function. */
            void detelePrunedMotions();

            Motion* getRootMotion(Motion *seed);
            
            ompl::geometric::RRTstar::Motion* getInitialParent(Motion *rmotion, base::State *dstate, base::State *xstate);


            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double                                         maxDistance_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief Option to delay and reduce collision checking within iterations */
            bool                                           delayCC_;

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr                 opt_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion*>                           goalMotions_;

            /** \brief Directionality of distance computation for
                 nearest neighbors. Either from neighbors to new state,
                 or from new state to neighbors. */
            DistanceDirection                              distanceDirection_;

            /** \brief If this vector contains states, they will be included to the tree in the next iterations of solve(). */
            std::vector<base::State*>                      statesToInclude_;

             /** \brief If this vector contains motions, they will be deteled once the solve() function ends. */
            std::deque<Motion*>	                           toBeDeleted_;

             /** \brief Lock for includeValidPath() and pathsToInclude_ */
            boost::mutex                                   includePathsLock_;

            /** \brief If this value is set to true, the CForest parallelization framework will be activated. */
            bool                                           isCForest_;

            /** \brief CForest-related parameter. The tree is only pruned if there is a cost improvement over this percentage (between 0 and 1). */
            double                                         pruneCostThreshold_;

            /** \brief CForest-related parameter. The tree is only pruned is the percentage of states to prune is above this threshold (between 0 and 1). */
            double                                         pruneStatesThreshold_;

            /** \brief The cost used to prune the tree. It is set by CForest functions. */
            base::Cost                                     pruneTreeCost_;
            
            Motion *                                       prevMotion_;
            
            bool                                           addingSharedState_;
            
            Motion *                                       startMotion_;
            
            bool                                           restartPrevMotion_;

            //////////////////////////////
            // Planner progress properties

            /** \brief Number of iterations the algorithm performed */
            unsigned int                                   iterations_;

            /** \brief Number of collisions checks performed by the algorithm */
            unsigned int                                   collisionChecks_;

            /** \brief Best cost found so far by algorithm */
            base::Cost                                     bestCost_;     
            
            
            
            int idx_; 
        };
    }
}

#endif
