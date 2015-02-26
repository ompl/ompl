/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Autonomous Systems Laboratory, Stanford University
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
*   * Neither the name of Stanford University nor the names of its
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

/* Authors: Ashley Clark (Stanford) and Wolfgang Pointner (AIT) */
/* Co-developers: Brice Rebsamen (Stanford), Tim Wheeler (Stanford)
                  Edward Schmerling (Stanford), and Javier V. Gómez (UC3M - Stanford)*/
/* Algorithm design: Lucas Janson (Stanford) and Marco Pavone (Stanford) */
/* Acknowledgements for insightful comments: Oren Salzman (Tel Aviv University),
 *                                           Joseph Starek (Stanford) */

#ifndef OMPL_GEOMETRIC_PLANNERS_FMT_
#define OMPL_GEOMETRIC_PLANNERS_FMT_

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/base/OptimizationObjective.h>
#include <map>


namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gFMT
           @par Short description
           \ref gFMT "FMT*" is an asymptotically-optimal sampling-based motion
            planning algorithm, which is guaranteed to converge to a shortest
            path solution. The algorithm is specifically aimed at solving complex
            motion planning problems in high-dimensional configuration spaces.
            The \ref gFMT "FMT*" algorithm essentially performs a lazy dynamic
            programming recursion on a set of probabilistically-drawn samples to
            grow a tree of paths, which moves steadily outward in cost-to-come space.

           @par Deviation from the paper
           The implementation includes a cache in the collision checking since the original
           algorithm could check the same collision more than once. It increases the
           memory requirements to O(n logn), but as samples tend to infinity this
           bound tend to O(n).

           @par External documentation
           L. Janson, A. Clark, and M. Pavone, Fast Marching Trees: a Fast Marching
           Sampling-Based Method for Optimal Motion Planning in Many Dimensions,
           International Journal on Robotics Research, 2014. Submitted.
           http://arxiv.org/pdf/1306.3532v3.pdf<br>
           [[PDF]](http://web.stanford.edu/~pavone/papers/Janson.Pavone.IJRR14.pdf)
        */
        /** @brief Asymptotically Optimal Fast Marching Tree algorithm developed
            by L. Janson and M. Pavone. */
        class FMT : public ompl::base::Planner
        {
        public:

            FMT(const base::SpaceInformationPtr &si);

            virtual ~FMT();

            virtual void setup();

            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            virtual void clear();

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Set the number of states that the planner should sample.
                The planner will sample this number of states in addition to the
                initial states. If any of the goal states are not reachable from
                the randomly sampled states, those goal states will also be
                added. The default value is 1000 */
            void setNumSamples(const unsigned int numSamples)
            {
                numSamples_ = numSamples;
            }

            /** \brief Get the number of states that the planner will sample */
            unsigned int getNumSamples() const
            {
                return numSamples_;
            }

            /** \brief If nearestK is true, FMT will be run using the Knearest strategy */
            void setNearestK(bool nearestK)
            {
                nearestK_ = nearestK;
            }

            /** \brief Get the state of the nearestK strategy */
            bool getNearestK() const
            {
                return nearestK_;
            }

            /** \brief The planner searches for neighbors of a node within a
                cost r, where r is the value described for FMT* in Section 4
                of [L. Janson, A. Clark, and M. Pavone, "Fast Marching Trees: a Fast
                Marching Sampling-Based Method for Optimal Motion Planning in
                Many Dimensions," International Symposium on
                Robotics Research, 2013.               http://arxiv.org/pdf/1306.3532v3.pdf] For guaranteed asymptotic
                convergence, the user should choose a constant multiplier for
                the search radius that is greater than one. The default value is 1.1.
                In general, a radius multiplier between 0.9 and 5 appears to
                perform the best */
            void setRadiusMultiplier(const double radiusMultiplier)
            {
                if (radiusMultiplier <= 0.0)
                    throw Exception("Radius multiplier must be greater than zero");
                radiusMultiplier_ = radiusMultiplier;
            }

            /** \brief Get the multiplier used for the nearest neighbors search
                radius */
            double getRadiusMultiplier() const
            {
                return radiusMultiplier_;
            }

            /** \brief Store the volume of the obstacle-free configuration space.
                If no value is specified, the default assumes an obstacle-free
                unit hypercube, freeSpaceVolume = (maximumExtent/sqrt(dimension))^(dimension) */
            void setFreeSpaceVolume(const double freeSpaceVolume)
            {
                if (freeSpaceVolume < 0.0)
                    throw Exception("Free space volume should be greater than zero");
                freeSpaceVolume_ = freeSpaceVolume;
            }

            /** \brief Get the volume of the free configuration space that is
                being used by the planner */
            double getFreeSpaceVolume() const
            {
                return freeSpaceVolume_;
            }

            /** \brief Sets the collision check caching to save calls to the collision
                checker with slightly memory usage as a counterpart */
            void setCacheCC(bool ccc)
            {
                cacheCC_ = ccc;
            }

            /** \brief Get the state of the collision check caching */
            bool getCacheCC() const
            {
                return cacheCC_;
            }

           /** \brief Activates the cost to go heuristics when ordering the heap */
           void setHeuristics (bool h)
           {
               heuristics_ = h;
           }

            /** \brief Returns true if the heap is ordered taking into account
                cost to go heuristics */
            bool getHeuristics() const
            {
                return heuristics_;
            }

        protected:
            /** \brief Representation of a motion
              */
            class Motion
            {
                public:

                    /** \brief The FMT* planner begins with all nodes included in
                        set Unvisited "Waiting for optimal connection". As nodes are
                        connected to the tree, they are transferred into set Open
                        "Horizon of explored tree." Once a node in Open is no longer
                        close enough to the frontier to connect to any more nodes in
                        Unvisited, it is removed from Open. These three SetTypes are flags
                        indicating which set the node belongs to; Open, Unvisited, or Closed (neither) */
                    enum SetType { SET_CLOSED, SET_OPEN, SET_UNVISITED };

                    Motion()
                        : state_(NULL), parent_(NULL), cost_(0.0), currentSet_(SET_UNVISITED)
                    {
                    }

                    /** \brief Constructor that allocates memory for the state */
                    Motion(const base::SpaceInformationPtr &si)
                        : state_(si->allocState()), parent_(NULL), cost_(0.0), currentSet_(SET_UNVISITED)
                    {
                    }

                    ~Motion()
                    {
                    }

                    /** \brief Set the state associated with the motion */
                    void setState(base::State *state)
                    {
                        state_ = state;
                    }

                    /** \brief Get the state associated with the motion */
                    base::State* getState() const
                    {
                        return state_;
                    }

                    /** \brief Set the parent motion of the current motion */
                    void setParent(Motion *parent)
                    {
                        parent_ = parent;
                    }

                    /** \brief Get the parent motion of the current motion */
                    Motion* getParent() const
                    {
                        return parent_;
                    }

                    /** \brief Set the cost-to-come for the current motion */
                    void setCost(const base::Cost cost)
                    {
                        cost_ = cost;
                    }

                    /** \brief Get the cost-to-come for the current motion */
                    base::Cost getCost() const
                    {
                        return cost_;
                    }

                    /** \brief Specify the set that this motion belongs to */
                    void setSetType(const SetType currentSet)
                    {
                        currentSet_ = currentSet;
                    }

                    /** \brief Get the set that this motion belongs to */
                    SetType getSetType() const
                    {
                        return currentSet_;
                    }

                    /** \brief Returns true if the connection to m has been already
                        tested and failed because of a collision */
                    bool alreadyCC(Motion *m)
                    {
                        if (collChecksDone_.find(m) == collChecksDone_.end())
                            return false;
                        return true;
                    }

                    /** \brief Caches a failed collision check to m */
                    void addCC(Motion *m)
                    {
                        collChecksDone_.insert(m);
                    }

                    /** \brief Set the cost to go heuristic cost */
                    void setHeuristicCost(const base::Cost h)
                    {
                        hcost_ = h;
                    }

                    /** \brief Get the cost to go heuristic cost */
                    base::Cost getHeuristicCost() const
                    {
                        return hcost_;
                    }

                protected:

                    /** \brief The state contained by the motion */
                    base::State *state_;

                    /** \brief The parent motion in the exploration tree */
                    Motion *parent_;

                    /** \brief The cost of this motion */
                    base::Cost cost_;

                    /** \brief The minimum cost to go of this motion (heuristically computed) */
                    base::Cost hcost_;

                    /** \brief The flag indicating which set a motion belongs to */
                    SetType currentSet_;

                    /** \brief Contains the connections attempted FROM this node */
                    std::set<Motion*> collChecksDone_;
            };

            /** \brief Comparator used to order motions in a binary heap */
            struct MotionCompare
            {
                MotionCompare() : opt_(NULL), heuristics_(false)
                {
                }

                /* Returns true if m1 is lower cost than m2. m1 and m2 must
                   have been instantiated with the same optimization objective */
                bool operator()(const Motion *m1, const Motion *m2) const
                {
                    if (heuristics_)
                        return opt_->isCostBetterThan(opt_->combineCosts(m1->getCost(), m1->getHeuristicCost()),
                                                      opt_->combineCosts(m2->getCost(), m2->getHeuristicCost()));
                    else
                        return opt_->isCostBetterThan(m1->getCost(), m2->getCost());
                }

                base::OptimizationObjective* opt_;
                bool heuristics_;
            };

            /** \brief Compute the distance between two motions as the cost
                between their contained states. Note that for computationally
                intensive cost functions, the cost between motions should be
                stored to avoid duplicate calculations */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return opt_->motionCost(a->getState(), b->getState()).value();
            }

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Sample a state from the free configuration space and save
                it into the nearest neighbors data structure */
            void sampleFree(const ompl::base::PlannerTerminationCondition &ptc);

            /** \brief For each goal region, check to see if any of the sampled
                states fall within that region. If not, add a goal state from
                that region directly into the set of vertices. In this way, FMT
                is able to find a solution, if one exists. If no sampled nodes
                are within a goal region, there would be no way for the
                algorithm to successfully find a path to that region */
            void assureGoalIsSampled(const ompl::base::GoalSampleableRegion *goal);

            /** \brief Compute the volume of the unit ball in a given dimension */
            double calculateUnitBallVolume(const unsigned int dimension) const;

            /** \brief Calculate the radius to use for nearest neighbor searches,
                using the bound given in [L. Janson, A. Clark, and M.
                Pavone, "Fast Marching Trees: a Fast Marching Sampling-Based
                Method for Optimal Motion Planning in Many Dimensions,"
                International Journal on Robotics Research,
                2013. http://arxiv.org/pdf/1306.3532v3.pdf]. The radius depends on
                the radiusMultiplier parameter, the volume of the free
                configuration space, the volume of the unit ball in the current
                dimension, and the number of nodes in the graph */
            double calculateRadius(unsigned int dimension, unsigned int n) const;

            /** \brief Save the neighbors within a neighborhood of a given state. The strategy
                used (nearestK or nearestR depends on the planner configuration */
            void saveNeighborhood(Motion *m);

            /** \brief Trace the path from a goal state back to the start state
                and save the result as a solution in the Problem Definiton.
             */
            void traceSolutionPathThroughTree(Motion *goalMotion);

            /** \brief Complete one iteration of the main loop of the FMT* algorithm:
                Find K nearest nodes in set Unvisited (or within a radius r) of the node z.
                Attempt to connect them to their optimal cost-to-come parent
                in set Open. Remove all newly connected nodes fromUnvisited and insert
                them into Open. Remove motion z from Open, and update z to be the
                current lowest cost-to-come node in Open */
            bool expandTreeFromNode(Motion **z);

            /** \brief A binary heap for storing explored motions in
                cost-to-come sorted order */
            typedef ompl::BinaryHeap<Motion*, MotionCompare> MotionBinHeap;

            /** \brief A binary heap for storing explored motions in
                cost-to-come sorted order. The motions in Open have been explored,
                yet are still close enough to the frontier of the explored set Open
                to be connected to nodes in the unexplored set Unvisited */
            MotionBinHeap Open_;

            /** \brief A map linking a motion to all of the motions within a
                distance r of that motion */
            std::map<Motion*, std::vector<Motion*> > neighborhoods_;

            /** \brief The number of samples to use when planning */
            unsigned int numSamples_;

            /** \brief Number of collision checks performed by the algorithm */
            unsigned int collisionChecks_;

            /** \brief Flag to activate the K nearest neighbors strategy */
            bool nearestK_;

            /** \brief Flag to activate the collision check caching */
            bool cacheCC_;

            /** \brief Flag to activate the cost to go heuristics */
            bool heuristics_;

            /** \brief Radius employed in the nearestR strategy. */
            double NNr_;

            /** \brief K used in the nearestK strategy */
            unsigned int NNk_;

            /** \brief The volume of the free configuration space, computed
                as an upper bound with 95% confidence */
            double freeSpaceVolume_;

            /** \brief This planner uses a nearest neighbor search radius
                proportional to the lower bound for optimality derived for FMT*
                in Section 4 of [L. Janson, A. Clark, and M. Pavone, "Fast
                Marching Trees: a Fast Marching Sampling-Based Method for
                Optimal Motion Planning in Many Dimensions," International
                Journal on Robotics Research, 2013.
                http://arxiv.org/pdf/1306.3532v3.pdf].  The radius multiplier
                is the multiplier for the lower bound. For guaranteed asymptotic
                convergence, the user should choose a multiplier for the search
                radius that is greater than one. The default value is 1.1.
                In general, a radius between 0.9 and 5 appears to perform the best
             */
            double radiusMultiplier_;

            /** \brief A nearest-neighbor datastructure containing the set of all motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The cost objective function */
            base::OptimizationObjectivePtr opt_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;

            /** \brief Goal state caching to accelerate cost to go heuristic computation */
            base::State* goalState_;
        };
    }
}

#endif // OMPL_GEOMETRIC_PLANNERS_FMT_
