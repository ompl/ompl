/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025, University of Santa Cruz Hybrid Systems Laboratory
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
 *   * Neither the name of the University of Santa Cruz nor the names of 
 *     its contributors may be used to endorse or promote products derived
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

/* Authors: Beverly Xu */
/* Adapted from: ompl/geometric/planners/src/SST.cpp by  Zakary Littlefield of Rutgers the State University of New Jersey, New Brunswick */

#ifndef OMPL_CONTROL_PLANNERS_SST_HySST_
#define OMPL_CONTROL_PLANNERS_SST_HySST_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/control/Control.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/spaces/HybridStateSpace.h"

namespace ompl
{
    namespace control
    {
        /**
           @anchor gHySST
           @par Hybrid Stable-sparse RRT (HySST) is an asymptotically near-optimal incremental
           sampling-based motion planning algorithm. It is recommended for geometric problems
           to use an alternative method that makes use of a steering function. Using HySST for
           geometric problems does not take advantage of this function.
           @par External documentation
           N. Wang and R. G. Sanfelice, "HySST: An Asymptotically Near-Optimal Motion Planning Algorithm for Hybrid Systems."
           [[PDF]](https://arxiv.org/pdf/2305.18649)
        */
        class HySST : public base::Planner
        {
        public:
            /** \brief Constructor */
            HySST(const control::SpaceInformationPtr &si);

            /** \brief Destructor */
            ~HySST() override;

            /** \brief Set the problem instance to solve */
            void setup() override;

            /** \brief Representation of a motion
             * @par This only contains pointers to parent motions as we
             * only need to go backwards in the tree. 
             */
            class Motion
            {
            public:
                /// \brief Default constructor
                Motion() = default;

                /// \brief Constructor that allocates memory for the state
                Motion(const control::SpaceInformation *si) : state(si->allocState()), control(si->allocControl()) {}

                /// \brief Destructor
                virtual ~Motion() = default;

                /** \brief Get the state contained by the motion */
                virtual base::State *getState() const
                {
                    return state;
                }

                /** \brief Get the parent motion in the tree */
                virtual Motion *getParent() const
                {
                    return parent;
                }
                
                /// \brief The total cost accumulated from the root to this vertex
                base::Cost accCost_{0.};

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief Number of children. Starting with 0.
                unsigned numChildren_{0};

                /// \brief If inactive, this node is not considered for selection.
                bool inactive_{false};

                /// \brief The integration steps defining the edge of the motion, between the parent and child vertices
                std::vector<base::State *> *solutionPair{nullptr};

                /// \brief The inputs associated with the solution pair
                control::Control *control{nullptr};
            };

            /** 
             * \brief Main solve function. 
             * @par Continue solving for some amount of time.
             * @param ptc The condition to terminate solving.
             * @return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** 
             * \brief Get the PlannerData object associated with this planner
             * @param data the PlannerData object storing the edges and vertices of the solution
             */
            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Clear all allocated memory. */
            void clear() override;

            /**
             * \brief Set the radius for selecting nodes relative to random sample.
             * @par This radius is used to mimic behavior of RRT* in that it promotes
             * extending from nodes with good path cost from the root of the tree.
             * Making this radius larger will provide higher quality paths, but has two
             * major drawbacks; exploration will occur much more slowly and exploration
             * around the boundary of the state space may become impossible. 
             * @param selectionRadius The maximum distance from the random sampled vertex, for a vertex in the search tree to be considered
             */
            void setSelectionRadius(double selectionRadius)
            {
                if (selectionRadius < 0)
                    throw Exception("Selection radius must be positive");
                selectionRadius_ = selectionRadius;
            }

            /** 
             * \brief Get the selection radius the planner is using 
             * @return The selection radius
             */
            double getSelectionRadius() const
            {
                return selectionRadius_;
            }

            /**
             * \brief Set the radius for pruning nodes.
             * @par This is the radius used to surround nodes in the witness set.
             * Within this radius around a state in the witness set, only one
             * active tree node can exist. This limits the size of the tree and
             * forces computation to focus on low path costs nodes. If this value
             * is too large, narrow passages will be impossible to traverse. In addition,
             * children nodes may be removed if they are not at least this distance away
             * from their parent nodes.
             * @param pruningRadius The radius used to prune vertices in the search tree
             */
            void setPruningRadius(double pruningRadius)
            {
                if (pruningRadius < 0)
                    throw ompl::Exception("Pruning radius must be non-negative");
                pruningRadius_ = pruningRadius;
            }

            /** 
             * \brief Get the pruning radius the planner is using 
             * @return The pruning radius
             */
            double getPruningRadius() const
            {
                return pruningRadius_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                witnesses_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            /** 
             * \brief Set the maximum time allocated to a full continuous simulator step.
             * @param tM the maximum time allocated. Must be greater than 0, and greater than than the time
             * allocated to a single continuous simulator call. 
             */
            void setTm(double tM)
            {
                if (tM <= 0)
                    throw Exception("Maximum flow time per propagation step must be greater than 0");
                if (!flowStepDuration_)
                {
                    if (tM < flowStepDuration_)
                        throw Exception("Maximum flow time per propagation step must be greater than or equal to the length of time for each flow "
                                        "integration step (flowStepDuration_)");
                }
                tM_ = tM;
            }

            /** 
             * \brief Set the time allocated to a single continuous simulator call, within the full period of a continuous simulator step. 
             * @param duration the time allocated per simulator step. Must be
             * greater than 0 and less than the time allocated to a full continuous
             * simulator step. 
             */
            void setFlowStepDuration(double duration)
            {
                if (duration <= 0)
                    throw Exception("Flow step length must be greater than 0");
                if (!tM_)
                {
                    if (tM_ < duration)
                        throw Exception("Flow step length must be less than or equal to the maximum flow time per propagation step (Tm)");
                }
                flowStepDuration_ = duration;
            }

            /** 
             * \brief Define the jump set
             * @param jumpSet the jump set associated with the hybrid system. 
             */
            void setJumpSet(std::function<bool(Motion *)> jumpSet)
            {
                jumpSet_ = jumpSet;
            }

            /** 
             * \brief Define the flow set
             * @param flowSet the flow set associated with the hybrid system.
             */
            void setFlowSet(std::function<bool(Motion *)> flowSet)
            {
                flowSet_ = flowSet;
            }

            /** 
             * \brief Define the unsafe set
             * @param unsafeSet the unsafe set associated with the hybrid system.
             */
            void setUnsafeSet(std::function<bool(Motion *)> unsafeSet)
            {
                unsafeSet_ = unsafeSet;
            }

            /** 
             * \brief Define the distance measurement function
             * @param function the distance function associated with the motion planning problem.
             */
            void setDistanceFunction(std::function<double(base::State *, base::State *)> function)
            {
                distanceFunc_ = function;
            }

            /** 
             * \brief Define the discrete dynamics simulator
             * @param function the discrete simulator associated with the hybrid system.
             */
            void setDiscreteSimulator(std::function<base::State *(base::State *curState, const control::Control *u, base::State *newState)> function)
            {
                discreteSimulator_ = function;
            }

            /** 
             * \brief Define the continuous dynamics simulator
             * @param function the continuous simulator associated with the hybrid system.
             */
            void setContinuousSimulator(std::function<base::State *(const control::Control *u, base::State *curState, double tFlowMax, 
                                        base::State *newState)> function)
            {
                continuousSimulator_ = function;
            }

            
            /** \brief Simulates the dynamics of the system. */
            std::function<ompl::base::State *(const control::Control *control, ompl::base::State *x_cur, double tFlow, ompl::base::State *new_state)> continuousSimulator = [this](const control::Control *control, base::State *x_cur, double tFlow, base::State *new_state)
            {
                siC_->getStatePropagator()->propagate(x_cur, control, tFlow, new_state);
                return new_state;
            };

            /** 
             * \brief Define the collision checker
             * @param function the collision checker associated with the state space. Default is a point-by-point collision checker.
             */
            void setCollisionChecker(std::function<bool(Motion *motion, std::function<bool(Motion *motion)> obstacleSet, 
                                     base::State *newState, double *collisionTime)> function)
            {
                collisionChecker_ = function;
            }

            /** 
             * \brief Set solution batch size 
             * @param batchSize the number of solutions to be generated by the planner until the one with the best cost is returned.
             */
            void setBatchSize(int batchSize) {
                if(batchSize < 1)
                    throw Exception("Batch size must be greater than 0");
                batchSize_ = batchSize;
            }
            /** \brief Check if all required parameters have been set. */
            void checkMandatoryParametersSet(void) const
            {
                if (!discreteSimulator_)
                    throw Exception("Jump map not set");
                if (!continuousSimulator_)
                    throw Exception("Flow map not set");
                if (!flowSet_)
                    throw Exception("Flow set not set");
                if (!jumpSet_)
                    throw Exception("Jump set not set");
                if (!unsafeSet_)
                    throw Exception("Unsafe set not set");
                if (tM_ < 0.0)
                    throw Exception("Max flow propagation time (Tm) not set");
                if (pruningRadius_ < 0)
                    throw Exception("Pruning radius (pruningRadius_) not set");
                if (selectionRadius_ < 0)
                    throw Exception("Selection radius (selectionRadius_) not set");
            }

        protected:
            const static ompl::control::Control *getFlowControl(const ompl::control::Control *control)
            {
                return control->as<CompoundControl>()->as<ompl::control::Control>(0);
            }

            const static ompl::control::Control *getJumpControl(const ompl::control::Control *control)
            {
                return control->as<CompoundControl>()->as<ompl::control::Control>(1);
            }

            /// \brief Representation of a witness vertex in the search tree
            class Witness : public Motion
            {
            public:
                /// \brief Default Constructor
                Witness() = default;

                /// \brief Constructor that allocates memory for the state
                Witness(const control::SpaceInformation *si) : Motion(si) {}

                /** 
                 * \brief Get the state contained by the representative motion 
                 * @return The state contained by the representative motion
                 */
                base::State *getState() const override
                {
                    return rep_->state;
                }

                /** 
                 * \brief Get the state contained by the parent motion of the representative motion 
                 * @return The state contained by the parent motion of the representative motion
                 */
                Motion *getParent() const override
                {
                    return rep_->parent;
                }

                /** 
                 * \brief Set the representative of the witness
                 * \param lRep The representative motion
                 */
                void linkRep(Motion *lRep)
                {
                    rep_ = lRep;
                }

                /** \brief The node in the tree that is within the pruning radius.*/
                Motion *rep_{nullptr};
            };

            /// \brief The most recent goal motion.  Used for PlannerData computation
            Motion *lastGoalMotion_{nullptr};

            /** \brief Runs the initial setup tasks for the tree. */
            void initTree(void);

            /** 
             * \brief Sample the random motion. 
             * @param randomMotion The motion to be initialized
             */
            void randomSample(Motion *randomMotion);

            /// \brief A nearest-neighbors datastructure containing the tree of motions
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /**
             * The following are all customizeable parameters,
             * and affect how @b cHySST generates trajectories.
             * Customize using setter functions above. */

            /**
             * \brief Collision checker. Default is point-by-point collision checking using the jump set.
             * @param motion The motion to check for collision
             * @param obstacleSet A function that returns true if the motion's solution pair intersects with the obstacle set
             * @param ts The start time of the motion. Default is -1.0
             * @param tf The end time of the motion. Default is -1.0
             * @param newState The collision state (if a collision occurs)
             * @param collisionTime The time of collision (if a collision occurs). If no collision occurs, this value is -1.0
             * @return true if a collision occurs, false otherwise
             */
            std::function<bool(Motion *motion,
                               std::function<bool(Motion *motion)> obstacleSet,
                               base::State *newState, double *collisionTime)>
                collisionChecker_ =
                    [this](Motion *motion,
                           std::function<bool(Motion *motion)> obstacleSet, base::State *newState, double *collisionTime) -> bool
            {
                if (obstacleSet(motion)) {
                    si_->copyState(newState, motion->solutionPair->back());
                    *collisionTime = ompl::base::HybridStateSpace::getStateTime(motion->solutionPair->back()) - flowStepDuration_;
                    motion->solutionPair->resize(motion->solutionPair->size() - 1);
                    return true;
                }
                return false;
            };

            /// \brief Control Sampler
            control::DirectedControlSamplerPtr controlSampler_;

            /// \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience
            control::SpaceInformation *siC_;

            /** 
             * \brief Compute distance between states, default is Euclidean distance 
             * @param state1 The first state
             * @param state2 The second state
             * @return The distance between the two states
             */
            std::function<double(base::State *state1, base::State *state2)> distanceFunc_ = [this](base::State *state1, base::State *state2) -> double
            {
                return si_->distance(state1, state2);
            };

            /// \brief The maximum flow time for a given flow propagation step. Must be set by the user.
            double tM_{-1.};

            /// \brief The minimum step length for a given flow propagation step. Default value is 1e-6
            double minStepLength = 1e-06;

            /// \brief The flow time for a given integration step, within a flow propagation step. Must be set by user.
            double flowStepDuration_;

            /** 
             * \brief Simulator for propagation under jump regime
             * @param curState The current state
             * @param u The input
             * @param newState The newly propagated state
             * @return The newly propagated state
             */
            std::function<base::State *(base::State *curState, const control::Control *u, base::State *newState)> discreteSimulator_;

            /** 
             * \brief Function that returns true if a motion intersects with the jump set, and false if not. 
             * @param motion The motion to check
             * @return True if the state is in the jump set, false if not
             */
            std::function<bool(Motion *motion)> jumpSet_;

            /** 
             * \brief Function that returns true if a motion intersects with the flow set, and false if not. 
             * @param motion The motion to check
             * @return True if the state is in the flow set, false if not
             */
            std::function<bool(Motion *motion)> flowSet_;

            /** 
             * \brief Function that returns true if a motion intersects with the unsafe set, and false if not. 
             * @param motion The motion to check
             * @return True if the state is in the unsafe set, false if not
             */
            std::function<bool(Motion *motion)> unsafeSet_;

            /// \brief The optimization objective. Default is a shortest path objective.
            base::OptimizationObjectivePtr opt_;

            /** \brief Calculate the cost of a motion. Default is using optimization objective. */
            std::function<base::Cost (Motion *motion)> costFunc_;

            /** 
             * \brief Simulator for propagation under flow regime
             * @param input The input
             * @param curState The current state
             * @param tFlowMax The random maximum flow time
             * @param newState The newly propagated state
             * @return The newly propagated state
             */
            std::function<base::State *(const control::Control *u, base::State *curState, double tFlowMax, base::State *newState)> continuousSimulator_;

            /** 
             * \brief Construct the path, starting at the last edge. 
             * @param lastMotion The last motion in the solution
             * @return the planner status (APPROXIMATE, EXACT, or UNKNOWN)
             */
            base::PlannerStatus constructSolution(Motion *lastMotion);

            /** 
             * \brief Finds the best node in the tree withing the selection radius around a random sample.
             * @param sample The random sampled vertex to find the closest witness to.
             */
            Motion *selectNode(Motion *sample);

            /** 
             * \brief Find the closest witness node to a newly generated potential node.
             * @param node The vertex to find the closest witness to.
             */
            Witness *findClosestWitness(Motion *node);

            /** 
             * \brief Randomly propagate a new edge.
             * @param m The motion to extend
             */
            std::vector<Motion *> extend(Motion *m);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /// \brief State sampler
            base::StateSamplerPtr sampler_;

            /// \brief A nearest-neighbors datastructure containing the tree of witness motions
            std::shared_ptr<NearestNeighbors<Motion *>> witnesses_;

            /// \brief The radius for determining the node selected for extension. Delta_s.
            double selectionRadius_{-1.};   

            /// \brief The radius for determining the size of the pruning region. Delta_bn.
            double pruningRadius_{-1.};

            /// \brief The random number generator
            RNG rng_;

            /// \brief Minimum distance from goal to final vertex of generated trajectories.
            double dist_;

            /// \brief The best solution (with best cost) we have found so far.
            std::vector<Motion *> prevSolution_;

            /// \brief The best solution cost we have found so far.
            base::Cost prevSolutionCost_{std::numeric_limits<double>::quiet_NaN()};

            /// \brief The number of solutions allowed until the most optimal solution is returned.
            int batchSize_{1};
        };
    }
}

#endif