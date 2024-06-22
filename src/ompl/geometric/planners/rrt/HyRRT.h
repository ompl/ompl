/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, University of Santa Cruz Hybrid Systems Laboratory
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

/* Author: Beverly Xu */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_HYRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_HYRRT_

#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/GoalTypes.h"
#include "ompl/base/Planner.h"

using namespace std;

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor HyRRT
           @par Brief description
           Hybrid RRT (HyRRT) is an RRT algorithm that solves separate optimization
           problems associated with the flows and jumps of the systems, to solve a
           variety of robotic motion planning problems. As an RRT algorithm, HyRRT is
           probabilistically-complete. The logical flow of the algorithm is as follows:
           1. Initialize the tree with a start state.
           2. While a solution has not been found:
                a. Sample a random state.
                b. Find the nearest state in the tree to the random state.
                c. Extend from that state under either the flow or jump regimes, using
           the continuous or discrete simulators, respectively. d. Continue until the
           state is in collision, or the maximum flow time has been exceeded. e. If the
           state is not within the unsafe set, add the state to the tree. If the state
                   is in collision, proceed to jump.
            3. Return the solution.

           @par External documentation: https://ieeexplore.ieee.org/document/9992444
           DOI: [10.1109/CDC51059.2022.9992444]
        */

        /** \brief Hybrid Rapidly-exploring Random Trees */
        class HyRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            HyRRT(const base::SpaceInformationPtr &si);
            ~HyRRT() override;
            void clear() override;
            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            // See ompl::RNG for more information about each distribution
            enum inputSamplingMethods_
            {
                UNIFORM_01,
                UNIFORM_REAL,
                UNIFORM_INT,
                GAUSSIAN_01,
                GAUSSIAN_REAL,
                HALF_NORMAL_REAL,
                HALF_NORMAL_INT,
                QUATERNION,
                EURLER_RPY
            };

            /// \brief Representation of a motion in the search tree
            class Motion
            {
            public:
                /// \brief Default constructor
                Motion() = default;

                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state (si->allocState()) {}

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief Pointer to the root of the tree this motion is
                /// contained in.
                const base::State *root{nullptr};

                /// \brief The integration steps defining the edge of the motion, between the parent and child vertices
                std::vector<base::State *> *edge{nullptr};
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Set the input range the planner is supposed to use. */
            void setFlowInputRange(std::vector<double> min, std::vector<double> max)
            {
                int size = max.size() > min.size() ? max.size() : min.size();
                if (min.size() != max.size())
                    throw Exception("Max input value (maxFlowInputValue) and min input value (minFlowInputValue) must be of the same size");
                for (int i = 0; i < size; i++)
                {
                    if (min.at(0) > max.at(0))
                        throw Exception("Max input value must be greater than or equal to min input value");
                }
                minFlowInputValue_ = min;
                maxFlowInputValue_ = max;
            }

            /** \brief Set the input range the planner is supposed to use. */
            void setJumpInputRange(std::vector<double> min, std::vector<double> max)
            {
                int size = max.size() > min.size() ? max.size() : min.size();
                if (min.size() != max.size())
                    throw Exception("Max input value (maxJumpInputValue_) and min input value (minJumpInputValue) must be of the same size");
                for (int i = 0; i < size; i++)
                {
                    if (min.at(i) > max.at(i))
                        throw Exception("Max input value must be greater than or equal to min input value");
                }
                minJumpInputValue_ = min;
                maxJumpInputValue_ = max;
            }

            /** \brief Set the maximum time allocated to a full continuous simulator step.
             * The duration must be greater than 0, and greater than than the time
             * allocated to a single continuous simulator call. */
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

            /** \brief Set the time allocated to a single continuous simulator call,
             * within the full period of a continuous simulator step. The duration must be
             * greater than 0, and less than the time allocated to a full continuous
             * simulator step. */
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

            /** \brief Set distance tolerance from goal state. Tolerance must be greater
             * than 0. */
            void setGoalTolerance(double tolerance)
            {
                if (tolerance < 0)
                    throw Exception("Goal tolerance must be greater than or equal to 0");
                tolerance_ = tolerance;
            }

            /** \brief Set the jump set. */
            void setJumpSet(std::function<bool(base::State *)> jumpSet)
            {
                jumpSet_ = jumpSet;
            }

            /** \brief Set the flow set. */
            void setFlowSet(std::function<bool(base::State *)> flowSet)
            {
                flowSet_ = flowSet;
            }

            /** \brief Set the unsafe set. */
            void setUnsafeSet(std::function<bool(base::State *)> unsafeSet)
            {
                unsafeSet_ = unsafeSet;
            }

            /** \brief Set the distance measurement function. */
            void setDistanceFunction(std::function<double(base::State *, base::State *)> function)
            {
                distanceFunc_ = function;
            }

            /** \brief Set the discrete dyanmics simulator. */
            void setDiscreteSimulator(std::function<base::State *(base::State *curState, std::vector<double> u, base::State *newState)> function)
            {
                discreteSimulator_ = function;
            }

            /** \brief Set the continuous dyanmics simulator. */
            void setContinuousSimulator(std::function<base::State *(std::vector<double> inputs, base::State *curState, double tFlowMax, 
                                        base::State *newState)> function)
            {
                continuousSimulator_ = function;
            }

            /** \brief Set the collision checker. */
            void setCollisionChecker(std::function<bool(std::vector<base::State *> *edge, std::function<bool(base::State *state)> obstacleSet, 
                                     double ts, double tf, base::State *newState, int tFIndex)> function)
            {
                collisionChecker_ = function;
            }

            /** \brief Set the flow input sampling mode. See
             * https://github.com/ompl/ompl/blob/main/src/ompl/util/RandomNumbers.h for
             * details on each available mode.
             */
            void setFlowInputSamplingMode(inputSamplingMethods_ mode, std::vector<double> inputs)
            {
                inputSamplingMethod_ = mode;

                unsigned int targetParameterCount = 0;

                switch (mode)
                {
                case UNIFORM_INT:
                    targetParameterCount = 2;
                    getRandFlowInput_ = [this](int i) { return randomSampler_->uniformInt(minFlowInputValue_[i], maxFlowInputValue_[i]); };
                    break;
                case GAUSSIAN_REAL:
                    targetParameterCount = 2;
                    getRandFlowInput_ = [&](int i) { return randomSampler_->gaussian(inputs[0], inputs[1]); };
                    break;
                case HALF_NORMAL_REAL:
                    targetParameterCount = 2; // Can also be three, if want to specify focus,
                                              // which defaults to 3.0
                    getRandFlowInput_ = [&](int i) { return randomSampler_->halfNormalReal(inputs[0], inputs[1], inputs[2]); };
                    break;
                default:
                    targetParameterCount = 2;
                    getRandFlowInput_ = [this](int i) { return randomSampler_->uniformReal(minFlowInputValue_[i], maxFlowInputValue_[i]); };
                }

                if (inputs.size() == targetParameterCount || (mode == HALF_NORMAL_INT && targetParameterCount == 3))
                    inputSamplingParameters_ = inputs;
                else
                    throw Exception("Invalid number of input parameters for input sampling mode.");
            }

            /** \brief Set the jump input sampling mode. See
             * https://github.com/ompl/ompl/blob/main/src/ompl/util/RandomNumbers.h for
             * details on each available mode.
             */
            void setJumpInputSamplingMode(inputSamplingMethods_ mode, std::vector<double> inputs)
            {
                inputSamplingMethod_ = mode;

                unsigned int targetParameterCount = 0;

                switch (mode)
                {
                case UNIFORM_INT:
                    targetParameterCount = 2;
                    getRandJumpInput_ = [this](int i) { return randomSampler_->uniformInt(minJumpInputValue_[i], maxJumpInputValue_[i]); };
                    break;
                case GAUSSIAN_REAL:
                    targetParameterCount = 2;
                    getRandJumpInput_ = [&](int i) { return randomSampler_->gaussian(inputs[0], inputs[1]); };
                    break;
                case HALF_NORMAL_REAL:
                    targetParameterCount = 2; // Can also be three parameters, if want to specify focus,
                                              // which defaults to 3.0
                    getRandJumpInput_ = [&](int i) { return randomSampler_->halfNormalReal(inputs[0], inputs[1], inputs[2]); };
                    break;
                default:
                    targetParameterCount = 2;
                    getRandJumpInput_ = [this](int i) { return randomSampler_->uniformReal(minJumpInputValue_[i], maxJumpInputValue_[i]); };
                }

                if (inputs.size() == targetParameterCount || (mode == HALF_NORMAL_INT && targetParameterCount == 3))
                    inputSamplingParameters_ = inputs;
                else
                    throw Exception("Invalid number of input parameters for input sampling mode.");
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors(void)
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
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
                if (!tM_)
                    throw Exception("Max flow propagation time (Tm) no set");
                if (maxJumpInputValue_.size() == 0)
                    throw Exception("Max input value (maxJumpInputValue) not set");
                if (minJumpInputValue_.size() == 0)
                    throw Exception("Min input value (minJumpInputValue) not set");
                if (maxFlowInputValue_.size() == 0)
                    throw Exception("Max input value (maxFlowInputValue) not set");
                if (minFlowInputValue_.size() == 0)
                    throw Exception("Min input value (minFlowInputValue) not set");
                if (!flowStepDuration_)
                    throw Exception("Flow step length (flowStepDuration_) not set");
            }

        protected:
            /** \brief Random sampler for the full vector of flow input. */
            std::function<std::vector<double>(void)> sampleFlowInputs_ = [this](void)
            {
                std::vector<double> u;
                for (unsigned int i = 0; i < maxFlowInputValue_.size(); i++)
                    u.push_back(getRandFlowInput_(i));
                return u;
            };

            /** \brief Random sampler for the the full vector of jump input. */
            std::function<std::vector<double>(void)> sampleJumpInputs_ = [this](void)
            {
                std::vector<double> u;
                for (unsigned int i = 0; i < maxJumpInputValue_.size(); i++)
                    u.push_back(getRandJumpInput_(i));
                return u;
            };

            /** \brief Random sampler for one value of the flow input. */
            std::function<double(int i)> getRandFlowInput_ = [this](int i) { return randomSampler_->uniformReal(minFlowInputValue_[i], maxFlowInputValue_[i]); };

            /** \brief Random sampler for one value of the jump input. */
            std::function<double(int i)> getRandJumpInput_ = [this](int i) { return randomSampler_->uniformReal(minJumpInputValue_[i], maxJumpInputValue_[i]); };

            /** \brief Collision checker. Optional is point-by-point collision checking
             * using the jump set. */
            std::function<bool(std::vector<base::State *> *edge,
                               std::function<bool(base::State *state)> obstacleSet,
                               double ts, double tf, base::State *newState, int tFIndex)>
                collisionChecker_ =
                    [this](std::vector<base::State *> *edge,
                           std::function<bool(base::State *state)> obstacleSet, double t = -1.0,
                           double tf = -1.0, base::State *newState, int tFIndex = -1) -> bool
            {
                for (unsigned int i = 0; i < edge->size(); i++)
                {
                    if (obstacleSet(edge->at(i)))
                    {
                        if (i == 0)
                            si_->copyState(newState, edge->at(i));
                        else
                            si_->copyState(newState, edge->at(i - 1));
                        return true;
                    }
                }
                return false;
            };

            /** \brief Name of input sampling method, default is "uniform" */
            inputSamplingMethods_ inputSamplingMethod_{UNIFORM_01};

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            base::StateSpacePtr setup_;

            /** \brief Runs the initial setup tasks for the tree. */
            void initTree(void);

            /** \brief Sample the random motion. */
            void randomSample(Motion *randomMotion);

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /**
             * The following are all customizeable parameters,
             * and affect how @b cHyRRT generates trajectories.
             * Customize using setter functions above. */

            /** \brief Compute distance between states, default is Euclidean distance */
            std::function<double(base::State *state1, base::State *state2)> distanceFunc_;

            /** \brief The maximum flow time for a given flow propagation step. Must be
             * set by the user */
            double tM_;

            /** \brief The distance tolerance from the goal state for a state to be
             * regarded as a valid final state. Default is .1 */
            double tolerance_{.1};

            double minStepLength = 1e-06;

            /** \brief The flow time for a given integration step, within a flow
             * propagation step. Must be set by user */
            double flowStepDuration_;

            /** \brief Minimum input value */
            std::vector<double> minFlowInputValue_;

            /** \brief Maximum input value */
            std::vector<double> maxFlowInputValue_;

            /** \brief Minimum input value */
            std::vector<double> minJumpInputValue_;

            /** \brief Maximum input value */
            std::vector<double> maxJumpInputValue_;

            /** \brief Simulator for propgation under jump regime */
            std::function<base::State *(base::State *curState, std::vector<double> u, base::State *newState)> discreteSimulator_;

            /** \brief Function that returns true if a state is in the jump set, and false
             * if not. */
            std::function<bool(base::State *state)> jumpSet_;

            /** \brief Function that returns true if a state is in the flow set, and false
             * if not. */
            std::function<bool(base::State *state)> flowSet_;

            /** \brief Function that returns true if a state is in the flow set, and false
             * if not. */
            std::function<bool(base::State *state)> unsafeSet_;

            /** \brief Simulator for propgation under flow regime */
            std::function<base::State *(std::vector<double> input, base::State *curState, double tFlowMax, base::State *newState)> continuousSimulator_;

            /** \brief Random sampler for the input. Default constructor always seeds a
             * different value, and returns a uniform real distribution. */
            RNG *randomSampler_ = new RNG();

            /** \brief Construct the path, starting at the last edge. */
            base::PlannerStatus constructSolution(Motion *lastMotion);

            /** \brief Name of input sampling method, default is "uniform" */
            std::vector<double> inputSamplingParameters_{};

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;
        };
    }
}

#endif