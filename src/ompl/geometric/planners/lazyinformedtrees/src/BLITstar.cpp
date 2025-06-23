/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, University of New Hampshire
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
 *   * Neither the names of the copyright holders nor the names of its
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

// Authors: Yi Wang, Eyal Weiss, Bingxian Mu, Oren Salzman

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"
#include "ompl/geometric/planners/lazyinformedtrees/BLITstar.h"

#include <algorithm>
#include <cmath>
#include <string>

#include <boost/range/adaptor/reversed.hpp>

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/Console.h"

using namespace std::string_literals;
using namespace ompl::geometric::blitstar;
namespace ob = ompl::base;
using namespace std;
namespace ompl
{
    namespace geometric
    {
        BLITstar::BLITstar(const ompl::base::SpaceInformationPtr &spaceInformation)
          : ompl::base::Planner(spaceInformation, "BLITstar")
          , detectionState_(spaceInformation->allocState())
          , solutionCost_()
          , graph_(solutionCost_)
          , forwardVertexQueue_([this](const auto &lhs, const auto &rhs) { return isVertexBetter(lhs, rhs); })
          , reverseVertexQueue_([this](const auto &lhs, const auto &rhs) { return isVertexBetter(lhs, rhs); })
          , space_(spaceInformation->getStateSpace())
        {
            // Specify BLIT*'s planner specs.
            specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
            specs_.multithreaded = false;
            specs_.approximateSolutions = true;
            specs_.optimizingPaths = true;
            specs_.directed = true;
            specs_.provingSolutionNonExistence = false;
            specs_.canReportIntermediateSolutions = true;
            spaceInformation_ = spaceInformation;
            // Register the setting callbacks.
            declareParam<bool>("use_k_nearest", this, &BLITstar::setUseKNearest, &BLITstar::getUseKNearest, "0,1");
            declareParam<double>("rewire_factor", this, &BLITstar::setRewireFactor, &BLITstar::getRewireFactor,
                                 "1.0:0.01:3.0");
            declareParam<std::size_t>("samples_per_batch", this, &BLITstar::setBatchSize, &BLITstar::getBatchSize,
                                      "1:1:1000");
            declareParam<bool>("use_graph_pruning", this, &BLITstar::enablePruning, &BLITstar::isPruningEnabled, "0,1");
            declareParam<std::size_t>("set_max_num_goals", this, &BLITstar::setMaxNumberOfGoals,
                                      &BLITstar::getMaxNumberOfGoals, "1:1:1000");

            // Register the progress properties.
            addPlannerProgressProperty("iterations INTEGER", [this]() { return std::to_string(numIterations_); });
            addPlannerProgressProperty("best cost DOUBLE", [this]() { return std::to_string(solutionCost_.value()); });
            addPlannerProgressProperty("state collision checks INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfStateCollisionChecks()); });
            addPlannerProgressProperty("edge collision checks INTEGER",
                                       [this]() { return std::to_string(numEdgeCollisionChecks_); });
            addPlannerProgressProperty("nearest neighbour calls INTEGER",
                                       [this]() { return std::to_string(graph_.getNumberOfNearestNeighborCalls()); });
        }
        
        BLITstar::~BLITstar()
        {
            si_->freeState(detectionState_);
        }
        
        void BLITstar::setup()
        {
            // Call the base-class setup.
            Planner::setup();

            // Check that a problem definition has been set.
            if (static_cast<bool>(Planner::pdef_))
            {
                // Default to path length optimization objective if none has been specified.
                if (!pdef_->hasOptimizationObjective())
                {
                    OMPL_WARN("%s: No optimization objective has been specified. Defaulting to path length.",
                              Planner::getName().c_str());
                    Planner::pdef_->setOptimizationObjective(
                        std::make_shared<ompl::base::PathLengthOptimizationObjective>(Planner::si_));
                }

                if (static_cast<bool>(pdef_->getGoal()))
                {
                    // If we were given a goal, make sure its of appropriate type.
                    if (!(pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION)))
                    {
                        OMPL_ERROR("BLIT* is currently only implemented for goals that can be cast to "
                                   "ompl::base::GOAL_SAMPLEABLE_GOAL_REGION.");
                        setup_ = false;
                        return;
                    }
                }

                // Pull the optimization objective through the problem definition.
                objective_ = pdef_->getOptimizationObjective();

                // Initialize the solution cost to be infinite.
                solutionCost_ = objective_->infiniteCost();
                
                iSolution_ = false;
                
                C_curr = objective_->infiniteCost();
                approximateSolutionCostToGoal_ = objective_->infiniteCost();

                // Pull the motion validator through the space information.
                motionValidator_ = si_->getMotionValidator();

                // Setup a graph.
                graph_.setup(si_, pdef_, &pis_); 
            }
            else
            {
                // BLIT* can't be setup without a problem definition.
                setup_ = false;
                OMPL_WARN("BLIT*: Unable to setup without a problem definition.");
            }
        }

        ompl::base::PlannerStatus::StatusType BLITstar::ensureSetup()
        {
            // Call the base planners validity check. This checks if the
            // planner is setup if not then it calls setup().
            checkValidity();

            // Ensure the planner is setup.
            if (!setup_)
            {
                OMPL_ERROR("%s: The planner is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            // Ensure the space is setup.
            if (!si_->isSetup())
            {
                OMPL_ERROR("%s: The space information is not setup.", name_.c_str());
                return ompl::base::PlannerStatus::StatusType::ABORT;
            }

            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        ompl::base::PlannerStatus::StatusType
        BLITstar::ensureStartAndGoalStates(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // If the graph currently does not have a start state, try to get one.
            if (!graph_.hasAStartState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If we could not get a start state, then there's nothing to solve.
                if (!graph_.hasAStartState())
                {
                    OMPL_WARN("%s: No solution can be found as no start states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_START;
                }
            }

            // If the graph currently does not have a goal state, we wait until we get one.
            if (!graph_.hasAGoalState())
            {
                graph_.updateStartAndGoalStates(terminationCondition, &pis_);

                // If the graph still doesn't have a goal after waiting, then there's nothing to solve.
                if (!graph_.hasAGoalState())
                {
                    OMPL_WARN("%s: No solution can be found as no goal states are available", name_.c_str());
                    return ompl::base::PlannerStatus::StatusType::INVALID_GOAL;
                }
            }

            // Would it be worth implementing a 'setup' or 'checked' status type?
            return ompl::base::PlannerStatus::StatusType::UNKNOWN;
        }

        void BLITstar::clear()
        {
            graph_.clear();
            numIterations_ = 0u;
            approximateSolutionCostToGoal_ = approximateSolutionCost_ = solutionCost_ = objective_->infiniteCost();
            Planner::clear();
            setup_ = false;
        }

        ompl::base::PlannerStatus BLITstar::solve(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Ensure that the planner and state space are setup before solving.
            auto status = ensureSetup();

            // Return early if the planner or state space are not setup.
            if (status == ompl::base::PlannerStatus::StatusType::ABORT)
            {
                return status;
            }

            // Ensure that the problem has start and goal states before solving.
            status = ensureStartAndGoalStates(terminationCondition);

            // Return early if the problem cannot be solved.
            if (status == ompl::base::PlannerStatus::StatusType::INVALID_START ||
                status == ompl::base::PlannerStatus::StatusType::INVALID_GOAL)
            {
                return status;
            }
            OMPL_INFORM("%s: Solving the given planning problem. The current best solution cost is %.4f", name_.c_str(),
                        solutionCost_.value());
                          
            // Iterate to solve the problem.
            while (!terminationCondition && !objective_->isSatisfied(solutionCost_))
            { 
                iterate(terminationCondition);
            } 
            // Someone might call ProblemDefinition::clearSolutionPaths() between invocations of Planner::sovle(), in
            // which case previously found solutions are not registered with the problem definition anymore.
            status = updateSolution();

            // Let the caller know the status.
            informAboutPlannerStatus(status);
            return status;
        }

        void BLITstar::getPlannerData(base::PlannerData &data) const
        {
            // base::PlannerDataVertex takes a raw pointer to a state. I want to guarantee, that the state lives as
            // long as the program lives.
            static std::set<std::shared_ptr<Vertex>,
                            std::function<bool(const std::shared_ptr<Vertex> &, const std::shared_ptr<Vertex> &)>>
                liveStates([](const auto &lhs, const auto &rhs) { return lhs->getId() < rhs->getId(); });

            // Fill the planner progress properties.
            Planner::getPlannerData(data);

            // Get the vertices.
            auto vertices = graph_.getVertices();

            // Add the vertices and edges.
            for (const auto &vertex : vertices)
            {
                // Add the vertex to the live states.
                liveStates.insert(vertex);

                // Add the vertex as the right kind of vertex.
                if (graph_.isStart(vertex))
                {
                    data.addStartVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else if (graph_.isGoal(vertex))
                {
                    data.addGoalVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }
                else
                {
                    data.addVertex(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()));
                }

                // If it has a parent, add the corresponding edge.
                if (vertex->hasForwardParent())
                {
                    data.addEdge(ompl::base::PlannerDataVertex(vertex->getState(), vertex->getId()),
                                 ompl::base::PlannerDataVertex(vertex->getForwardParent()->getState(),
                                                               vertex->getForwardParent()->getId()));
                }
            }
        }

        void BLITstar::setBatchSize(std::size_t batchSize)
        {
            batchSize_ = batchSize;
        }

        std::size_t BLITstar::getBatchSize() const
        {
            return batchSize_;
        }

        void BLITstar::setRewireFactor(double rewireFactor)
        {
            graph_.setRewireFactor(rewireFactor);
        }

        double BLITstar::getRewireFactor() const
        {
            return graph_.getRewireFactor();
        }


        void BLITstar::enablePruning(bool prune)
        {
            isPruningEnabled_ = prune;
        }

        bool BLITstar::isPruningEnabled() const
        {
            return isPruningEnabled_;
        }

        void BLITstar::setUseKNearest(bool useKNearest)
        {
            graph_.setUseKNearest(useKNearest);
        }

        bool BLITstar::getUseKNearest() const
        {
            return graph_.getUseKNearest();
        }

       void BLITstar::setMaxNumberOfGoals(unsigned int numberOfGoals)
        {
            graph_.setMaxNumberOfGoals(numberOfGoals);
        }

        unsigned int BLITstar::getMaxNumberOfGoals() const
        {
            return graph_.getMaxNumberOfGoals();
        }

        void BLITstar::clearReverseVertexQueue()
        {
            std::vector<blitstar::KeyVertexPair> reverseQueue;
            reverseVertexQueue_.getContent(reverseQueue);
            for (const auto &element : reverseQueue)
            {
                element.second->resetReverseVertexQueuePointer();
            }
            reverseVertexQueue_.clear();
        }

        void BLITstar::clearForwardVertexQueue()
        {
            std::vector<blitstar::KeyVertexPair> forwardQueue;
            forwardVertexQueue_.getContent(forwardQueue);
            for (const auto &element : forwardQueue)
            {
                element.second->resetForwardVertexQueuePointer();
            }
            forwardVertexQueue_.clear();
        }
        void BLITstar::informAboutNewSolution() const
        {
            OMPL_INFORM("%s (%u iterations): Found a new exact solution of cost %.4f. Sampled a total of %u states, %u "
                        "of which were valid samples (%.1f \%). Processed %u edges, %u of which were collision checked "
                        "(%.1f \%). The forward search tree has %u vertices, %u of which are start states. The reverse "
                        "search tree has %u vertices, %u of which are goal states.",
                        name_.c_str(), numIterations_, solutionCost_.value(), graph_.getNumberOfSampledStates(),
                        graph_.getNumberOfValidSamples(),
                        graph_.getNumberOfSampledStates() == 0u ?
                            0.0 :
                            100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                                     static_cast<double>(graph_.getNumberOfSampledStates())),
                        numProcessedEdges_, numEdgeCollisionChecks_,
                        numProcessedEdges_ == 0u ? 0.0 :
                                                   100.0 * (static_cast<float>(numEdgeCollisionChecks_) /
                                                            static_cast<float>(numProcessedEdges_)),
                        countNumVerticesInForwardTree(), graph_.getStartVertices().size(),
                        countNumVerticesInReverseTree(), graph_.getGoalVertices().size());
        }

        void BLITstar::informAboutPlannerStatus(ompl::base::PlannerStatus::StatusType status) const
        {
            switch (status)
            {
                case ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Found an exact solution of cost %.4f.", name_.c_str(),
                                numIterations_, solutionCost_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION:
                {
                    OMPL_INFORM("%s (%u iterations): Did not find an exact solution, but found an approximate "
                                "solution "
                                "of cost %.4f which is %.4f away from a goal (in cost space).",
                                name_.c_str(), numIterations_, approximateSolutionCost_.value(),
                                approximateSolutionCostToGoal_.value());
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::TIMEOUT:
                {
                    if (trackApproximateSolutions_)
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find any solution.", name_.c_str(), numIterations_);
                    }
                    else
                    {
                        OMPL_INFORM("%s (%u iterations): Did not find an exact solution, and tracking approximate "
                                    "solutions is disabled.",
                                    name_.c_str(), numIterations_);
                    }
                    break;
                }
                case ompl::base::PlannerStatus::StatusType::INFEASIBLE:
                case ompl::base::PlannerStatus::StatusType::UNKNOWN:
                case ompl::base::PlannerStatus::StatusType::INVALID_START:
                case ompl::base::PlannerStatus::StatusType::INVALID_GOAL:
                case ompl::base::PlannerStatus::StatusType::UNRECOGNIZED_GOAL_TYPE:
                case ompl::base::PlannerStatus::StatusType::CRASH:
                case ompl::base::PlannerStatus::StatusType::ABORT:
                case ompl::base::PlannerStatus::StatusType::TYPE_COUNT:
                {
                    OMPL_INFORM("%s (%u iterations): Unable to solve the given planning problem.", name_.c_str(),
                                numIterations_);
                }
            }

            OMPL_INFORM(
                "%s (%u iterations): Sampled a total of %u states, %u of which were valid samples (%.1f \%). "
                "Processed %u edges, %u of which were collision checked (%.1f \%). The forward search tree "
                "has %u vertices. The reverse search tree has %u vertices.",
                name_.c_str(), numIterations_, graph_.getNumberOfSampledStates(), graph_.getNumberOfValidSamples(),
                graph_.getNumberOfSampledStates() == 0u ?
                    0.0 :
                    100.0 * (static_cast<double>(graph_.getNumberOfValidSamples()) /
                             static_cast<double>(graph_.getNumberOfSampledStates())),
                numProcessedEdges_, numEdgeCollisionChecks_,
                numProcessedEdges_ == 0u ?
                    0.0 :
                    100.0 * (static_cast<float>(numEdgeCollisionChecks_) / static_cast<float>(numProcessedEdges_)),
                countNumVerticesInForwardTree(), countNumVerticesInReverseTree());
        }

        std::size_t BLITstar::countNumVerticesInReverseTree() const
        {
            std::size_t numVerticesInReverseTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isGoal(vertex) || vertex->hasReverseParent())
                {
                    ++numVerticesInReverseTree;
                }
            }
            return numVerticesInReverseTree;
        }
         
        std::size_t BLITstar::countNumVerticesInForwardTree() const
        {
            std::size_t numVerticesInForwardTree = 0u;
            auto vertices = graph_.getVertices();
            for (const auto &vertex : vertices)
            {
                if (graph_.isStart(vertex) || vertex->hasForwardParent())
                {
                    ++numVerticesInForwardTree;
                }
            }
            return numVerticesInForwardTree;
        }  
        
        void BLITstar::insertStartVerticesInForWardVertexQueue()
        {
            for (const auto &start : graph_.getStartVertices())
            {
                // Set the cost to come from the goal to identity and the expanded cost to infinity.
                start->setCostToComeFromStart(objective_->identityCost());
                start->setCostToComeFromGoal(objective_->infiniteCost());
                // Set the lower cost bound for start to go or to come
                start->setLowerCostBoundToStart(objective_->identityCost());
                start->setLowerCostBoundToGoal(lowerboundToGoal(start));
                
                start->setStartVertex();
                start->resetForwardExpanded();
                start->setForwardId(forwardId_);   
                // Create an element for the queue.
                blitstar::KeyVertexPair element({start->getLowerCostBoundToGoal(), objective_->identityCost()},
                                               start); 
                // Insert the element into the queue and set the corresponding pointer.
                auto forwardQueuePointer = forwardVertexQueue_.insert(element);
                start->setForwardVertexQueuePointer(forwardQueuePointer);
            } 
        }    
        
         
        void BLITstar::insertGoalVerticesInReverseVertexQueue()
        {
            for (const auto &goal : graph_.getGoalVertices())
            {
                // Set the cost to come from the goal to identity and the expanded cost to infinity.
                goal->setCostToComeFromGoal(objective_->identityCost());
                goal->setCostToComeFromStart(objective_->infiniteCost());
                goal->setLowerCostBoundToGoal(objective_->identityCost());
                goal->setLowerCostBoundToStart(lowerboundToStart(goal));
                goal->resetReverseExpanded();
                goal->setGoalVertex();
                goal->setReverseId(reverseId_); 
                // Create an element for the queue.
                blitstar::KeyVertexPair element({goal->getLowerCostBoundToStart(), objective_->identityCost()},
                                               goal);
                // Insert the element into the queue and set the corresponding pointer.
                auto reverseQueuePointer = reverseVertexQueue_.insert(element);
                goal->setReverseVertexQueuePointer(reverseQueuePointer);
            }
        }
        
        void BLITstar::lookingForBestNeighbor(ompl::base::Cost curMin_, size_t neighbor)
        {
              if(betterThan(curMin_,minimalneighbor_))
              {
                 minimalneighbor_ = curMin_;
                 bestNeighbor_ = neighbor;
              }
        }
        void BLITstar::bestNeighbor(ompl::base::Cost costToCome, ompl::base::Cost costToGoal, size_t neighbor)
        {
              ompl::base::Cost f_value = objective_->combineCosts(costToCome,costToGoal);
              lookingForBestNeighbor(f_value,neighbor);    
        }
        
        void BLITstar::insertOrUpdateInReverseVertexQueue(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal, bool couldMeet)
        {
          auto element = vertex->getReverseVertexQueuePointer();
          //Update it if it is in
          if(element) {
                 element->data.first = computeEstimatedPathCosts(CostToCome,CostToGoal);
                 reverseVertexQueue_.update(element);
          }
          else //Insert the pointer into the queue
          {
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(computeEstimatedPathCosts(CostToCome,CostToGoal),
                                                                                          vertex);
                // Insert the vertex into the queue, storing the corresponding pointer.
                auto backwardQueuePointer = reverseVertexQueue_.insert(element);
                vertex->setReverseVertexQueuePointer(backwardQueuePointer);
          }
          if(couldMeet)
             bestNeighbor(CostToCome,CostToGoal,vertex->getId());
        }   


        bool BLITstar::NeedMoreSamples()
        {  return isVertexEmpty_ ? true : false; }
        
        bool BLITstar::PathValidity(std::shared_ptr<Vertex> &vertex) 
        {
            bool found_validity = true;
            forwardInvalid_ = false;
            ForwardPathValidityChecking(vertex,found_validity);
            reverseInvalid_= false;
            ReversePathValidityChecking(vertex,found_validity);            
            if(forwardInvalid_)
            {
                   forwardId_++;
                   clearForwardVertexQueue();
                   insertStartVerticesInForWardVertexQueue();
            }
            if(reverseInvalid_)
            {
                   reverseId_++; 
                   clearReverseVertexQueue();
                   insertGoalVerticesInReverseVertexQueue();
            }
            return found_validity;          
        }
        void BLITstar::ForwardPathValidityChecking(std::shared_ptr<Vertex> &vertex, bool &validity)
        {
                 std::shared_ptr<Vertex> tar_ = vertex; // target vertex
                 std::vector<std::shared_ptr<Vertex>> reversePath;
                 while (!graph_.isStart(tar_))
                 {
                       std::shared_ptr<Vertex> src_ = tar_->getForwardParent();// source vertex
                       reversePath.emplace_back(tar_);
                       bool valid_edge = false;
                       if(!tar_->hasForwardParent())
		        {
		            forwardInvalid_ = !(validity =  false);
		            break;
		        }
		        
		        if(!(valid_edge = CCD(make_pair(src_,tar_))))
		        {
                                forwardInvalid_ = !(validity =  false);
                                resetForwardParentAndRemeberTheVertex(tar_,src_);
                                tar_->resetReverseEdgeParent();
		        }
		        else
		        {
		           src_->setReverseValidParent(tar_,tar_->getEdgeCostFromForwardParent());
		        }
		        tar_ = src_;   
                 }
        }
        
        void BLITstar::ReversePathValidityChecking(std::shared_ptr<Vertex> &vertex, bool &validity)
        {
                 std::shared_ptr<Vertex> tar_ = vertex;
                 std::vector<std::shared_ptr<Vertex>> reversePath;
                 while (!graph_.isGoal(tar_))
                 {
                     reversePath.emplace_back(tar_);
                     std::shared_ptr<Vertex> src_ = tar_->getReverseParent();
                     bool valid_edge = false;
                      if(!tar_->hasReverseParent())
	              {  
		                  reverseInvalid_ = !(validity =  false);
		                  break;
		      }
                     if(!(valid_edge = CCD(make_pair(tar_,src_))))
                     {
                            reverseInvalid_ = !(validity =  false);
                            resetReverseParentAndRemeberTheVertex(tar_,src_);
                            tar_->resetForwardEdgeParent();
                     }
                     else
                     {
                         src_->setForwardValidParent(tar_,tar_->getEdgeCostFromReverseParent());  
                     }
                     tar_ = src_;                
                 }   
        }
        
        void BLITstar::resetForwardParentInformation(const std::shared_ptr<blitstar::Vertex> & vertex)
        {
                 vertex->resetForwardParent();
                 resetForwardValue(vertex);
        }
        
        void BLITstar::resetForwardValue(const std::shared_ptr<Vertex> & vertex)
        {
            vertex->resetForwardExpanded();
            vertex->resetForwardVertexQueuePointer();
            vertex->setCostToComeFromStart(objective_->infiniteCost());
        }

        void BLITstar::resetForwardParentAndRemeberTheVertex(const std::shared_ptr<Vertex> &child, const std::shared_ptr<Vertex> &parent)
        {      
                 child->setNearObstacle();
                 parent->setNearObstacle(); 
                 resetForwardParentInformation(child);
                 child->setForwardInvalid();
                 parent->removeFromForwardChildren(child->getId());  
        }
        
        void BLITstar::resetReverseParentInformation(const std::shared_ptr<blitstar::Vertex> &vertex)
        {
                 vertex->resetReverseParent();
                 resetReverseValue(vertex);
        }
        
        void BLITstar::resetReverseValue(const std::shared_ptr<Vertex> &vertex)
        {
            vertex->resetReverseExpanded();
            vertex->resetReverseVertexQueuePointer(); 
            vertex->setCostToComeFromGoal(objective_->infiniteCost());
        }

        void BLITstar::resetReverseParentAndRemeberTheVertex(const std::shared_ptr<Vertex> &child, const std::shared_ptr<Vertex> &parent)
        { 
                 child->setNearObstacle();
                 parent->setNearObstacle(); 
                 child->setReverseInvalid();
                 resetReverseParentInformation(child);
                 parent->removeFromReverseChildren(child->getId());
        }                 
        bool BLITstar::terminateSearch()
        {
             if(isVertexEmpty_)
             {  return true; }
             if(!found_meeting_ && betterThan(fmin_,C_curr))
             { return false; } 
             
             if(betterThan(C_curr,solutionCost_) && !PathValidity(V_meet.second))
             {
                 C_curr = solutionCost_;
                 BestVertex_->resetMeet();
                 V_meet.second->resetMeet();
                 start_scratch_ = true;
                 return (found_meeting_ = false);
             }   
             return (find_solution_ = true);
        }
                     
        void BLITstar::iterate(const ompl::base::PlannerTerminationCondition &terminationCondition)
        {
            // Add new samples to the graph, respecting the termination condition, if needed. 
            if (NeedMoreSamples() && graph_.addSamples(batchSize_, terminationCondition,need_Prune_))
            {
                    // Expanding start and goal vertices simultaneously
                    insertStartVerticesInForWardVertexQueue();
                    insertGoalVerticesInReverseVertexQueue();
                    
                    //count the scratch_
                    forwardId_ = reverseId_ = 0u; 
                    isVertexEmpty_ = find_solution_ = false;
            }
            // Selecte a state with the minimal priority on both queues for expansion. */ 
            bool forwardDirection_ = false;
            if(!SelectExpandState(forwardDirection_))
            {   return;  }

            bool terminateSearch_ =  terminateSearch();
            if(start_scratch_)
            {return;    }
            // If terminate function has not yet been triggered or either queue is empty, continuing the search in either direction
            if(!terminateSearch_)
            {
                 if(forwardDirection_) 
                 {   
                        BestVertex_->setForwardExpanded();
                        ForwardLazySearch(BestVertex_);    
                 }
                 else
                 {    
                        BestVertex_->setReverseExpanded(); 
                        ReverseLazySearch(BestVertex_);    
                 }
            }
            else
            {
                   if(find_solution_ && betterThan(C_curr, solutionCost_))
                   { 
                          iSolution_ = true;
                          solutionCost_ = C_curr;
                          path_= getPathToVertex(V_meet.second);   
                   } 
                   clearReverseVertexQueue();
                   clearForwardVertexQueue();
                   need_Prune_ = !searchExhausted_;
                   found_meeting_ = !(isVertexEmpty_ = true);                      
            }
            // Keep track of the number of iterations.
            ++numIterations_;
        }
        
        bool BLITstar::betterThan(const ompl::base::Cost & cost1, const ompl::base::Cost & cost2)
        {
            return cost1.value() + 0.000001 < cost2.value();
        }  
         
        bool BLITstar::largerThan(const ompl::base::Cost & cost1, const ompl::base::Cost & cost2)
        {
            return cost1.value()  > cost2.value()+ 0.000001;
        }  
         
        bool BLITstar::isVertexBetter(const blitstar::KeyVertexPair &lhs, const blitstar::KeyVertexPair &rhs) const
        {
                // it's a regular comparison of the keys.
                return std::lexicographical_compare(
                    lhs.first.cbegin(), lhs.first.cend(), rhs.first.cbegin(), rhs.first.cend(),
                    [this](const auto &a, const auto &b) { return objective_->isCostBetterThan(a, b); });
        }
               
        void BLITstar::ForwardLazySearch(const std::shared_ptr<blitstar::Vertex> &vertex) 
        {   
            // Operation in forward search, with analogous operation in reverse search.
            minimalneighbor_ = objective_->infiniteCost();
            // Insert current children of this vertex into forward search tree.  
            for (const auto &child : vertex->getForwardChildren())
            {      
                    // Checks collision detection if the first valid solution has not been found or the promising edge is near obstacles.
                    if((!iSolution_ || vertex->nearObstacle() || child->nearObstacle()) && !SCD(make_pair(vertex,child)))
                    {
                            resetForwardParentAndRemeberTheVertex(child,vertex);
                            continue;
                    }

                    // g_hat(x_v) = g(x_u) + c(x_u, x_v) 
                    auto edgeCost = child->getEdgeCostFromForwardParent();
                    auto gValue =  objective_->combineCosts(vertex->getCostToComeFromStart(),edgeCost);
                    if(iSolution_ && !child->hasReverseParent() && largerThan(objective_->combineCosts(gValue,gValue),solutionCost_))
                    {      continue;   }
                    // h_bar(x_v): lower cost bound to go such as Eculidean distance. 
                    auto hValue = child->getLowerCostBoundToGoal();       
                         child->setCostToComeFromStart(gValue); 
                         gValue = child->getCostToComeFromStart();
                    // Refine heuristic value if needed.     
                         updateForwardCost(child, gValue, hValue);
                         insertOrUpdateInForwardVertexQueue(child,gValue,hValue,vertex->meetVertex());            
            }
            
            // Add a new promising vertex to the forward search tree.
            bool Potential_collide_ = false;
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {    
                if(neighbor->getId() == vertex->getId())
                   continue; 
                if(iSolution_ && !vertex->nearObstacle() && neighbor->nearObstacle())
                {
                     Potential_collide_ = true;  
                }
                
                // Reset value metrics if the neighbor belongs to an obsolete branch of the forward search tree.      
                if(forwardInvalid_ && neighbor->getForwardId() != forwardId_)
                {
                     neighbor->setCostToComeFromStart(objective_->infiniteCost());
                     neighbor->resetForwardExpanded();  
                }
                if (!neighbor->isForwardExpanded())
                { 
                    // If x_u is the parent of x_v, it will not be added to the Q_F.
                    if(iSolution_ && vertex->hasReverseEdgeParent() && vertex->getReverseEdgeParent()->getId() == neighbor->getId())
                    {    continue;  }
                    
                    if(neighbor->hasForwardParent() && neighbor->getForwardParent()->getId() == vertex->getId())
                    {    continue;  }
                    
                    if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                    {    continue;  }
                    // Check whether start and goal are very close.
                    if(vertex->isStart() &&neighbor->isGoal() )
                    {
                             EvaluateValidityStartAndToGoal(neighbor,vertex); 
                             continue;
                    }
                    
                    //g_hat(x_v) =  g_hat(x_u) + c_hat(x_u,x_v).
                    auto gValue = (neighbor->hasForwardParent())? neighbor->getCostToComeFromStart() : objective_->infiniteCost();
                    auto edgeCost = objective_->motionCostHeuristic(neighbor->getState(), vertex->getState());
                    auto est_gValue = objective_->combineCosts(vertex->getCostToComeFromStart(), edgeCost);
                    
                    // If g_F(x_v) > g_F(x_u) + c_hat(x_u,x_v), this neighbor is a promising vertex to provide a better solution. 
                    if (betterThan(est_gValue, gValue)&& !neighbor->isGoal() )
                    { 
                         neighbor->setForwardId(forwardId_);
                         bool fValid_ = false, rValid_ = false, supposeMeet = false;
                         
                         // Use SCD (Sparse Collision Detection) to check edge validity before a solution is found.
                         if(!iSolution_ && !SCD(make_pair(vertex,neighbor)) )
                         {
                           vertex->setNearObstacle();
                           neighbor->setNearObstacle();
                           neighbor->setForwardInvalid();
                           neighbor->resetForwardExpanded();
                           continue;
                         }
                         // Check whether this promising edge is near obstacles
                         bool startCollid_ = goalCloseToStart_ && vertex->isStart();
                         if(startCollid_ || Potential_collide_ || vertex->nearObstacle() || neighbor->nearObstacle())
                         {
                           if(!(fValid_ = SCD(make_pair(vertex,neighbor))) || (neighbor->forwardInvalid() && !CCD(make_pair(vertex,neighbor)))) 
                           {  
                                neighbor->setNearObstacle();
                                vertex->setNearObstacle();
                                neighbor->setForwardInvalid();
                                neighbor->resetForwardExpanded();
                                continue;
                           }
                           if(startCollid_)
                           { neighbor->setNearObstacle(); }
                         }
                         
                         // Set x_u to be x_v's parent and update g_hat(x_v)
                         neighbor->setCostToComeFromStart(est_gValue);
                         neighbor->setForwardVertexParent(vertex,edgeCost);  
                         vertex->addToForwardChildren(neighbor);
                         auto hValue = neighbor->getLowerCostBoundToGoal();
                         auto getReversepointer = neighbor->getReverseVertexQueuePointer();  
                         
                         bool onReverseTree = !(neighbor->hasReverseParent() && neighbor->getReverseId() != reverseId_);  
                         if(!onReverseTree)
                         { resetReverseValue(neighbor); }
                         
                         // Check if x_v already exists in reverse search tree. If so, operate associated updating
                         if(onReverseTree && (getReversepointer || (neighbor->hasReverseParent())))
                         {  
                            keyEdgePair fEdge = make_pair(vertex,neighbor);
                            keyEdgePair rEdge = make_pair(neighbor,neighbor->getReverseParent());
                            if(!iSolution_)
                            {
                                 if(!fValid_) 
                                 {  fValid_ = SCD(fEdge); }
                                 rValid_ = SCD(rEdge);
                            }
                            
                            // If the promising edge could be valid and offer a better solution, operate associated updating
                            if((fValid_ && rValid_) || iSolution_) 
                            {                                     
                                    auto Totalcost = objective_->combineCosts(est_gValue,neighbor->getCostToComeFromGoal());
                                    lookingForBestNeighbor(Totalcost, neighbor->getId());
                                    supposeMeet = true;
                                    if(betterThan(Totalcost,C_curr)&& (iSolution_ || ((fValid_ = CCD(fEdge)) && (rValid_=CCD(rEdge)))) )//
                                    {
                                           updateBestSolutionFoundSoFar(neighbor,Totalcost,est_gValue,hValue,neighbor->getCostToComeFromGoal());
                                           insertOrUpdateInForwardVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()); 
                                           continue;
                                    }
                            }
                            
                            // Label states if the promising edge is invalid. 
                            if(!iSolution_ && (!fValid_ || !rValid_)) 
                            {
                                    neighbor->setNearObstacle(); 
                                    auto backwardParent = neighbor->getReverseParent();
                                    if(!fValid_) 
                                    {
                                        resetForwardParentAndRemeberTheVertex(neighbor, vertex);  
                                    } 
                                    else 
                                    {
                                            updateCostToGo(est_gValue, hValue,hValue,false);  
                                            insertOrUpdateInForwardVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()); 
                                    }
                                    if(!rValid_)
                                    {   resetReverseParentAndRemeberTheVertex(neighbor, backwardParent); }
                                    continue;
                            }
                         }
                         
                         // Refine the heuristic value on-the-fly and insert it into the forward queue
                         updateCostToGo(est_gValue, hValue,hValue,false);  
                         if(iSolution_ && !neighbor->hasReverseParent() && largerThan(hValue,solutionCost_))
                         {      continue;   }
                         insertOrUpdateInForwardVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()&& !supposeMeet);   
                    } 
                }
            }
            
            // Check whether the current vertex is the meeting vertex.
            found_meeting_ = (vertex->meetVertex() && betterThan(minimalneighbor_,C_curr)) ? true : false;
            
            // Check if its valid reverse parent can offer a better solution      
            if(iSolution_ && vertex->hasReverseEdgeParent())
            {
                  auto reverseParent = vertex->getReverseEdgeParent();
                  auto edgeCost = vertex->getValidReverseEdgeCost();
                  auto curValue = objective_->combineCosts(vertex->getCostToComeFromStart(),edgeCost);
                  auto hValue = reverseParent->getLowerCostBoundToGoal();
                  if(betterThan(curValue,reverseParent->getCostToComeFromStart()))
                  {
                         reverseParent->setCostToComeFromStart(curValue);  
                         reverseParent->setForwardVertexParent(vertex,edgeCost);
                         vertex->resetReverseEdgeParent();
                         vertex->addToForwardChildren(reverseParent);
                  }
                  updateForwardCost(reverseParent, reverseParent->getCostToComeFromStart(), hValue);
                  insertOrUpdateInForwardVertexQueue(reverseParent,reverseParent->getCostToComeFromStart(),hValue,vertex->meetVertex()); 
            }
        } 
        
        void BLITstar::updateBestSolutionFoundSoFar(const std::shared_ptr<Vertex> &vertex, ompl::base::Cost meetCost, ompl::base::Cost costToCome, ompl::base::Cost &costToGo, ompl::base::Cost costFromOriginal)
        {
                           if(V_meet.second)
                           {  V_meet.second->resetMeet(); }
                           vertex->setMeet();
                           V_meet = make_pair(C_curr=meetCost,vertex);
                           updateCostToGo(costToCome, costToGo,costFromOriginal,true); 
        }
        void BLITstar::updateCostToGo(ompl::base::Cost &costToCome, ompl::base::Cost &costToGo, ompl::base::Cost costFromOriginal, bool meetOnPath)
        {
                    if(betterThan(costToGo,costToCome))
                    {
                            costToGo =   costToCome;
                    }
                    if(meetOnPath && betterThan(costFromOriginal,costToGo))
                    {
                            costToGo = costFromOriginal;
                    }
           
        }
        
        void BLITstar::updateForwardCost(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo)
        {
                         vertex->resetForwardExpanded();
                         vertex->setForwardId(forwardId_); 
                         if(vertex->hasReverseParent() && vertex->getReverseId() == reverseId_)
                         {
                             auto bettersolution_ = objective_->combineCosts(vertex->getCostToComeFromGoal(),vertex->getCostToComeFromStart());
                             if(betterThan(bettersolution_,C_curr))
                             {
                                updateBestSolutionFoundSoFar(vertex,bettersolution_,costToCome,costToGo,vertex->getCostToComeFromGoal());
                             }
                             else
                             {
                                updateCostToGo(costToCome, costToGo,vertex->getCostToComeFromGoal(),true);
                             }  
                         }
                         else
                         {
                             updateCostToGo(costToCome, costToGo,costToGo,false); 
                         }   
        }
        
        void BLITstar::updateReverseCost(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost costToCome, ompl::base::Cost &costToGo)
        {
                         vertex->resetReverseExpanded();
                         vertex->setReverseId(reverseId_);
                         if(vertex->hasForwardParent() && vertex->getForwardId() == forwardId_)
                         {
                             auto bettersolution_ = objective_->combineCosts(vertex->getCostToComeFromGoal(),vertex->getCostToComeFromStart());
                             if(betterThan(bettersolution_,C_curr))
                             {
                                 updateBestSolutionFoundSoFar(vertex,bettersolution_,costToCome,costToGo,vertex->getCostToComeFromStart());
                             }
                             else 
                             {
                                updateCostToGo(costToCome,costToGo,vertex->getCostToComeFromStart(),true);
                             } 
                         }
                         else
                         {
                             updateCostToGo(costToCome,costToGo,costToGo,false);
                         }
        }
        
        
        void BLITstar::EvaluateValidityStartAndToGoal(const std::shared_ptr<Vertex> &start, const std::shared_ptr<Vertex> &goal)
        {
                      goalCloseToStart_ = true;
                      if(CCD(make_pair(start,goal)))
                      {
                            solutionCost_ = objective_->motionCost(start->getState(), goal->getState()); 
                            start->setReverseVertexParent(goal,solutionCost_);    
                            goal->setForwardVertexParent(start,solutionCost_);     
                      }
        }
              
        void BLITstar::ReverseLazySearch(const std::shared_ptr<blitstar::Vertex> &vertex) 
        { 
            minimalneighbor_ = objective_->infiniteCost();
            for (const auto &child : vertex->getReverseChildren())
            {     
                    if((!iSolution_ || vertex->nearObstacle()  || child->nearObstacle()) && !SCD(make_pair(child,vertex)))
                    {
                            resetReverseParentAndRemeberTheVertex(child,vertex);
                            continue;
                    }
                    auto edgeCost = child->getEdgeCostFromReverseParent() ;
                    auto gValue = objective_->combineCosts(vertex->getCostToComeFromGoal(),edgeCost);
                    if(iSolution_ && !child->hasForwardParent() && largerThan(objective_->combineCosts(gValue,gValue),solutionCost_))
                    {      continue;   }
                    auto hValue = child->getLowerCostBoundToStart(); 
                         child->setCostToComeFromGoal(gValue);                                              
                         gValue = child->getCostToComeFromGoal();
                         updateReverseCost(child,gValue,hValue);
                         insertOrUpdateInReverseVertexQueue(child,gValue,hValue,vertex->meetVertex()); 
            }
            bool Potential_collide_ = false;
            for (const auto &neighbor : graph_.getNeighbors(vertex))
            {  
                if(neighbor->getId() == vertex->getId())
                {
                     continue;
                }
                if(iSolution_ && !vertex->nearObstacle() && neighbor->nearObstacle())
                {
                     Potential_collide_ = true;  
                }                    
                if(reverseInvalid_ && neighbor->getReverseId() != reverseId_)
                {
                     neighbor->resetReverseExpanded();
                     neighbor->setCostToComeFromGoal(objective_->infiniteCost());    
                } 
                if (!neighbor->isReverseExpanded())
                { 
                    if(iSolution_ && vertex->hasForwardEdgeParent() && neighbor->getId() == vertex->getForwardEdgeParent()->getId())
                    {  continue;   }
                    if(vertex->isGoal() && neighbor->isStart())
                    {
                        EvaluateValidityStartAndToGoal(vertex,neighbor);
                        continue;
                    } 
                    if(neighbor->hasReverseParent() && neighbor->getReverseParent()->getId() == vertex->getId())
                    {   continue;  }

                    if (neighbor->isBlacklistedAsChild(vertex) || vertex->isBlacklistedAsChild(neighbor))
                    {   continue;  }
                    auto gValue = neighbor->hasReverseParent() ? neighbor->getCostToComeFromGoal() :objective_->infiniteCost();
                    auto edgeCost = objective_->motionCostHeuristic(neighbor->getState(), vertex->getState());
                    auto est_gValue = objective_->combineCosts(vertex->getCostToComeFromGoal(), edgeCost);   
                    if (betterThan(est_gValue, gValue) && !neighbor->isStart())
                    {
                         neighbor->setReverseId(reverseId_);
                         if(!iSolution_ && !SCD(make_pair(neighbor,vertex)))
                         {
                            vertex->setNearObstacle();
                            neighbor->setNearObstacle();
                            neighbor->setReverseInvalid();
                            neighbor->resetReverseExpanded();
                            continue;
                         }
                         bool fValid_ = false, rValid_ = false, supposeMeet = false;
                         bool goalCollid_ = goalCloseToStart_ && vertex->isGoal();
                         if(goalCollid_ || Potential_collide_ || vertex->nearObstacle() || neighbor->nearObstacle())
                         {
                           if(!(rValid_ = SCD(make_pair(neighbor,vertex))) || (neighbor->reverseInvalid()&& !CCD(make_pair(neighbor,vertex)))) 
                           {  
                                neighbor->setNearObstacle();
                                vertex->setNearObstacle();
                                neighbor->setReverseInvalid();
                                neighbor->resetReverseExpanded();
                                continue;
                           }
                           if(goalCollid_)
                           { neighbor->setNearObstacle();}
                         }
                         neighbor->setCostToComeFromGoal(est_gValue); 
                         neighbor->setReverseVertexParent(vertex,edgeCost);
                         vertex->addToReverseChildren(neighbor);
                         auto hValue = neighbor->getLowerCostBoundToStart();
                         auto getForwardpointer = neighbor->getForwardVertexQueuePointer(); 
                         bool onForwardTree = !(neighbor->hasForwardParent() && neighbor->getForwardId() != forwardId_);
                         if(!onForwardTree)
                         { resetForwardValue(neighbor);}          
                         
                         if(onForwardTree && (getForwardpointer || neighbor->hasForwardParent()))
                         { 
                            keyEdgePair rEdge = make_pair(neighbor,vertex);
                            keyEdgePair fEdge = make_pair(neighbor->getForwardParent(),neighbor);
                            if(!iSolution_)
                            {                                
                                if(!rValid_)
                                { rValid_ = SCD(rEdge);  }
                                fValid_ = SCD(fEdge);
                            }                           
                            if(iSolution_ || (fValid_ && rValid_)) 
                            {
                                auto Totalcost = objective_->combineCosts(est_gValue,neighbor->getCostToComeFromStart()); 
                                supposeMeet = true;
                                lookingForBestNeighbor(Totalcost, neighbor->getId());
                                if(betterThan(Totalcost,C_curr)&&(iSolution_|| ((fValid_ = CCD(fEdge))&&(rValid_=CCD(rEdge)))))//
                                {
                                     updateBestSolutionFoundSoFar(neighbor,Totalcost,est_gValue,hValue,neighbor->getCostToComeFromStart());
                                     insertOrUpdateInReverseVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex());
                                     continue;
                                }
                            }
                            if(!iSolution_ && (!fValid_ || !rValid_)) 
                            {
                                      neighbor->setNearObstacle();
                                      auto forwardParent = neighbor->getForwardParent();
                                      if(!fValid_) 
                                      {
                                                resetForwardParentAndRemeberTheVertex(neighbor, forwardParent);
                                      } 
                                        
                                      if(!rValid_)
                                      {
                                                resetReverseParentAndRemeberTheVertex(neighbor, vertex);    
                                      }
                                      else 
                                      {
                                                updateCostToGo(est_gValue, hValue,hValue,false); 
                                                insertOrUpdateInReverseVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()); 
                                      }
                                      continue; 
                            }
                         } 
                         updateCostToGo(est_gValue, hValue,hValue,false);
                         if(iSolution_&& !neighbor->hasForwardParent()  && largerThan(hValue,solutionCost_))
                         {      continue;   }  
                         insertOrUpdateInReverseVertexQueue(neighbor,est_gValue,hValue,vertex->meetVertex()&& !supposeMeet); 
                    } 
                }
            }
            found_meeting_ = (vertex->meetVertex() && betterThan(minimalneighbor_,C_curr)) ? true : false; 
            if(iSolution_ && vertex->hasForwardEdgeParent())
            {
                  auto forwardParent = vertex->getForwardEdgeParent();
                  auto edgeCost = vertex->getValidForwardEdgeCost(); 
                  auto curValue = objective_->combineCosts(vertex->getCostToComeFromGoal(),edgeCost);
                  auto hValue = forwardParent->getLowerCostBoundToStart();
                  if(betterThan(curValue,forwardParent->getCostToComeFromGoal()))
                  {
                         forwardParent->setCostToComeFromGoal(curValue); 
                         vertex->resetForwardEdgeParent();
                         forwardParent->setReverseVertexParent(vertex,edgeCost);
                         vertex->addToReverseChildren(forwardParent);
                  }
                  updateReverseCost(forwardParent,forwardParent->getCostToComeFromGoal(),hValue);                  
                  insertOrUpdateInReverseVertexQueue(forwardParent,forwardParent->getCostToComeFromGoal(),hValue,vertex->meetVertex()); 
            }
        }
        
          
        void BLITstar::insertOrUpdateInForwardVertexQueue(const std::shared_ptr<blitstar::Vertex> &vertex, ompl::base::Cost CostToCome, ompl::base::Cost CostToGoal, bool couldMeet)
        {
          //Get the pointer to the element in the queue
          auto element = vertex->getForwardVertexQueuePointer();
          //Update it if it is in
          if(element) {
                 element->data.first = computeEstimatedPathCosts(CostToCome,CostToGoal);
                 forwardVertexQueue_.update(element);
          }
          else //Insert the pointer into the queue
          {
                std::pair<std::array<ompl::base::Cost, 2u>, std::shared_ptr<Vertex>> element(computeEstimatedPathCosts(CostToCome,CostToGoal),
                                                                                             vertex);
                // Insert the vertex into the queue, storing the corresponding pointer.
                auto forwardQueuePointer = forwardVertexQueue_.insert(element);
                vertex->setForwardVertexQueuePointer(forwardQueuePointer);
          }
          if(couldMeet)
             bestNeighbor(CostToCome,CostToGoal,vertex->getId());
        }
           
        std::shared_ptr<ompl::geometric::PathGeometric> BLITstar::getPathToVertex(const std::shared_ptr<Vertex> &vertex) const
        {
            // Create the reverse path by following the parents in forward tree to the start from the meeting state.
            std::vector<std::shared_ptr<Vertex>> reversePath;
            auto current = vertex;
            while (!graph_.isStart(current))
            {
                reversePath.emplace_back(current);
                current = current->getForwardParent();
            }
            reversePath.emplace_back(current);

            // Reverse the reverse path to get the forward path.
            auto path = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);
            for (const auto &vertex_ : boost::adaptors::reverse(reversePath))
            {   
                path->append(vertex_->getState());
            }
            reversePath.clear();
            // Trace back the forward path by following the parents in reverse tree to goal from the meeting state.
            current = vertex; 
            while (!graph_.isGoal(current))
            {
                reversePath.emplace_back(current);
                current = current->getReverseParent();
            }
            reversePath.emplace_back(current);
            for (const auto &vertex_ : reversePath)
            {
                if(vertex_->getId() != vertex->getId())
                { path->append(vertex_->getState());  }   
            }
            return path;
        }

        std::array<ompl::base::Cost, 3u> BLITstar::computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal, ompl::base::Cost motionCost) const
        {
            // f = g(v) + c(v,w)+ h(w); g(x) = g(parent(x))+c(parent(x),x)
            return {objective_->combineCosts(CostToStart,CostToGoal),CostToStart, motionCost};
        }

        std::array<ompl::base::Cost, 2u> BLITstar::computeEstimatedPathCosts(ompl::base::Cost CostToStart, ompl::base::Cost CostToGoal) const
        {
            // f = g(x) + h(x); g(x) = g(parent(x))+c(parent(x),x)
            return {objective_->combineCosts(CostToStart,CostToGoal),CostToStart};
        }
                                
        void BLITstar::updateExactSolution()
        {      
                    // Create a solution.
                    ompl::base::PlannerSolution solution(path_);
                    solution.setPlannerName(name_);

                    // Set the optimized flag.
                    solution.setOptimized(objective_, solutionCost_, objective_->isSatisfied(solutionCost_));

                    // Let the problem definition know that a new solution exists.
                    pdef_->addSolutionPath(solution);

                    // Let the user know about the new solution.
                    informAboutNewSolution();
        }

        ompl::base::PlannerStatus::StatusType BLITstar::updateSolution()
        {
            updateExactSolution();
            
            if (objective_->isFinite(solutionCost_))
            {
                return ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION;
            }
            else if (trackApproximateSolutions_)
            {
                return ompl::base::PlannerStatus::StatusType::APPROXIMATE_SOLUTION;
            }
            else
            {
                return ompl::base::PlannerStatus::StatusType::TIMEOUT;
            }
        }

        ompl::base::Cost BLITstar::lowerboundToStart(const std::shared_ptr<Vertex> &vertex) const
        {
            const auto &start = graph_.getStartVertices()[0u];
            return objective_->motionCostHeuristic(vertex->getState(), start->getState());
        }

        ompl::base::Cost BLITstar::lowerboundToGoal(const std::shared_ptr<Vertex> &vertex) const
        {
            const auto &goal =  graph_.getGoalVertices()[0u];
            return objective_->motionCostHeuristic(vertex->getState(), goal->getState());
        }

               
        bool BLITstar::SelectExpandState(bool &forwardDirection_)//
        {
            searchExhausted_ = start_scratch_ = found_meeting_ = false;
            if(found_meeting_)
            {
                 return true;
            }
            
            // Check if the search is exhausted in either direction
            if(forwardVertexQueue_.empty() || reverseVertexQueue_.empty())
            {    
                 C_curr = solutionCost_;
                 return (isVertexEmpty_ = (searchExhausted_ = true));
            }   
            
            auto forwardVertex_ = forwardVertexQueue_.top()->data.second;
            auto backwardVertex_ = reverseVertexQueue_.top()->data.second;
            ForwardCost = forwardVertexQueue_.top()->data.first[0u];
            ReverseCost = reverseVertexQueue_.top()->data.first[0u];
            
            // Check if the best vertex is on an obsolete search direction
            if(forwardInvalid_ && forwardVertex_->getForwardId() != forwardId_)
            {
                forwardVertexQueue_.pop(); 
                resetForwardValue(forwardVertex_); 
                return false;
            }
            
            if(reverseInvalid_ && backwardVertex_->getReverseId() != reverseId_)
            {
                reverseVertexQueue_.pop();
                resetReverseValue(backwardVertex_);
                return false;   
            } 
             
            // Select the vertex with the lowest priority from both queues for expansion
            if(betterThan(ForwardCost, ReverseCost))
            {
                   BestVertex_ = forwardVertexQueue_.top()->data.second;
                   forwardVertexQueue_.pop();
                   BestVertex_->resetForwardVertexQueuePointer();
                   fmin_ = ForwardCost;
                   if(BestVertex_->isForwardExpanded())
                   {  return false; }
                   forwardDirection_ = true;
            } 
            else 
            {   
                   BestVertex_ = reverseVertexQueue_.top()->data.second;
                   reverseVertexQueue_.pop();
                   BestVertex_->resetReverseVertexQueuePointer();
                   fmin_ = ReverseCost;
                   if(BestVertex_->isReverseExpanded())
                   {return false;}
            }
            return true;
        }
        
        bool BLITstar::SCD(const keyEdgePair &edge)
        {
            return isValidAtResolution(edge, numSparseCollisionChecksCurrentLevel_,true);
        }
          
        bool BLITstar::CCD(const keyEdgePair &edge)
        {
            return isValidAtResolution(edge,space_->validSegmentCount(edge.first->getState(), edge.second->getState()),iSolution_);  
        }
        
        bool BLITstar::isValidAtResolution(const keyEdgePair &edge, std::size_t numChecks, bool sparseCheck)
        {
        
            auto parent = edge.first;
            auto child = edge.second;
            // Check if the edge is whitelisted.
            if (parent->isWhitelistedAsChild(child))
            {
                return true;
            }

            // If the edge is blacklisted.
            if (child->isBlacklistedAsChild(parent))
            {
                return false;
            }

            // Get the segment count for the full resolution.
            const std::size_t fullSegmentCount = space_->validSegmentCount(parent->getState(), child->getState());

            // The segment count is the number of checks on this level plus 1, capped by the full resolution segment
            // count.
            const auto segmentCount = std::min(numChecks + 1u, fullSegmentCount);
           
            // Store the current check number.
            std::size_t currentCheck = 1u;

            // Get the number of checks already performed on this edge.
            const std::size_t performedChecks = child->getIncomingCollisionCheckResolution(parent->getId());
            // Initialize the queue of positions to be tested.
            std::queue<std::pair<std::size_t, std::size_t>> indices;
            indices.emplace(1u, numChecks);
            
            // Test states while there are states to be tested.
            while (!indices.empty())
            {
                // Get the current segment.
                const auto current = indices.front();

                // Get the midpoint of the segment.
                auto mid = (current.first + current.second) / 2;
                // Only do the detection if we haven't tested this state on a previous level.
                if (currentCheck > performedChecks)
                {
                    space_->interpolate(parent->getState(), child->getState(),static_cast<double>(mid) / static_cast<double>(segmentCount), detectionState_);
                    if (!spaceInformation_->isValid(detectionState_))
                    {
                        // Blacklist the edge.
                        parent->blacklistAsChild(child);
                        child->blacklistAsChild(parent);
                        
                        // Update the maximum sparse collision detection resolution if a new level is reached
                        if(!sparseCheck && currentCheck > numSparseCollisionChecksCurrentLevel_)
                        {
                            numSparseCollisionChecksCurrentLevel_ = currentCheck+1u;
                        }
                        return false;
                    }
                }

                // Remove the current segment from the queue.
                indices.pop();

                // Create the first and second half of the split segment if necessary.
                if (current.first < mid)
                {
                    indices.emplace(current.first, mid - 1u);
                }

                if (current.second > mid)
                {
                    indices.emplace(mid + 1u, current.second);
                }

                // Increase the current check number.
                ++currentCheck;
            }

            // Remember at what resolution this edge was already checked. We're assuming that the number of collision
            // checks is symmetric for each edge.
            parent->setIncomingCollisionCheckResolution(child->getId(), currentCheck - 1u);
            child->setIncomingCollisionCheckResolution(parent->getId(), currentCheck - 1u);

            // Whitelist this edge if it was checked at full resolution.
            if (segmentCount == fullSegmentCount)
            { 
                ++numCollisionCheckedEdges_;
                parent->whitelistAsChild(child);
                child->whitelistAsChild(parent);
            }
            return true;
        }  
    }  // namespace geometric
}  
