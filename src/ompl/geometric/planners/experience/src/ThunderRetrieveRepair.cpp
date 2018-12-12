/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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

/* Author: Dave Coleman */

#include <ompl/geometric/planners/experience/ThunderRetrieveRepair.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/util/Console.h>
#include <ompl/tools/thunder/ThunderDB.h>
#include "ompl/tools/config/MagicConstants.h"

#include <thread>

#include <limits>
#include <utility>

namespace ompl
{
    namespace geometric
    {
        ThunderRetrieveRepair::ThunderRetrieveRepair(const base::SpaceInformationPtr &si,
                                                     tools::ThunderDBPtr experienceDB)
          : base::Planner(si, "Thunder_Retrieve_Repair")
          , experienceDB_(std::move(experienceDB))
          , nearestK_(ompl::magic::NEAREST_K_RECALL_SOLUTIONS)  // default value
          , smoothingEnabled_(false)  // makes understanding recalled paths more difficult if enabled
        {
            specs_.approximateSolutions = true;
            specs_.directed = true;

            // Repair Planner Specific:
            repairProblemDef_ = std::make_shared<base::ProblemDefinition>(si_);

            path_simplifier_ = std::make_shared<PathSimplifier>(si_);
        }

        ThunderRetrieveRepair::~ThunderRetrieveRepair()
        {
            freeMemory();
        }

        void ThunderRetrieveRepair::clear()
        {
            Planner::clear();
            freeMemory();

            // Clear the inner planner
            if (repairPlanner_)
                repairPlanner_->clear();
        }

        void ThunderRetrieveRepair::setExperienceDB(const tools::ThunderDBPtr &experienceDB)
        {
            experienceDB_ = experienceDB;
        }

        void ThunderRetrieveRepair::setRepairPlanner(const base::PlannerPtr &planner)
        {
            if (planner && planner->getSpaceInformation().get() != si_.get())
                throw Exception("Repair planner instance does not match space information");
            repairPlanner_ = planner;
            setup_ = false;
        }

        void ThunderRetrieveRepair::setup()
        {
            Planner::setup();

            // Setup repair planner (for use by the rrPlanner)
            // Note: does not use the same pdef as the main planner in this class
            if (!repairPlanner_)
            {
                // Set the repair planner
                auto repair_planner(std::make_shared<RRTConnect>(si_));

                OMPL_DEBUG("No repairing planner specified. Using default: %s", repair_planner->getName().c_str());
                repairPlanner_ = repair_planner;  // Planner( repair_planer );
            }
            // Setup the problem definition for the repair planner
            repairProblemDef_->setOptimizationObjective(pdef_->getOptimizationObjective());  // copy primary problem def

            // Setup repair planner
            repairPlanner_->setProblemDefinition(repairProblemDef_);
            if (!repairPlanner_->isSetup())
                repairPlanner_->setup();
        }

        void ThunderRetrieveRepair::freeMemory()
        {
        }

        base::PlannerStatus ThunderRetrieveRepair::solve(const base::PlannerTerminationCondition &ptc)
        {
            bool solved = false;
            double approxdif = std::numeric_limits<double>::infinity();
            nearestPaths_.clear();

            // Check if the database is empty
            if (experienceDB_->isEmpty())
            {
                OMPL_INFORM("Experience database is empty so unable to run ThunderRetrieveRepair algorithm.");

                return base::PlannerStatus::ABORT;
            }

            // Restart the Planner Input States so that the first start and goal state can be fetched
            pis_.restart();

            // Get a single start and goal state TODO: more than one
            const base::State *startState = pis_.nextStart();
            const base::State *goalState = pis_.nextGoal(ptc);

            // Create solution path struct
            SPARSdb::CandidateSolution candidateSolution;

            // Search for previous solution in database
            // TODO make this more than 1 path
            if (!experienceDB_->findNearestStartGoal(nearestK_, startState, goalState, candidateSolution, ptc))
            {
                OMPL_INFORM("RetrieveRepair::solve() No nearest start or goal found");
                return base::PlannerStatus::TIMEOUT;  // The planner failed to find a solution
            }

            // Save this for future debugging
            nearestPaths_.push_back(candidateSolution.getGeometricPath());
            nearestPathsChosenID_ = 0;  // TODO not hardcode

            // All save trajectories should be at least 2 states long, then we append the start and goal states, for min
            // of 4
            assert(candidateSolution.getStateCount() >= 4);

            // Smooth the result
            if (smoothingEnabled_)
            {
                OMPL_INFORM("ThunderRetrieveRepair solve: Simplifying solution (smoothing)...");
                time::point simplifyStart = time::now();
                std::size_t numStates = candidateSolution.getGeometricPath().getStateCount();
                // ompl::geometric::PathGeometric pg = candidateSolution.getGeometricPath(); // TODO do not copy to new
                // type
                path_simplifier_->simplify(candidateSolution.getGeometricPath(), ptc);
                double simplifyTime = time::seconds(time::now() - simplifyStart);
                OMPL_INFORM("ThunderRetrieveRepair: Path simplification took %f seconds and removed %d states",
                            simplifyTime, numStates - candidateSolution.getGeometricPath().getStateCount());
            }

            // Finished
            approxdif = 0;
            bool approximate = candidateSolution.isApproximate_;

            pdef_->addSolutionPath(candidateSolution.path_, approximate, approxdif, getName());
            solved = true;
            return {solved, approximate};
        }

        bool ThunderRetrieveRepair::repairPath(const base::PlannerTerminationCondition &ptc, PathGeometric &primaryPath)
        {
            // \todo: we could reuse our collision checking from the previous step to make this faster
            //        but that complicates everything and I'm not suppose to be spending too much time
            //        on this prototype - DTC

            OMPL_INFORM("Repairing path ----------------------------------");

            // Error check
            if (primaryPath.getStateCount() < 2)
            {
                OMPL_ERROR("Cannot repair a path with less than 2 states");
                return false;
            }

            // Loop through every pair of states and make sure path is valid.
            // If not, replan between those states
            for (std::size_t toID = 1; toID < primaryPath.getStateCount(); ++toID)
            {
                std::size_t fromID = toID - 1;  // this is our last known valid state
                base::State *fromState = primaryPath.getState(fromID);
                base::State *toState = primaryPath.getState(toID);

                // Check if our planner is out of time
                if (ptc)
                {
                    OMPL_DEBUG("Repair path function interrupted because termination condition is true.");
                    return false;
                }

                // Check path between states
                if (!si_->checkMotion(fromState, toState))
                {
                    // Path between (from, to) states not valid, but perhaps to STATE is
                    // Search until next valid STATE is found in existing path
                    std::size_t subsearch_id = toID;
                    base::State *new_to;
                    OMPL_DEBUG("Searching for next valid state, because state %d to %d was not valid out  %d total "
                               "states",
                               fromID, toID, primaryPath.getStateCount());
                    while (subsearch_id < primaryPath.getStateCount())
                    {
                        new_to = primaryPath.getState(subsearch_id);
                        if (si_->isValid(new_to))
                        {
                            OMPL_DEBUG("State %d was found to valid, we can now repair between states", subsearch_id);
                            // This future state is valid, we can stop searching
                            toID = subsearch_id;
                            toState = new_to;
                            break;
                        }
                        ++subsearch_id;  // keep searching for a new state to plan to
                    }
                    // Check if we ever found a next state that is valid
                    if (subsearch_id >= primaryPath.getStateCount())
                    {
                        // We never found a valid state to plan to, instead we reached the goal state and it too wasn't
                        // valid. This is bad.
                        // I think this is a bug.
                        OMPL_ERROR("No state was found valid in the remainder of the path. Invalid goal state. This "
                                   "should not happen.");
                        return false;
                    }

                    // Plan between our two valid states
                    PathGeometric newPathSegment(si_);

                    // Not valid motion, replan
                    OMPL_DEBUG("Planning from %d to %d", fromID, toID);

                    if (!replan(fromState, toState, newPathSegment, ptc))
                    {
                        OMPL_WARN("Unable to repair path between state %d and %d", fromID, toID);
                        return false;
                    }

                    // TODO make sure not approximate solution

                    // Reference to the path
                    std::vector<base::State *> &primaryPathStates = primaryPath.getStates();

                    // Remove all invalid states between (fromID, toID) - not including those states themselves
                    while (fromID != toID - 1)
                    {
                        OMPL_INFORM("Deleting state %d", fromID + 1);
                        primaryPathStates.erase(primaryPathStates.begin() + fromID + 1);
                        --toID;  // because vector has shrunk
                    }

                    // Insert new path segment into current path
                    OMPL_DEBUG("Inserting new %d states into old path. Previous length: %d",
                               newPathSegment.getStateCount() - 2, primaryPathStates.size());

                    // Note: skip first and last states because they should be same as our start and goal state, same as
                    // `fromID` and `toID`
                    for (std::size_t i = 1; i < newPathSegment.getStateCount() - 1; ++i)
                    {
                        std::size_t insertLocation = toID + i - 1;
                        OMPL_DEBUG("Inserting newPathSegment state %d into old path at position %d", i, insertLocation);
                        primaryPathStates.insert(primaryPathStates.begin() + insertLocation,
                                                 si_->cloneState(newPathSegment.getStates()[i]));
                    }
                    // primaryPathStates.insert( primaryPathStates.begin() + toID, newPathSegment.getStates().begin(),
                    // newPathSegment.getStates().end() );
                    OMPL_DEBUG("Inserted new states into old path. New length: %d", primaryPathStates.size());

                    // Set the toID to jump over the newly inserted states to the next unchecked state. Subtract 2
                    // because we ignore start and goal
                    toID = toID + newPathSegment.getStateCount() - 2;
                    OMPL_DEBUG("Continuing searching at state %d", toID);
                }
            }

            OMPL_INFORM("Done repairing ---------------------------------");

            return true;
        }

        bool ThunderRetrieveRepair::replan(const base::State *start, const base::State *goal,
                                           PathGeometric &newPathSegment, const base::PlannerTerminationCondition &ptc)
        {
            // Reset problem definition
            repairProblemDef_->clearSolutionPaths();
            repairProblemDef_->clearStartStates();
            repairProblemDef_->clearGoal();

            // Reset planner
            repairPlanner_->clear();

            // Configure problem definition
            repairProblemDef_->setStartAndGoalStates(start, goal);

            // Configure planner
            repairPlanner_->setProblemDefinition(repairProblemDef_);

            // Solve
            OMPL_INFORM("Preparing to repair path-----------------------------------------");
            base::PlannerStatus lastStatus = base::PlannerStatus::UNKNOWN;
            time::point startTime = time::now();

            // TODO: if we use replanner like RRT* the ptc will allow it to run too long and no time will be left for
            // the rest of algorithm
            lastStatus = repairPlanner_->solve(ptc);

            // Results
            double planTime = time::seconds(time::now() - startTime);
            if (!lastStatus)
            {
                OMPL_WARN("Replan Solve: No replan solution between disconnected states found after %f seconds",
                          planTime);
                return false;
            }

            // Check if approximate
            if (repairProblemDef_->hasApproximateSolution() ||
                repairProblemDef_->getSolutionDifference() > std::numeric_limits<double>::epsilon())
            {
                OMPL_INFORM("Replan Solve: Solution is approximate, not using");
                return false;
            }

            // Convert solution into a PathGeometric path
            base::PathPtr p = repairProblemDef_->getSolutionPath();
            if (!p)
            {
                OMPL_ERROR("Unable to get solution path from problem definition");
                return false;
            }

            newPathSegment = static_cast<PathGeometric &>(*p);

            // Smooth the result
            OMPL_INFORM("Repair: Simplifying solution (smoothing)...");
            time::point simplifyStart = time::now();
            std::size_t numStates = newPathSegment.getStateCount();
            path_simplifier_->simplify(newPathSegment, ptc);
            double simplifyTime = time::seconds(time::now() - simplifyStart);
            OMPL_INFORM("ThunderRetrieveRepair: Path simplification took %f seconds and removed %d states",
                        simplifyTime, numStates - newPathSegment.getStateCount());

            // Save the planner data for debugging purposes
            repairPlannerDatas_.push_back(std::make_shared<base::PlannerData>(si_));
            repairPlanner_->getPlannerData(*repairPlannerDatas_.back());
            repairPlannerDatas_.back()->decoupleFromPlanner();  // copy states so that when planner unloads/clears we
                                                                // don't lose them

            // Return success
            OMPL_INFORM("Replan Solve: solution found in %f seconds with %d states", planTime,
                        newPathSegment.getStateCount());

            return true;
        }

        void ThunderRetrieveRepair::getPlannerData(base::PlannerData &data) const
        {
            OMPL_INFORM("ThunderRetrieveRepair getPlannerData: including %d similar paths", nearestPaths_.size());

            // Visualize the n candidate paths that we recalled from the database
            for (auto path : nearestPaths_)
            {
                for (std::size_t j = 1; j < path.getStateCount(); ++j)
                {
                    data.addEdge(base::PlannerDataVertex(path.getState(j - 1)),
                                 base::PlannerDataVertex(path.getState(j)));
                }
            }
        }

        const std::vector<PathGeometric> &ThunderRetrieveRepair::getLastRecalledNearestPaths() const
        {
            return nearestPaths_;  // list of candidate paths
        }

        std::size_t ThunderRetrieveRepair::getLastRecalledNearestPathChosen() const
        {
            return nearestPathsChosenID_;  // of the candidate paths list, the one we chose
        }

        const PathGeometric &ThunderRetrieveRepair::getChosenRecallPath() const
        {
            return nearestPaths_[nearestPathsChosenID_];
        }

        void ThunderRetrieveRepair::getRepairPlannerDatas(std::vector<base::PlannerDataPtr> &data) const
        {
            data = repairPlannerDatas_;
        }

        std::size_t ThunderRetrieveRepair::checkMotionScore(const base::State *s1, const base::State *s2) const
        {
            int segmentCount = si_->getStateSpace()->validSegmentCount(s1, s2);

            std::size_t invalidStatesScore = 0;  // count number of interpolated states in collision

            // temporary storage for the checked state
            base::State *test = si_->allocState();

            // Linerarly step through motion between state 0 to state 1
            double iteration_step = 1.0 / double(segmentCount);
            for (double location = 0.0; location <= 1.0; location += iteration_step)
            {
                si_->getStateSpace()->interpolate(s1, s2, location, test);

                if (!si_->isValid(test))
                {
                    // OMPL_DEBUG("Found INVALID location between states at gradient %f", location);
                    invalidStatesScore++;
                }
                else
                {
                    // OMPL_DEBUG("Found valid location between states at gradient %f", location);
                }
            }
            si_->freeState(test);

            return invalidStatesScore;
        }

    }  // namespace geometric
}  // namespace ompl
