/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK, The University of Tokyo.
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
 *   * Neither the name of the JSK, The University of Tokyo nor the names of its
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

#include <ompl/tools/thunder/Thunder.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/util/Console.h>

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;

ompl::tools::Thunder::Thunder(const base::SpaceInformationPtr &si) : ompl::tools::ExperienceSetup(si)
{
    initialize();
}

ompl::tools::Thunder::Thunder(const base::StateSpacePtr &space) : ompl::tools::ExperienceSetup(space)
{
    initialize();
}

void ompl::tools::Thunder::initialize()
{
    OMPL_INFORM("Initializing Thunder Framework");

    filePath_ = "unloaded";

    // Load the experience database
    experienceDB_ = std::make_shared<ompl::tools::ThunderDB>(si_->getStateSpace());

    // Load the Retrieve repair database. We do it here so that setRepairPlanner() works
    rrPlanner_ = std::make_shared<og::ThunderRetrieveRepair>(si_, experienceDB_);

    OMPL_INFORM("Thunder Framework initialized.");
}

void ompl::tools::Thunder::setup()
{
    if (!configured_ || !si_->isSetup() || !planner_->isSetup() || !rrPlanner_->isSetup())
    {
        SimpleSetup::setup();

        // Decide if we should setup the second planning from scratch planner for benchmarking w/o recall
        if (dualThreadScratchEnabled_ && !recallEnabled_)
        {
            // Setup planning from scratch planner 2
            if (!planner2_)
            {
                if (pa_)
                    planner2_ = pa_(si_);
                if (!planner2_)
                {
                    OMPL_INFORM("Getting default planner: ");
                    planner2_ = std::make_shared<ompl::geometric::RRTConnect>(si_);
                    // This was disabled because i like to use Thunder / SPARSdb without setting a goal definition
                    // planner2_ = ompl::geometric::getDefaultPlanner(pdef_->getGoal()); // we could use the
                    // repairProblemDef_ here but that isn't setup yet

                    OMPL_INFORM("No planner 2 specified. Using default: %s", planner2_->getName().c_str());
                }
            }
            planner2_->setProblemDefinition(pdef_);
            if (!planner2_->isSetup())
                planner2_->setup();
        }

        // Setup planning from experience planner
        rrPlanner_->setProblemDefinition(pdef_);

        if (!rrPlanner_->isSetup())
            rrPlanner_->setup();

        // Create the parallel component for splitting into two threads
        pp_ = std::make_shared<ot::ParallelPlan>(pdef_);
        if (!scratchEnabled_ && !recallEnabled_)
        {
            throw Exception("Both planning from scratch and experience have been disabled, unable to plan");
        }
        if (recallEnabled_)
            pp_->addPlanner(rrPlanner_);  // Add the planning from experience planner if desired
        if (scratchEnabled_)
            pp_->addPlanner(planner_);  // Add the planning from scratch planner if desired
        if (dualThreadScratchEnabled_ && !recallEnabled_)
        {
            OMPL_INFORM("Adding second planning from scratch planner");
            pp_->addPlanner(planner2_);  // Add a SECOND planning from scratch planner if desired
        }

        // Setup SPARS
        if (!experienceDB_->getSPARSdb())
        {
            OMPL_INFORM("Calling setup() for SPARSdb");

            // Load SPARSdb
            experienceDB_->getSPARSdb() = std::make_shared<ompl::geometric::SPARSdb>(si_);
            experienceDB_->getSPARSdb()->setProblemDefinition(pdef_);
            experienceDB_->getSPARSdb()->setup();

            experienceDB_->getSPARSdb()->setStretchFactor(1.2);
            experienceDB_->getSPARSdb()->setSparseDeltaFraction(
                0.05);  // vertex visibility range  = maximum_extent * this_fraction
            // experienceDB_->getSPARSdb()->setDenseDeltaFraction(0.001);

            experienceDB_->getSPARSdb()->printDebug();

            experienceDB_->load(filePath_);  // load from file
        }
    }
}

void ompl::tools::Thunder::clear()
{
    if (planner_)
        planner_->clear();
    if (rrPlanner_)
        rrPlanner_->clear();
    if (planner2_)
        planner2_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
    if (pp_)
    {
        pp_->clearHybridizationPaths();
    }
}

void ompl::tools::Thunder::setPlannerAllocator(const base::PlannerAllocator &pa)
{
    pa_ = pa;
    planner_.reset();
    // note: the rrPlanner_ never uses the allocator so does not need to be reset
    configured_ = false;
}

ompl::base::PlannerStatus ompl::tools::Thunder::solve(const base::PlannerTerminationCondition &ptc)
{
    // we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner
    // termination condition

    OMPL_INFORM("Thunder Framework: Starting solve()");

    // Setup again in case it has not been done yet
    setup();

    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();

    // Warn if there are queued paths that have not been added to the experience database
    if (!queuedSolutionPaths_.empty())
    {
        OMPL_WARN("Previous solved paths are currently uninserted into the experience database and are in the "
                  "post-proccessing queue");
    }

    // There are two modes for running parallel plan - one in which both threads are run until they both return a result
    // and/or fail
    // The second mode stops with the first solution found - we want this one
    bool stopWhenFirstSolutionFound = true;

    if (stopWhenFirstSolutionFound)
    {
        // If \e hybridize is false, when the first solution is found, the rest of the planners are stopped as well.
        // OMPL_DEBUG("Thunder: stopping when first solution is found from either thread");
        // Start both threads
        bool hybridize = false;
        lastStatus_ = pp_->solve(ptc, hybridize);
    }
    else
    {
        OMPL_WARN("Thunder: not stopping until a solution or a failure is found from both threads. THIS MODE IS JUST "
                  "FOR TESTING");
        // This mode is more for benchmarking, since I don't care about optimality
        // If \e hybridize is false, when \e minSolCount new solutions are found (added to the set of solutions
        // maintained by ompl::base::Goal), the rest of the planners are stopped as well.

        // Start both threads
        std::size_t minSolCount = 2;
        std::size_t maxSolCount = 2;
        bool hybridize = false;
        lastStatus_ = pp_->solve(ptc, minSolCount, maxSolCount, hybridize);
    }

    // Planning time
    planTime_ = time::seconds(time::now() - start);

    // Create log
    ExperienceLog log;
    log.planning_time = planTime_;

    // Record stats
    stats_.totalPlanningTime_ += planTime_;  // used for averaging
    stats_.numProblems_++;                   // used for averaging

    if (lastStatus_ == ompl::base::PlannerStatus::TIMEOUT)
    {
        // Skip further processing if absolutely no path is available
        OMPL_ERROR("Thunder Solve: No solution found after %f seconds", planTime_);

        stats_.numSolutionsTimedout_++;

        // Logging
        log.planner = "neither_planner";
        log.result = "timedout";
        log.is_saved = "not_saved";
    }
    else if (!lastStatus_)
    {
        // Skip further processing if absolutely no path is available
        OMPL_ERROR("Thunder Solve: Unknown failure");
        stats_.numSolutionsFailed_++;

        // Logging
        log.planner = "neither_planner";
        log.result = "failed";
        log.is_saved = "not_saved";
    }
    else
    {
        OMPL_INFORM("Thunder Solve: Possible solution found in %f seconds", planTime_);

        // Smooth the result
        simplifySolution(ptc);

        og::PathGeometric solutionPath = getSolutionPath();  // copied so that it is non-const
        OMPL_INFORM("Solution path has %d states and was generated from planner %s", solutionPath.getStateCount(),
                    getSolutionPlannerName().c_str());

        // Logging
        log.planner = getSolutionPlannerName();

        // Do not save if approximate
        if (!haveExactSolutionPath())
        {
            OMPL_INFORM("THUNDER RESULTS: Approximate");

            // Logging
            log.result = "not_exact_solution";
            log.is_saved = "not_saved";
            log.approximate = true;

            // Stats
            stats_.numSolutionsApproximate_++;

            // TODO not sure what to do here, use case not tested
            OMPL_WARN("NOT saving to database because the solution is APPROXIMATE");
        }
        else if (getSolutionPlannerName() == rrPlanner_->getName())
        {
            OMPL_INFORM("THUNDER RESULTS: From Recall");

            // Stats
            stats_.numSolutionsFromRecall_++;

            // Logging
            log.result = "from_recall";

            // Make sure solution has at least 2 states
            if (solutionPath.getStateCount() < 2)
            {
                OMPL_INFORM("NOT saving to database because solution is less than 2 states long");
                stats_.numSolutionsTooShort_++;

                // Logging
                log.is_saved = "less_2_states";
                log.too_short = true;
            }
            else if (false)  // always add when from recall
            {
                OMPL_INFORM("Adding path to database because SPARS will decide for us if we should keep the nodes");

                // Stats
                stats_.numSolutionsFromRecallSaved_++;

                // Queue the solution path for future insertion into experience database (post-processing)
                queuedSolutionPaths_.push_back(solutionPath);

                // Logging
                log.insertion_failed = false;  // TODO this is wrong logging data
                log.is_saved = "always_attempt";
            }
            else  // never add when from recall
            {
                OMPL_INFORM("NOT adding path to database because SPARS already has it");

                // Logging
                log.is_saved = "skipped";
            }
        }
        else
        {
            OMPL_INFORM("THUNDER RESULTS: From Scratch");

            // Logging
            log.result = "from_scratch";

            // Stats
            stats_.numSolutionsFromScratch_++;

            // Make sure solution has at least 2 states
            if (solutionPath.getStateCount() < 2)
            {
                OMPL_INFORM("NOT saving to database because solution is less than 2 states long");

                // Logging
                log.is_saved = "less_2_states";
                log.too_short = true;

                // Stats
                stats_.numSolutionsTooShort_++;
            }
            else
            {
                OMPL_INFORM("Adding path to database because best solution was not from database");

                // Logging
                log.result = "from_scratch";
                log.is_saved = "saving";

                // Queue the solution path for future insertion into experience database (post-processing)
                queuedSolutionPaths_.push_back(solutionPath);

                log.insertion_failed = false;  // TODO fix this wrong logging info
            }
        }
    }

    // Final log data
    // log.insertion_time = insertionTime; TODO fix this
    log.num_vertices = experienceDB_->getSPARSdb()->getNumVertices();
    log.num_edges = experienceDB_->getSPARSdb()->getNumEdges();
    log.num_connected_components = experienceDB_->getSPARSdb()->getNumConnectedComponents();

    // Flush the log to buffer
    convertLogToString(log);

    return lastStatus_;
}

ompl::base::PlannerStatus ompl::tools::Thunder::solve(double time)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(time);
    return solve(ptc);
}

bool ompl::tools::Thunder::save()
{
    setup();  // ensure the PRM db has been loaded to the Experience DB
    return experienceDB_->save(filePath_);
}

bool ompl::tools::Thunder::saveIfChanged()
{
    setup();  // ensure the PRM db has been loaded to the Experience DB
    return experienceDB_->saveIfChanged(filePath_);
}

void ompl::tools::Thunder::printResultsInfo(std::ostream &out) const
{
    for (std::size_t i = 0; i < pdef_->getSolutionCount(); ++i)
    {
        out << "Solution " << i << "\t | Length: " << pdef_->getSolutions()[i].length_
            << "\t | Approximate: " << (pdef_->getSolutions()[i].approximate_ ? "true" : "false")
            << "\t | Planner: " << pdef_->getSolutions()[i].plannerName_ << std::endl;
    }
}

void ompl::tools::Thunder::print(std::ostream &out) const
{
    if (si_)
    {
        si_->printProperties(out);
        si_->printSettings(out);
    }
    if (planner_)
    {
        planner_->printProperties(out);
        planner_->printSettings(out);
    }
    if (rrPlanner_)
    {
        rrPlanner_->printProperties(out);
        rrPlanner_->printSettings(out);
    }
    if (planner2_)
    {
        planner2_->printProperties(out);
        planner2_->printSettings(out);
    }
    if (pdef_)
        pdef_->print(out);
}

void ompl::tools::Thunder::printLogs(std::ostream &out) const
{
    if (!recallEnabled_)
        out << "Scratch Planning Logging Results (inside Thunder Framework)" << std::endl;
    else
        out << "Thunder Framework Logging Results" << std::endl;
    out << "  Solutions Attempted:           " << stats_.numProblems_ << std::endl;
    out << "    Solved from scratch:        " << stats_.numSolutionsFromScratch_ << " ("
        << stats_.numSolutionsFromScratch_ / stats_.numProblems_ * 100 << "%)" << std::endl;
    out << "    Solved from recall:         " << stats_.numSolutionsFromRecall_ << " ("
        << stats_.numSolutionsFromRecall_ / stats_.numProblems_ * 100 << "%)" << std::endl;
    out << "      That were saved:         " << stats_.numSolutionsFromRecallSaved_ << std::endl;
    out << "      That were discarded:     " << stats_.numSolutionsFromRecall_ - stats_.numSolutionsFromRecallSaved_
        << std::endl;
    out << "      Less than 2 states:      " << stats_.numSolutionsTooShort_ << std::endl;
    out << "    Failed:                     " << stats_.numSolutionsFailed_ << std::endl;
    out << "    Timedout:                    " << stats_.numSolutionsTimedout_ << std::endl;
    out << "    Approximate:                 " << stats_.numSolutionsApproximate_ << std::endl;
    out << "  SPARSdb                        " << std::endl;
    out << "    Vertices:                    " << experienceDB_->getSPARSdb()->getNumVertices() << std::endl;
    out << "    Edges:                       " << experienceDB_->getSPARSdb()->getNumEdges() << std::endl;
    out << "    Connected Components:        " << experienceDB_->getSPARSdb()->getNumConnectedComponents() << std::endl;
    out << "    Unsaved paths inserted:      " << experienceDB_->getNumPathsInserted() << std::endl;
    out << "    Consecutive state failures:  " << experienceDB_->getSPARSdb()->getNumConsecutiveFailures() << std::endl;
    out << "    Connected path failures:     " << experienceDB_->getSPARSdb()->getNumPathInsertionFailed() << std::endl;
    out << "    Sparse Delta Fraction:       " << experienceDB_->getSPARSdb()->getSparseDeltaFraction() << std::endl;
    out << "  Average planning time:         " << stats_.getAveragePlanningTime() << std::endl;
    out << "  Average insertion time:        " << stats_.getAverageInsertionTime() << std::endl;
}

std::size_t ompl::tools::Thunder::getExperiencesCount() const
{
    return experienceDB_->getSPARSdb()->getNumVertices();
}

void ompl::tools::Thunder::getAllPlannerDatas(std::vector<ob::PlannerDataPtr> &plannerDatas) const
{
    experienceDB_->getAllPlannerDatas(plannerDatas);
}

void ompl::tools::Thunder::convertPlannerData(const ob::PlannerDataPtr &plannerData, og::PathGeometric &path)
{
    // Convert the planner data vertices into a vector of states
    for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
        path.append(plannerData->getVertex(i).getState());
}

bool ompl::tools::Thunder::reversePathIfNecessary(og::PathGeometric &path1, og::PathGeometric &path2)
{
    // Reverse path2 if it matches better
    const ob::State *s1 = path1.getState(0);
    const ob::State *s2 = path2.getState(0);
    const ob::State *g1 = path1.getState(path1.getStateCount() - 1);
    const ob::State *g2 = path2.getState(path2.getStateCount() - 1);

    double regularDistance = si_->distance(s1, s2) + si_->distance(g1, g2);
    double reversedDistance = si_->distance(s1, g2) + si_->distance(s2, g1);

    // Check if path is reversed from normal [start->goal] direction
    if (regularDistance > reversedDistance)
    {
        // needs to be reversed
        path2.reverse();
        return true;
    }

    return false;
}

ompl::tools::ThunderDBPtr ompl::tools::Thunder::getExperienceDB()
{
    return experienceDB_;
}

bool ompl::tools::Thunder::doPostProcessing()
{
    OMPL_INFORM("Performing post-processing");

    for (auto &queuedSolutionPath : queuedSolutionPaths_)
    {
        // Time to add a path to experience database
        double insertionTime;

        experienceDB_->addPath(queuedSolutionPath, insertionTime);
        OMPL_INFORM("Finished inserting experience path in %f seconds", insertionTime);
        stats_.totalInsertionTime_ += insertionTime;  // used for averaging
    }

    // Remove all inserted paths from the queue
    queuedSolutionPaths_.clear();

    return true;
}
