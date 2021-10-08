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

#include "ompl/tools/lightning/Lightning.h"
#include "ompl/tools/lightning/LightningDB.h"

namespace og = ompl::geometric;
namespace ob = ompl::base;
namespace ot = ompl::tools;

ompl::tools::Lightning::Lightning(const base::SpaceInformationPtr &si) : ompl::tools::ExperienceSetup(si)
{
    initialize();
}

ompl::tools::Lightning::Lightning(const base::StateSpacePtr &space) : ompl::tools::ExperienceSetup(space)
{
    initialize();
}

void ompl::tools::Lightning::initialize()
{
    // Load dynamic time warp
    dtw_ = std::make_shared<ot::DynamicTimeWarp>(si_);

    // Load the experience database
    experienceDB_ = std::make_shared<ompl::tools::LightningDB>(si_->getStateSpace());

    // Load the Retrieve repair database. We do it here so that setRepairPlanner() works
    rrPlanner_ = std::make_shared<og::LightningRetrieveRepair>(si_, experienceDB_);

    OMPL_INFORM("Lightning Framework initialized.");
}

void ompl::tools::Lightning::setup()
{
    if (!configured_ || !si_->isSetup() || !planner_->isSetup() || !rrPlanner_->isSetup())
    {
        SimpleSetup::setup();

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
        if (scratchEnabled_)
            pp_->addPlanner(planner_);  // Add the planning from scratch planner if desired
        if (recallEnabled_)
            pp_->addPlanner(rrPlanner_);  // Add the planning from experience planner if desired

        // Check if experience database is already loaded
        if (experienceDB_->isEmpty())
        {
            if (filePath_.empty())
            {
                OMPL_ERROR("No file path has been specified, unable to load experience DB");
            }
            else
            {
                experienceDB_->load(filePath_);  // load from file
            }
        }
        else
            OMPL_ERROR("Attempting to load experience database when it is not empty");
    }
}

void ompl::tools::Lightning::clear()
{
    if (planner_)
        planner_->clear();
    if (rrPlanner_)
        rrPlanner_->clear();
    if (pdef_)
        pdef_->clearSolutionPaths();
    if (pp_)
    {
        pp_->clearHybridizationPaths();
    }
}

// we provide a duplicate implementation here to allow the planner to choose how the time is turned into a planner
// termination condition
ompl::base::PlannerStatus ompl::tools::Lightning::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("Lightning Framework: Starting solve()");

    // Setup again in case it has not been done yet
    setup();

    lastStatus_ = base::PlannerStatus::UNKNOWN;
    time::point start = time::now();

    // Insertion time
    double insertionTime = 0.;

    // Start both threads
    bool hybridize = false;
    lastStatus_ = pp_->solve(ptc, hybridize);

    // Planning time
    planTime_ = time::seconds(time::now() - start);
    stats_.totalPlanningTime_ += planTime_;  // used for averaging
    stats_.numProblems_++;                   // used for averaging

    // Create log
    ExperienceLog log;
    log.planning_time = planTime_;

    if (lastStatus_ == ompl::base::PlannerStatus::TIMEOUT)
    {
        // Skip further processing if absolutely no path is available
        OMPL_ERROR("Lightning Solve: No solution found after %f seconds", planTime_);
        stats_.numSolutionsTimedout_++;

        // Logging
        log.planner = "neither_planner";
        log.result = "timedout";
        log.is_saved = "not_saved";
    }
    else if (!lastStatus_)
    {
        // Skip further processing if absolutely no path is available
        OMPL_ERROR("Lightning Solve: Unknown failure, planner status: %s", lastStatus_.asString().c_str());
        stats_.numSolutionsFailed_++;

        // Logging
        log.planner = "neither_planner";
        log.result = "failed";
        log.is_saved = "not_saved";
    }
    else
    {
        OMPL_INFORM("Lightning Solve: Possible solution found in %f seconds", planTime_);

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
            // Logging
            log.result = "not_exact_solution";
            log.is_saved = "not_saved";
            log.approximate = true;

            // Stats
            stats_.numSolutionsApproximate_++;

            // not sure what to do here, use case not tested
            OMPL_INFORM("NOT saving to database because the solution is APPROXIMATE");
        }
        // Use dynamic time warping to see if the repaired path is too similar to the original
        else if (getSolutionPlannerName() == rrPlanner_->getName())
        {
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
            else
            {
                // Benchmark runtime
                time::point startTime = time::now();

                // Convert the original recalled path to PathGeometric
                ob::PlannerDataPtr chosenRecallPathData = getLightningRetrieveRepairPlanner().getChosenRecallPath();
                og::PathGeometric chosenRecallPath(si_);
                convertPlannerData(chosenRecallPathData, chosenRecallPath);

                // Reverse path2 if necessary so that it matches path1 better
                reversePathIfNecessary(solutionPath, chosenRecallPath);

                double score = dtw_->getPathsScore(solutionPath, chosenRecallPath);
                log.score = score;

                if (score < 4)
                {
                    OMPL_INFORM("NOT saving to database because best solution was from database and is too similar "
                                "(score %f)",
                                score);

                    // Logging
                    log.insertion_failed = true;
                    log.is_saved = "score_too_similar";
                }
                else
                {
                    OMPL_INFORM("Adding path to database because repaired path is different enough from original "
                                "recalled path (score %f)",
                                score);

                    // Logging
                    log.insertion_failed = false;
                    log.is_saved = "score_different_enough";

                    // Stats
                    stats_.numSolutionsFromRecallSaved_++;

                    // Save to database
                    double dummyInsertionTime;  // unused because does not include scoring function
                    experienceDB_->addPath(solutionPath, dummyInsertionTime);
                }
                insertionTime += time::seconds(time::now() - startTime);
            }
        }
        else
        {
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

                // Save to database
                experienceDB_->addPath(solutionPath, insertionTime);
            }
        }
    }

    stats_.totalInsertionTime_ += insertionTime;  // used for averaging

    // Final log data
    log.insertion_time = insertionTime;
    log.num_vertices = experienceDB_->getStatesCount();
    log.num_edges = 0;
    log.num_connected_components = 0;

    // Flush the log to buffer
    convertLogToString(log);

    return lastStatus_;
}

ompl::base::PlannerStatus ompl::tools::Lightning::solve(double time)
{
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(time);
    return solve(ptc);
}

bool ompl::tools::Lightning::save()
{
    if (filePath_.empty())
    {
        OMPL_ERROR("No file path has been specified, unable to save experience DB");
        return false;
    }
    return experienceDB_->save(filePath_);
}

bool ompl::tools::Lightning::saveIfChanged()
{
    if (filePath_.empty())
    {
        OMPL_ERROR("No file path has been specified, unable to save experience DB");
        return false;
    }
    return experienceDB_->saveIfChanged(filePath_);
}

void ompl::tools::Lightning::printResultsInfo(std::ostream &out) const
{
    for (std::size_t i = 0; i < pdef_->getSolutionCount(); ++i)
    {
        out << "Solution " << i << " | Length: " << pdef_->getSolutions()[i].length_
            << " | Approximate: " << (pdef_->getSolutions()[i].approximate_ ? "true" : "false")
            << " | Planner: " << pdef_->getSolutions()[i].plannerName_ << std::endl;
    }
}

void ompl::tools::Lightning::print(std::ostream &out) const
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
    if (pdef_)
        pdef_->print(out);
}

void ompl::tools::Lightning::printLogs(std::ostream &out) const
{
    out << "Lightning Framework Logging Results" << std::endl;
    out << "  Solutions Attempted:           " << stats_.numProblems_ << std::endl;
    out << "     Solved from scratch:        " << stats_.numSolutionsFromScratch_ << " ("
        << stats_.numSolutionsFromScratch_ / stats_.numProblems_ * 100.0 << "%)" << std::endl;
    out << "     Solved from recall:         " << stats_.numSolutionsFromRecall_ << " ("
        << stats_.numSolutionsFromRecall_ / stats_.numProblems_ * 100.0 << "%)" << std::endl;
    out << "        That were saved:         " << stats_.numSolutionsFromRecallSaved_ << std::endl;
    out << "        That were discarded:     " << stats_.numSolutionsFromRecall_ - stats_.numSolutionsFromRecallSaved_
        << std::endl;
    out << "        Less than 2 states:      " << stats_.numSolutionsTooShort_ << std::endl;
    out << "     Failed:                     " << stats_.numSolutionsFailed_ << std::endl;
    out << "     Timedout:                   " << stats_.numSolutionsTimedout_ << std::endl;
    out << "     Approximate:                " << stats_.numSolutionsApproximate_ << std::endl;
    out << "  LightningDb                    " << std::endl;
    out << "     Total paths:                " << experienceDB_->getExperiencesCount() << std::endl;
    out << "     Vertices (states):          " << experienceDB_->getStatesCount() << std::endl;
    out << "     Unsaved solutions:          " << experienceDB_->getNumUnsavedPaths() << std::endl;
    out << "  Average planning time:         " << stats_.getAveragePlanningTime() << std::endl;
    out << "  Average insertion time:        " << stats_.getAverageInsertionTime() << std::endl;
}

std::size_t ompl::tools::Lightning::getExperiencesCount() const
{
    return experienceDB_->getExperiencesCount();
}

void ompl::tools::Lightning::getAllPlannerDatas(std::vector<ob::PlannerDataPtr> &plannerDatas) const
{
    experienceDB_->getAllPlannerDatas(plannerDatas);
}

void ompl::tools::Lightning::convertPlannerData(const ob::PlannerDataPtr &plannerData, og::PathGeometric &path)
{
    // Convert the planner data vertices into a vector of states
    for (std::size_t i = 0; i < plannerData->numVertices(); ++i)
        path.append(plannerData->getVertex(i).getState());
}

bool ompl::tools::Lightning::reversePathIfNecessary(og::PathGeometric &path1, og::PathGeometric &path2)
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
