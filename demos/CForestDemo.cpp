/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Javier V. Gomez - adapted from Point2DPlanning.cpp */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/util/PPM.h>
#include <../tests/resources/config.h>

#include <boost/filesystem.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::time;

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

template <class planner_t>
class Plane2DEnvironment
{
public:

    Plane2DEnvironment<planner_t>(const char *ppm_file)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            space->setup(); 

            // Construct a space information instance for this state space
            ob::StateSpacePtr sspace (space);
            si_.reset(new ob::SpaceInformation(sspace));

            // Set the object used to check which states in the space are valid
            si_->setStateValidityChecker( boost::bind(&Plane2DEnvironment::isStateValid, this, _1) );
            si_->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            si_->setup();
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        if (!si_)
            return false;
        ob::ScopedState<> start(si_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(si_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        
        // Create a problem instance
        pdef_.reset(new ob::ProblemDefinition(si_));

        // Set the start and goal states
        pdef_->setStartAndGoalStates(start, goal);

        // Set the optimization objective (select one of them)
        pdef_->setOptimizationObjective(getPathLengthObjective(si_));
        pdef_->getOptimizationObjective()->setCostThreshold(ob::Cost(2070));

        // Construct our optimal planner using the RRTstar algorithm.
        planner_.reset(new planner_t(si_));
        // Set the problem instance for our planner to solve
        planner_->setProblemDefinition(pdef_);

        // testing some specific configurations
        //planner_->as<og::RRTstar>()->setPrune(true);
        //planner_->as<og::CForest>()->setPrune(false);
        //planner_->as<og::CForest>()->setPlannerInstances<og::RRTstar>(3);

        planner_->setup();

        // attempt to solve the planning problem within one second of
        // planning time
        ot::point start_t = ot::now();
        bool v = planner_->solve(5.0);
        double duration = ot::seconds(ot::now() - start_t);
        OMPL_DEBUG("Solution found by %s in %lf seconds", planner_->getName().c_str(), duration);
        return v;
    }

    void recordSolution()
    {
        if (!pdef_->getSolutionPath().get())
            return;

        const ob::PathPtr &pPtr = pdef_->getSolutionPath();
        og::PathGeometric &p = static_cast<og::PathGeometric&>(*pPtr);
        //p.printAsMatrix(std::cout);

        p.interpolate();
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    double getLowestCost() const 
    {
        if (!pdef_->getSolutionPath().get())
            return -1.0;
        return pdef_->getSolutionPath()->cost(getPathLengthObjective(si_)).v;
    }

    void save(const char *filename)
    {
        ppm_.saveFile(filename);
    }
    
    void saveTree(const char *filename, const size_t idx)
    {
        planner_->as<og::CForest>()->getPlannerInstance(idx)->as<og::RRTstar>()->saveTree(filename);
    }

    void saveTree(const char *filename)
    {
        planner_->as<og::RRTstar>()->saveTree(filename);
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    ob::SpaceInformationPtr si_;
    ob::ProblemDefinitionPtr pdef_;
    ob::PlannerPtr planner_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;

};


int main(int argc, char** argv)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    Plane2DEnvironment<og::CForest> env_cf((path / "ppm/floor.ppm").string().c_str());

   if (env_cf.plan(0, 0, 1140, 1402))
    {

        OMPL_INFORM("**** Plan successful ****");
        env_cf.recordSolution();
        env_cf.save("result_demo_cforest.ppm");
        //env.savePlannerData("plannerData");
        env_cf.saveTree("tree_0.txt", 0);
        env_cf.saveTree("tree_1.txt", 1);
        std::cout << "Final lowest cost CForest: " << env_cf.getLowestCost() << std::endl;
    }

    Plane2DEnvironment<og::RRTstar> env_rrts((path / "ppm/floor.ppm").string().c_str());

    if (env_rrts.plan(0, 0, 1140, 1402))
    {
        OMPL_INFORM("**** Plan successful ****");
        env_rrts.recordSolution();
        env_rrts.save("result_demo_rrtstar.ppm");
        env_rrts.saveTree("tree_rrt.txt");
        std::cout << "Final lowest cost RRTstar: " << env_rrts.getLowestCost() << std::endl;
    }

    return 0;
}


ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}
