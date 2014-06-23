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

/* Author: Javier V. Gomez - adapted from Point2DPlanning.cpp */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/util/PPM.h>
#include <../tests/resources/config.h>

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>

#include <boost/graph/astar_search.hpp>
#include <boost/filesystem.hpp>


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::time;

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState* goal,
                           const ob::OptimizationObjective* obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                           vertex_type_t>::type& plannerDataVertices);

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
		//planner_.reset(new og::RRTstar(si_));
		// Set the problem instance for our planner to solve
		planner_->setProblemDefinition(pdef_);
		
		//planner_->as<og::CForest>()->setPlannerInstances<og::RRTStar>(2);
		
		planner_->setup();

		// attempt to solve the planning problem within one second of
		// planning time
		ot::point start_t = ot::now();
		bool v = planner_->solve(5.0);
		double duration = ot::seconds(ot::now() - start_t);
        OMPL_DEBUG("Solution found by in %lf seconds", duration);
		return v;
    }

    void recordSolution()
    {
        if (!pdef_->getSolutionPath().get())
            return;
     
        const ob::PathPtr &pPtr = pdef_->getSolutionPath();
        og::PathGeometric &p = static_cast<og::PathGeometric&>(*pPtr);
       
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
    
  /*  void saveTree(const char *filename, const size_t idx)
    {
		planner_->as<og::CForest>()->getPlanner(idx)->as<og::RRTstar>()->saveTree(filename);		
	}
	*/
	void savePlannerData(const char *filename)
	{
		ob::PlannerData data(si_);
        planner_->getPlannerData(data);

        ob::PlannerDataStorage dataStorage;
        dataStorage.store(data, filename);
	}
	
	void loadPlannerData(const char *filename)
	{
		std::cout << "Reading PlannerData from "<< filename << std::endl;

		ob::PlannerDataStorage dataStorage;
		ob::PlannerData data(si_);

		// Loading an instance of PlannerData from disk.
		dataStorage.load("filename", data);
		
		// Re-extract the shortest path from the loaded planner data
		if (data.numStartVertices() > 0 && data.numGoalVertices() > 0)
		{
			// Create an optimization objective for optimizing path length in A*
			ob::PathLengthOptimizationObjective opt(si_);

			// Computing the weights of all edges based on the state space distance
			// This is not done by default for efficiency
			data.computeEdgeWeights(opt);

			// Getting a handle to the raw Boost.Graph data
			ob::PlannerData::Graph::Type& graph = data.toBoostGraph();

			// Now we can apply any Boost.Graph algorithm.  How about A*!

			// create a predecessor map to store A* results in
			boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());

			// Retieve a property map with the PlannerDataVertex object pointers for quick lookup
			boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

			// Run A* search over our planner data
			ob::GoalState goal(si_);
			goal.setState(data.getGoalVertex(0).getState());
			ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
			boost::astar_search(graph, start,
								boost::bind(&distanceHeuristic, _1, &goal, &opt, vertices),
								boost::predecessor_map(prev).
								distance_compare(boost::bind(&ob::OptimizationObjective::
															 isCostBetterThan, &opt, _1, _2)).
								distance_combine(boost::bind(&ob::OptimizationObjective::
															 combineCosts, &opt, _1, _2)).
								distance_inf(opt.infiniteCost()).
								distance_zero(opt.identityCost()));

			// Extracting the path
			og::PathGeometric path(si_);
			for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
				 prev[pos] != pos;
				 pos = prev[pos])
			{
				path.append(vertices[pos]->getState());
			}
			path.append(vertices[start]->getState());
			path.reverse();

			// print the path to screen
			//path.print(std::cout);
			std::cout << "Found stored solution with " << path.getStateCount() << " states and length " << path.length() << std::endl;
		}
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
	Plane2DEnvironment<og::CForest> env((path / "ppm/floor.ppm").string().c_str());
	
    if (env.plan(0, 0, 1140, 1402))
    {
		
		OMPL_INFORM("Plan successful");
        //env.recordSolution();
        //env.save("result_demo.ppm");
        //env.savePlannerData("plannerData");
        //env.saveTree("tree_0.txt", 0);
        //env.saveTree("tree_1.txt", 1);
        std::cout << "Final lowest cost: " << env.getLowestCost() << std::endl;
        //env.loadPlannerData("plannerData");
    }

/*	Plane2DEnvironment<og::RRTstar> env2((path / "ppm/floor.ppm").string().c_str());
	
    if (env2.plan(0, 0, 1140, 1402))
    {
		OMPL_INFORM("Plan successful");
		std::cout << "Final lowest cost 2: " << env2.getLowestCost() << std::endl;
	}*/

	return 0;
}


ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState* goal,
                           const ob::OptimizationObjective* obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                           vertex_type_t>::type& plannerDataVertices)
{
    return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}
