#include <ompl/geometric/planners/explorer/algorithms/PathSpaceQMP.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include "../PathSpaceQMP.h"
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/geometric/planners/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/geometric/planners/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/planners/explorer/datastructures/PathVisibilityChecker.h>

#define foreach BOOST_FOREACH

ompl::geometric::PathSpaceQMP::PathSpaceQMP(const base::SpaceInformationPtr &si, BundleSpace *parent_): 
  BaseT(si, parent_),
  PathSpace(this)
{
    setName("PathSpaceQMP" + std::to_string(id_));
    if (hasBaseSpace())
    {
        static_cast<BundleSpaceGraph*>(getParent())->getGraphSampler()->disablePathBias();
    }
}

ompl::geometric::PathSpaceQMP::~PathSpaceQMP()
{
}

void ompl::geometric::PathSpaceQMP::grow()
{
    if (firstRun_)
    {
        init();
        vGoal_ = addConfiguration(qGoal_);
        firstRun_ = false;

        // if (hasBaseSpace())
        // {
        //     if (getPathRestriction()->hasFeasibleSection(qStart_, qGoal_))
        //     {
        //         if (sameComponent(vStart_, vGoal_))
        //         {
        //             hasSolution_ = true;
        //         }
        //     }
        // }
    }

    //(1) Get Random Sample
    if (!sampleBundleValid(xRandom_->state))
        return;

    //(2) Add Configuration if valid
    Configuration *xNew = new Configuration(getBundle(), xRandom_->state);
    addConfiguration(xNew);

    //(3) Connect to K nearest neighbors
    connectNeighbors(xNew);

    if (!hasSolution_)
    {
        if (sameComponent(vStart_, vGoal_))
        {
            hasSolution_ = true;
        }
    }

    Vertex v = xNew->index;

    // FALL 1: Pfad ist mit Start und Ziel verbunden
    bool b = sameComponent(v, vStart_) && sameComponent(v, vGoal_);

    if (b)
    {
        ompl::base::PathPtr pPath = getPath(vStart_, v);
        std::vector<Vertex> path = shortestVertexPath_;

        ompl::base::PathPtr pPath2 = getPath(v, vGoal_);
        std::vector<Vertex> path2 = shortestVertexPath_;

        path.insert(path.end(), path2.begin() + 1, path2.end());
        double pathcost = pPath->length() + pPath2->length();

        bool isVisible = false;

        for (uint i = 0; i < getNumberOfPaths(); i++)
        {
            // FALL 2: verbessert bestehenden pfad
            // Knoten wurde mit 2 Knoten des Pfades verbunden
            if (getPathVisibilityChecker()->IsPathVisible(path, getCriticalPath(i), graph_))
            {
                isVisible = true;
                // Path is shorter than visible path else do nothing
                if (pathcost < getPathCost(i))
                {
                    updatePath(i, path, pathcost);
                }
                break;
            }
        }

        if (!isVisible)
        {
            addPath(path, pathcost);
        }

        if(pathcost < bestCost_)
        {
            bestCost_ = pathcost;
        }
        // ompl::base::PathPtr pp = getPath(vStart_, vGoal_);
        // std::cout << "Best cost found: " << bestCost_ << "; cost shortest path: " << pp->length() << std::endl;


    }
}
