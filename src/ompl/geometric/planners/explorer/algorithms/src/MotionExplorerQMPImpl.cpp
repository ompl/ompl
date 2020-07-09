#include <ompl/geometric/planners/explorer/algorithms/MotionExplorerQMPImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include "../MotionExplorerQMPImpl.h"
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/geometric/planners/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/geometric/planners/explorer/datastructures/PathVisibilityChecker.h>
#include <ompl/geometric/planners/multilevel/datastructures/PlannerDataVertexAnnotated.h>

#define foreach BOOST_FOREACH

ompl::geometric::MotionExplorerQMPImpl::MotionExplorerQMPImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : BaseT(si, parent_)
{
    setName("MotionExplorerQMPImpl" + std::to_string(id_));
    pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());
}

ompl::geometric::MotionExplorerQMPImpl::~MotionExplorerQMPImpl()
{
    delete pathVisibilityChecker_;
}

void ompl::geometric::MotionExplorerQMPImpl::grow()
{
    if (firstRun_)
    {
        init();
        vGoal_ = addConfiguration(qGoal_);
        firstRun_ = false;

        if (hasBaseSpace())
        {
            if (getPathRestriction()->hasFeasibleSection(qStart_, qGoal_))
            {
                if (sameComponent(vStart_, vGoal_))
                {
                    hasSolution_ = true;
                }
            }
        }
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
        double pathlength = pPath->length() + pPath2->length();

        bool isVisible = false;

        for (uint i = 0; i < getNumberOfPaths(); i++)
        {
            // FALL 2: verbessert bestehenden pfad
            // Knoten wurde mit 2 Knoten des Pfades verbunden
            if (pathVisibilityChecker_->IsPathVisible(path, getCriticalPath(i), graph_))
            {
                isVisible = true;
                // Path is shorter than visible path else do nothing
                if (pathlength < getPathLength(i))
                {
                    updatePath(i, path, pathlength);
                }
                break;
            }
        }

        if (!isVisible)
        {
            addPath(path, pathlength);
        }

        if(pathlength < bestLength_)
        {
            bestLength_ = pathlength;
        }
        ompl::base::PathPtr pp = getPath(vStart_, vGoal_);
        std::cout << "Best cost found: " << bestLength_ << "; length shortest path: " << pp->length() << std::endl;


    }
}

