#include <ompl/multilevel/planners/explorer/algorithms/PathSpaceSparse.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathVisibilityChecker.h>

#define foreach BOOST_FOREACH

using namespace ompl::multilevel;

PathSpaceSparse::PathSpaceSparse(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : PathSpace(this), BaseT(si, parent_)
{
    pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());

    setName("PathSpaceSparse" + std::to_string(id_));

    // if (hasBaseSpace())
    // {
    //     static_cast<BundleSpaceGraph *>(getBaseBundleSpace())->getGraphSampler()->disablePathBias();
    // }
}

PathSpaceSparse::~PathSpaceSparse()
{
    delete pathVisibilityChecker_;
}

void PathSpaceSparse::grow()
{
    if (firstRun_)
    {
        init();
        // vGoal_ = addConfiguration(qGoal_);
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

    addConfigurationConditional(xNew);

    Vertex v = xNew->index;
    Vertex vStart = getStartIndex();
    Vertex vGoal = getGoalIndex();

    if (!hasSolution_)
    {
        if (sameComponent(vStart, vGoal))
        {
            hasSolution_ = true;
        }
    }

    // FALL 1: Pfad ist mit Start und Ziel verbunden
    bool b = sameComponent(vStart, v) && sameComponent(v, vGoal);

    if (b)
    {
        ompl::base::PathPtr pPath = getPath(vStart, v);
        std::vector<Vertex> path = shortestVertexPath_;

        ompl::base::PathPtr pPath2 = getPath(v, vGoal);
        std::vector<Vertex> path2 = shortestVertexPath_;

        path.insert(path.end(), path2.begin() + 1, path2.end());
        double pathcost = pPath->length() + pPath2->length();

        bool isVisible = false;

        for (uint i = 0; i < getNumberOfPaths(); i++)
        {
            // FALL 2: verbessert bestehenden pfad
            // Knoten wurde mit 2 Knoten des Pfades verbunden
            if (pathVisibilityChecker_->IsPathVisible(path, getMinimumPath(i), graph_))
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

        if (pathcost < bestCost_)
        {
            bestCost_ = pathcost;
        }
        // ompl::base::PathPtr pp = getPath(vStart_, vGoal);
        // std::cout << "Best cost found: " << bestCost_ << "; cost shortest path: " << pp->length() << std::endl;

        // OMPL_INFORM("Found %d path classes.", getNumberOfPaths());
    }

}
