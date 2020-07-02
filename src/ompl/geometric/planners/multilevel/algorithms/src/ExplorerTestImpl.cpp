#include <ompl/geometric/planners/multilevel/algorithms/ExplorerTestImpl.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include "ompl/datastructures/PDF.h"
#include "../ExplorerTestImpl.h"
#include <boost/foreach.hpp>
#include <boost/math/constants/constants.hpp>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/geometric/planners/multilevel/datastructures/pathrestriction/PathRestriction.h>
#include <ompl/geometric/planners/explorer/PathVisibilityChecker.h>

#define foreach BOOST_FOREACH

ompl::geometric::ExplorerTestImpl::ExplorerTestImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_)
  : BaseT(si, parent_)
{
    setName("ExplorerTestImpl" + std::to_string(id_));
    pathVisibilityChecker_ = new PathVisibilityChecker(getBundle());
}

ompl::geometric::ExplorerTestImpl::~ExplorerTestImpl()
{
    delete pathVisibilityChecker_;
}

void ompl::geometric::ExplorerTestImpl::grow()
{
    std::cout << "GROW" << std::endl;

    if (firstRun_)
    {
        ExplorerTestImpl::firstGrow();
    }
    else
    {
        //(1) Get Random Sample
        if (!sampleBundleValid(xRandom_->state))
            return;

        //(2) Add Configuration if valid
        Configuration *xNew = new Configuration(getBundle(), xRandom_->state);
        addConfiguration(xNew);

        //(3) Connect to K nearest neighbors
        connectNeighbors(xNew);

        expand();

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
            // og::PathGeometric &gpath = static_cast<og::PathGeometric &>(*pPath);

            ompl::base::PathPtr pPath2 = getPath(v, vGoal_);
            std::vector<Vertex> path2 = shortestVertexPath_;
            // og::PathGeometric &gpath2 = static_cast<og::PathGeometric &>(*pPath2);

            path.insert(path.begin(), path2.begin(), path2.end());
            float pathlength = pPath->length() + pPath2->length();

            std::cout << pPath->length() << std::endl;

            for (int i = 0; i < solutionspaths.size(); i++)
            {
                // FALL 2: verbessert bestehenden pfad
                // Knoten wurde mit 2 Knoten des Pfades verbunden
                //TODO: Does not work
                if (pathVisibilityChecker_->IsPathVisible(path, solutionspaths.at(i), graph_))
                {
                    std::cout << "ITS VISIBLE" << std::endl;
                    // Path is shorter than visible path else do nothing
                    if (pathlength < solutionPathLength.at(i))
                    {
                        solutionspaths.at(i) = path;
                        solutionPathLength.at(i) = pathlength;
                    }
                }
            }

            // FALL 3: neuen Pfad
            // Knoten kann mit Start und Ziel verbunden werden
            std::cout << "NOT VISIBLE" << std::endl;
            solutionspaths.push_back(path);
            solutionPathLength.push_back(pathlength);
        }

        /*for (int i = 0; i < solutionspaths.size(); i++)
        {
            std::cout << "Path: ";
            for (int j = 0 ; j < solutionspaths.at(i).size(); j++)
            {
                std::cout << solutionspaths.at(i).at(j) << ' ';
            }
            std::cout << "\t Length: ";
            std::cout << solutionPathLength.at(i) << std::endl;
        }*/
    }
}

void ompl::geometric::ExplorerTestImpl::firstGrow()
{
    std::cout << "FIRST" << std::endl;

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

    while (!hasSolution_)
    {
        //(1) Get Random Sample
        if (!sampleBundleValid(xRandom_->state))
            return;

        //(2) Add Configuration if valid
        Configuration *xNew = new Configuration(getBundle(), xRandom_->state);
        addConfiguration(xNew);

        //(3) Connect to K nearest neighbors
        connectNeighbors(xNew);

        expand();

        if (!hasSolution_)
        {
            if (sameComponent(vStart_, vGoal_))
            {
                hasSolution_ = true;

                Vertex v = xNew->index;

                ompl::base::PathPtr pPath = getPath(vStart_, v);
                std::vector<Vertex> path = shortestVertexPath_;

                ompl::base::PathPtr pPath2 = getPath(v, vGoal_);
                std::vector<Vertex> path2 = shortestVertexPath_;

                path.insert(path.begin(), path2.begin(), path2.end());
                float pathlength = pPath->length() + pPath2->length();

                solutionspaths.push_back(path);
                solutionPathLength.push_back(pathlength);
            }
        }
    }

    //  base::PathPtr toStart = getPath(vStart_, xNew->index);
    //  base::PathPtr toGoal = getPath(vGoal_, xNew->index);

    // PathGeometric path = getPath(vStart_,vGoal_)->as<PathGeometric>();
    // solutions_.push_back(path.getStates());
}
