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

/* Author: Zachary Kingston */

#include "ConstrainedPlanningCommon.h"

/** Print usage information. Does not return. */
void usage(const char *const progname)
{
    std::cout << "Usage: " << progname << " -c <problem> -p <planner> -s <space> -t <timelimit> -w <sleep> -o\n";
    printProblems();
    printPlanners();
    exit(0);
}

enum SPACE
{
    ATLAS,
    PROJECTED,
    NULLSPACE
};

int main(int argc, char **argv)
{
    int c;
    opterr = 0;

    const char *plannerName = "RRTConnect";
    const char *problem = "sphere";
    const char *space = "projected";

    double artificalSleep = 0.0;
    double planningTime = 5.0;
    bool output = false;
    bool tb = true;
    int iter = 0;

    unsigned int links = 5;

    while ((c = getopt(argc, argv, "c:p:s:w:ot:n:i:a")) != -1)
    {
        switch (c)
        {
            case 'c':
                problem = optarg;
                break;

            case 'a':
                tb = false;
                break;

            case 'i':
                iter = atoi(optarg);
                break;

            case 'p':
                plannerName = optarg;
                break;

            case 's':
                space = optarg;
                break;

            case 'w':
                artificalSleep = atof(optarg);
                break;

            case 't':
                planningTime = atof(optarg);
                break;

            case 'n':
                links = atoi(optarg);
                break;

            case 'o':
                output = true;
                break;

            default:
                usage(argv[0]);
                break;
        }
    }

    enum SPACE spaceType = PROJECTED;

    if (std::strcmp("atlas", space) == 0)
        spaceType = ATLAS;
    else if (std::strcmp("projected", space) == 0)
        spaceType = PROJECTED;
    else if (std::strcmp("null", space) == 0)
        spaceType = NULLSPACE;
    else
    {
        std::cout << "Invalid constrained state space." << std::endl;
        usage(argv[0]);
    }

    Eigen::VectorXd x, y;
    ompl::base::StateValidityCheckerFn isValid;
    ompl::base::Constraint *constraint = parseProblem(problem, x, y, isValid, artificalSleep, links);

    if (!constraint)
    {
        std::cout << "Invalid problem." << std::endl;
        usage(argv[0]);
    }

    printf("Constrained Planning Testing: \n"
           "  Planning with in `%s' state space with `%s' for `%s' problem.\n"
           "  Timeout: %3.2fs   Artifical Delay: %3.2fs\n",
           space, plannerName, problem, planningTime, artificalSleep);

    ompl::base::ConstrainedStateSpacePtr css;
    ompl::geometric::SimpleSetupPtr ss;
    ompl::base::SpaceInformationPtr si;

    double range = 0.707;

    switch (spaceType)
    {
        case ATLAS:
        {
            ompl::base::AtlasStateSpacePtr atlas(
                new ompl::base::AtlasStateSpace(constraint->getAmbientSpace(), constraint));

            atlas->setExploration(0.5);
            atlas->setAlpha(M_PI / 8);
            atlas->setMaxChartsPerExtension(200);
            atlas->setRho(0.5);
            atlas->setEpsilon(0.2);
            atlas->setSeparate(tb);

            range = atlas->getRho_s();

            ss = ompl::geometric::SimpleSetupPtr(new ompl::geometric::SimpleSetup(atlas));
            si = ss->getSpaceInformation();
            si->setValidStateSamplerAllocator(avssa);

            atlas->setSpaceInformation(si);

            // The atlas needs some place to start sampling from. We will make start and goal charts.
            ompl::base::AtlasChart *startChart = atlas->anchorChart(x);
            ompl::base::AtlasChart *goalChart = atlas->anchorChart(y);

            ompl::base::ScopedState<> start(atlas);
            ompl::base::ScopedState<> goal(atlas);
            start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, startChart);
            goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, goalChart);

            ss->setStartAndGoalStates(start, goal);

            css = atlas;
            break;
        }

        case PROJECTED:
        {
            ompl::base::ProjectedStateSpacePtr proj(
                new ompl::base::ProjectedStateSpace(constraint->getAmbientSpace(), constraint));
            ss = ompl::geometric::SimpleSetupPtr(new ompl::geometric::SimpleSetup(proj));
            si = ss->getSpaceInformation();
            si->setValidStateSamplerAllocator(pvssa);

            proj->setSpaceInformation(si);

            // The proj needs some place to start sampling from. We will make start
            // and goal charts.
            ompl::base::ScopedState<> start(proj);
            ompl::base::ScopedState<> goal(proj);
            start->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(x);
            goal->as<ompl::base::ProjectedStateSpace::StateType>()->setRealState(y);
            ss->setStartAndGoalStates(start, goal);

            css = proj;
            break;
        }

        case NULLSPACE:
        {
            ompl::base::NullspaceStateSpacePtr proj(
                new ompl::base::NullspaceStateSpace(constraint->getAmbientSpace(), constraint));

            ss = ompl::geometric::SimpleSetupPtr(new ompl::geometric::SimpleSetup(proj));
            si = ss->getSpaceInformation();
            si->setValidStateSamplerAllocator(pvssa);

            proj->setSpaceInformation(si);

            // The proj needs some place to start sampling from. We will make start
            // and goal charts.
            ompl::base::ScopedState<> start(proj);
            ompl::base::ScopedState<> goal(proj);
            start->as<ompl::base::NullspaceStateSpace::StateType>()->setRealState(x);
            goal->as<ompl::base::NullspaceStateSpace::StateType>()->setRealState(y);
            ss->setStartAndGoalStates(start, goal);

            css = proj;
            break;
        }
    }

    ss->setStateValidityChecker(isValid);

    // Bounds
    int bound = 20;

    ompl::base::RealVectorBounds bounds(css->getAmbientDimension());
    bounds.setLow(-bound);
    bounds.setHigh(bound);

    css->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Choose the planner.
    ompl::base::PlannerPtr planner(parsePlanner(plannerName, si, range));
    if (!planner)
    {
        std::cout << "Invalid planner." << std::endl;
        usage(argv[0]);
    }

    ss->setPlanner(planner);
    ss->setup();

    css->setDelta(css->getMaximumExtent() / 1000);

    if (spaceType == ATLAS)
    {
        if (css->getAmbientDimension() > 40)
            css->as<ompl::base::AtlasStateSpace>()->setExploration(0.9);
        else if (css->getAmbientDimension() > 30)
            css->as<ompl::base::AtlasStateSpace>()->setExploration(0.85);
        else if (css->getAmbientDimension() > 20)
            css->as<ompl::base::AtlasStateSpace>()->setExploration(0.8);

        css->as<ompl::base::AtlasStateSpace>()->setRho(css->getMaximumExtent() / 500);
        css->as<ompl::base::AtlasStateSpace>()->setEpsilon(css->getMaximumExtent() / 100);
        css->as<ompl::base::AtlasStateSpace>()->setMaxChartsPerExtension(css->getMaximumExtent() * 10);
    }

    std::clock_t tstart = std::clock();

    ompl::base::PlannerStatus stat;
    if (iter)
    {
        ompl::base::IterationTerminationCondition cond(iter);
        stat = planner->solve(cond);
        std::cout << cond.getTimesCalled() << "/" << iter << " iterations." << std::endl;
    }
    else
        stat = planner->solve(planningTime);

    if (stat)
    {
        const double time = ((double)(std::clock() - tstart)) / CLOCKS_PER_SEC;

        ss->simplifySolution();

        ompl::geometric::PathGeometric &path = ss->getSolutionPath();

        if (output)
        {
            std::cout << "Interpolating path..." << std::endl;
            path.interpolate();
        }

        if (x.size() == 3 && output)
        {
            std::cout << "Dumping path mesh..." << std::endl;
            std::ofstream pathFile("path.ply");
            path.dumpPath(pathFile);
            pathFile.close();
        }

        // Extract the full solution path by re-interpolating between the
        // saved states (except for the special planners)
        const std::vector<ompl::base::State *> &waypoints = path.getStates();

        double length = 0;
        for (std::size_t i = 1; i < waypoints.size() - 1; i++)
            length += css->distance(waypoints[i - 1], waypoints[i]);

        if (output)
        {
            std::cout << "Dumping animation file..." << std::endl;
            std::ofstream animFile("anim.txt");
            for (std::size_t i = 0; i < waypoints.size() - 1; i++)
                animFile << waypoints[i]
                                ->as<ompl::base::ConstrainedStateSpace::StateType>()
                                ->constVectorView()
                                .transpose()
                         << "\n";
            animFile.close();
        }

        if (stat == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
            std::cout << "Solution is approximate.\n";

        std::cout << "Length: " << length << "\n";
        std::cout << "Took " << time << " seconds.\n";
    }
    else
    {
        std::cout << "No solution found.\n";
    }

    ompl::base::PlannerData data(si);
    planner->getPlannerData(data);
    if (data.properties.find("approx goal distance REAL") != data.properties.end())
        std::cout << "Approx goal distance: " << data.properties["approx goal distance REAL"] << "\n";

    if (spaceType == ATLAS)
    {
        std::cout << "Atlas created " << css->as<ompl::base::AtlasStateSpace>()->getChartCount() << " charts.\n";
        std::cout << css->as<ompl::base::AtlasStateSpace>()->estimateFrontierPercent() << "% open.\n";
    }

    if (x.size() == 3 && output)
    {
        std::cout << "Dumping graph mesh..." << std::endl;
        std::ofstream graphFile("graph.ply");
        ompl::base::PlannerData pd(si);
        planner->getPlannerData(pd);
        pd.dumpGraph(graphFile, false);
        graphFile.close();

        if (spaceType == ATLAS)
        {
            std::cout << "Dumping atlas mesh..." << std::endl;
            std::ofstream atlasFile("atlas.ply");
            css->as<ompl::base::AtlasStateSpace>()->dumpMesh(atlasFile);
            atlasFile.close();
        }
    }

    return 0;
}
