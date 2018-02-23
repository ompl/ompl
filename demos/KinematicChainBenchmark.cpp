/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Bryant Gipson, Mark Moll */

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTplus.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

// a 2D line segment
struct Segment
{
    Segment(double p0_x, double p0_y, double p1_x, double p1_y)
        : x0(p0_x), y0(p0_y), x1(p1_x), y1(p1_y)
    {
    }
    double x0, y0, x1, y1;
};

// the robot and environment are modeled both as a vector of segments.
using Environment = std::vector<Segment>;

// simply use a random projection
class KinematicChainProjector : public ompl::base::ProjectionEvaluator
{
public:
    KinematicChainProjector(const ompl::base::StateSpace *space)
        : ompl::base::ProjectionEvaluator(space)
    {
        int dimension = std::max(2, (int)ceil(log((double) space->getDimension())));
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }
    unsigned int getDimension() const override
    {
        return projectionMatrix_.mat.size1();
    }
    void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const override
    {
        std::vector<double> v(space_->getDimension());
        space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }
protected:
    ompl::base::ProjectionMatrix projectionMatrix_;
};


class KinematicChainSpace : public ompl::base::CompoundStateSpace
{
public:
    KinematicChainSpace(unsigned int numLinks, double linkLength, Environment *env = nullptr)
        : linkLength_(linkLength), environment_(env)
    {
        for (unsigned int i = 0; i < numLinks; ++i)
            addSubspace(std::make_shared<ompl::base::SO2StateSpace>(), 1.);
        lock();
    }

    void registerProjections() override
    {
        registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    }

    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {
        const auto *cstate1 = state1->as<StateType>();
        const auto *cstate2 = state2->as<StateType>();
        double theta1 = 0., theta2 = 0., dx = 0., dy = 0., dist = 0.;

        for (unsigned int i = 0; i < getSubspaceCount(); ++i)
        {
            theta1 += cstate1->as<ompl::base::SO2StateSpace::StateType>(i)->value;
            theta2 += cstate2->as<ompl::base::SO2StateSpace::StateType>(i)->value;
            dx += cos(theta1) - cos(theta2);
            dy += sin(theta1) - sin(theta2);
            dist += sqrt(dx * dx + dy * dy);
        }
        return dist * linkLength_;
    }
    double linkLength() const
    {
        return linkLength_;
    }
    const Environment* environment() const
    {
        return environment_;
    }

protected:
    double linkLength_;
    Environment* environment_;
};


class KinematicChainValidityChecker : public ompl::base::StateValidityChecker
{
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si)
        : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const ompl::base::State *state) const override
    {
        const KinematicChainSpace* space = si_->getStateSpace()->as<KinematicChainSpace>();
        const auto *s = state->as<KinematicChainSpace::StateType>();
        unsigned int n = si_->getStateDimension();
        Environment segments;
        double linkLength = space->linkLength();
        double theta = 0., x = 0., y = 0., xN, yN;

        segments.reserve(n + 1);
        for(unsigned int i = 0; i < n; ++i)
        {
            theta += s->as<ompl::base::SO2StateSpace::StateType>(i)->value;
            xN = x + cos(theta) * linkLength;
            yN = y + sin(theta) * linkLength;
            segments.emplace_back(x, y, xN, yN);
            x = xN;
            y = yN;
        }
        xN = x + cos(theta) * 0.001;
        yN = y + sin(theta) * 0.001;
        segments.emplace_back(x, y, xN, yN);
        return selfIntersectionTest(segments)
            && environmentIntersectionTest(segments, *space->environment());
    }

protected:
    // return true iff env does *not* include a pair of intersecting segments
    bool selfIntersectionTest(const Environment& env) const
    {
        for (unsigned int i = 0; i < env.size(); ++i)
            for (unsigned int j = i + 1; j < env.size(); ++j)
                if (intersectionTest(env[i], env[j]))
                    return false;
        return true;
    }
    // return true iff no segment in env0 intersects any segment in env1
    bool environmentIntersectionTest(const Environment& env0, const Environment& env1) const
    {
        for (const auto & i : env0)
            for (const auto & j : env1)
                if (intersectionTest(i, j))
                    return false;
        return true;
    }
    // return true iff segment s0 intersects segment s1
    bool intersectionTest(const Segment& s0, const Segment& s1) const
    {
        // adopted from:
        // http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/1201356#1201356
        double s10_x = s0.x1 - s0.x0;
        double s10_y = s0.y1 - s0.y0;
        double s32_x = s1.x1 - s1.x0;
        double s32_y = s1.y1 - s1.y0;
        double denom = s10_x * s32_y - s32_x * s10_y;
        if (fabs(denom) < std::numeric_limits<double>::epsilon())
            return false; // Collinear
        bool denomPositive = denom > 0;

        double s02_x = s0.x0 - s1.x0;
        double s02_y = s0.y0 - s1.y0;
        double s_numer = s10_x * s02_y - s10_y * s02_x;
        if ((s_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
            return false; // No collision
        double t_numer = s32_x * s02_y - s32_y * s02_x;
        if ((t_numer < std::numeric_limits<float>::epsilon()) == denomPositive)
            return false; // No collision
        if (((s_numer - denom > -std::numeric_limits<float>::epsilon()) == denomPositive)
            || ((t_numer - denom > std::numeric_limits<float>::epsilon()) == denomPositive))
            return false; // No collision
        return true;
    }
};


Environment createHornEnvironment(unsigned int d, double eps)
{
    std::ofstream envFile(boost::str(boost::format("environment_%i.dat") % d));
    std::vector<Segment> env;
    double w = 1. / (double)d, x = w, y = -eps, xN, yN, theta = 0.,
        scale = w * (1. + boost::math::constants::pi<double>() * eps);

    envFile << x << " " << y << std::endl;
    for(unsigned int i = 0; i < d - 1; ++i)
    {
        theta += boost::math::constants::pi<double>() / (double) d;
        xN = x + cos(theta) * scale;
        yN = y + sin(theta) * scale;
        env.emplace_back(x, y, xN, yN);
        x = xN;
        y = yN;
        envFile << x << " " << y << std::endl;
    }

    theta = 0.;
    x = w;
    y = eps;
    envFile << x << " " << y << std::endl;
    scale = w * (1.0 - boost::math::constants::pi<double>() * eps);
    for(unsigned int i = 0; i < d - 1; ++i)
    {
        theta += boost::math::constants::pi<double>() / d;
        xN = x + cos(theta) * scale;
        yN = y + sin(theta) * scale;
        env.emplace_back(x, y, xN, yN);
        x = xN;
        y = yN;
        envFile << x << " " << y << std::endl;
    }
    envFile.close();
    return env;
}

// A function that matches the ompl::base::PlannerAllocator type.
// It will be used in main() to allocate an instance of RRT+
ompl::base::PlannerPtr myConfiguredPlanner(const ompl::base::SpaceInformationPtr &si)
{
    ompl::geometric::RRTPlus *rrt_plus = new ompl::geometric::RRTPlus(si);
    rrt_plus->setRange(1.0);
    rrt_plus->setGoalBias(0.05);
    rrt_plus->setSubsearchBound(60.0);
    return ompl::base::PlannerPtr(rrt_plus);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage:\n" << argv[0] << " <num_links>\n";
        exit(0);
    }

    auto numLinks = boost::lexical_cast<unsigned int>(std::string(argv[1]));
    Environment env = createHornEnvironment(numLinks, log((double)numLinks) / (double)numLinks);
    auto chain(std::make_shared<KinematicChainSpace>(numLinks, 1. / (double)numLinks, &env));
    ompl::geometric::SimpleSetup ss(chain);

    ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation()));

    ompl::base::ScopedState<> start(chain), goal(chain);
    std::vector<double> startVec(numLinks, boost::math::constants::pi<double>() / (double)numLinks);
    std::vector<double> goalVec(numLinks, 0.);

    startVec[0] = 0.;
    goalVec[0] = boost::math::constants::pi<double>() - .001;
    chain->setup();
    chain->copyFromReals(start.get(), startVec);
    chain->copyFromReals(goal.get(), goalVec);
    ss.setStartAndGoalStates(start, goal);

    // SEKRIT BONUS FEATURE:
    // if you specify a second command line argument, it will solve the
    // problem just once with STRIDE and print out the solution path.
    if (argc > 2)
    {
        ss.setPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
        ss.setup();
        ss.print();
        ss.solve(3600);
        ss.simplifySolution();

        std::ofstream pathfile(
                    boost::str(boost::format("kinematic_path_%i.dat") % numLinks).c_str());
        ss.getSolutionPath().printAsMatrix(pathfile);
        exit(0);
    }

    // by default, use the Benchmark class
    double runtime_limit = 60, memory_limit = 1024;
    int run_count = 20;
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count, 0.5);
    ompl::tools::Benchmark b(ss, "KinematicChain");
    b.addExperimentParameter("num_links", "INTEGER", std::to_string(numLinks));

    b.addPlanner(std::make_shared<ompl::geometric::STRIDE>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::EST>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::KPIECE1>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::RRT>(ss.getSpaceInformation()));
    b.addPlanner(std::make_shared<ompl::geometric::PRM>(ss.getSpaceInformation()));

    b.addPlannerAllocator(std::bind(&myConfiguredPlanner, std::placeholders::_1));

    b.benchmark(request);
    b.saveResultsToFile(boost::str(boost::format("kinematic_%i.log") % numLinks).c_str());

    exit(0);
}
