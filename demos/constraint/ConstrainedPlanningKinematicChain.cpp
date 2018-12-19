/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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

#include "../KinematicChain.h"
#include "ConstrainedPlanningCommon.h"

class ConstrainedKinematicChainValidityChecker : public KinematicChainValidityChecker
{
public:
    ConstrainedKinematicChainValidityChecker(const ob::ConstrainedSpaceInformationPtr &si)
      : KinematicChainValidityChecker(si)
    {
    }

    bool isValid(const ob::State *state) const override
    {
        auto &&space = si_->getStateSpace()->as<ob::ConstrainedStateSpace>()->getSpace()->as<KinematicChainSpace>();
        auto &&s = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
        return isValidImpl(space, s);
    }
};

class KinematicChainConstraint : public ob::Constraint
{
public:
    KinematicChainConstraint(unsigned int links, double linkLength) : ob::Constraint(links, 1), linkLength_(linkLength)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        Eigen::Vector2d e = Eigen::Vector2d::Zero(), eN = Eigen::Vector2d::Zero();
        double theta = 0.;

        for (unsigned int i = 0; i < x.size(); ++i)
        {
            theta += x[i];
            eN[0] = e[0] + cos(theta) * linkLength_;
            eN[1] = e[1] + sin(theta) * linkLength_;
            e = eN;
        }

        out[0] = e[1] - linkLength_;
    }

private:
    double linkLength_;
};

bool chainPlanningOnce(ConstrainedProblem &cp, enum PLANNER_TYPE planner, bool output)
{
    cp.setPlanner(planner);

    // Solve the problem
    ob::PlannerStatus stat = cp.solveOnce(true);

    if (output && stat)
    {
        auto filename = boost::str(boost::format("kinematic_path_%i.dat") % cp.constraint->getAmbientDimension());
        OMPL_INFORM("Dumping problem information to `%s`.", filename.c_str());
        auto path = cp.ss->getSolutionPath();
        path.interpolate();
        std::ofstream pathfile(filename);
        path.printAsMatrix(pathfile);
        pathfile.close();
    }

    cp.atlasStats();

    return stat;
}

bool chainPlanningBench(ConstrainedProblem &cp, std::vector<enum PLANNER_TYPE> &planners)
{
    cp.setupBenchmark(planners, "kinematic");
    cp.bench->addExperimentParameter("links", "INTEGER", std::to_string(cp.constraint->getAmbientDimension()));

    cp.runBenchmark();
    return false;
}

bool chainPlanning(bool output, enum SPACE_TYPE space, std::vector<enum PLANNER_TYPE> &planners, unsigned int links,
                   struct ConstrainedOptions &c_opt, struct AtlasOptions &a_opt, bool bench)
{
    Environment env = createEmptyEnvironment(links);

    // Create the ambient space state space for the problem.
    auto ss = std::make_shared<KinematicChainSpace>(links, 1. / (double)links, &env);

    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<KinematicChainConstraint>(links, 1. / (double)links);

    ConstrainedProblem cp(space, ss, constraint);
    cp.setConstrainedOptions(c_opt);
    cp.setAtlasOptions(a_opt);

    Eigen::VectorXd start, goal;
    start = Eigen::VectorXd::Constant(links, 0);
    goal = Eigen::VectorXd::Constant(links, 0);

    start[links - 1] = boost::math::constants::pi<double>() / 2;
    goal[0] = boost::math::constants::pi<double>();
    goal[links - 1] = -boost::math::constants::pi<double>() / 2;

    cp.setStartAndGoalStates(start, goal);
    cp.ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(cp.csi));

    if (!bench)
        return chainPlanningOnce(cp, planners[0], output);
    else
        return chainPlanningBench(cp, planners);
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) and environment to data files that can be rendered with "
                  "`KinematicChainPathPlot.py`.";
auto links_msg = "Number of links in kinematic chain.";
auto bench_msg = "Do benchmarking on provided planner list.";

int main(int argc, char **argv)
{
    bool output, bench;
    enum SPACE_TYPE space = PJ;
    std::vector<enum PLANNER_TYPE> planners = {RRT};

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    unsigned int links;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
    desc.add_options()("links,l", po::value<unsigned int>(&links)->default_value(5), links_msg);
    desc.add_options()("bench", po::bool_switch(&bench)->default_value(false), bench_msg);

    addSpaceOption(desc, &space);
    addPlannerOption(desc, &planners);
    addConstrainedOptions(desc, &c_opt);
    addAtlasOptions(desc, &a_opt);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return 1;
    }

    return static_cast<int>(chainPlanning(output, space, planners, links, c_opt, a_opt, bench));
}
