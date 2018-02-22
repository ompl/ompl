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
        Eigen::Vector2d xv = Eigen::Vector2d::Zero();
        Eigen::Vector2d xN = Eigen::Vector2d::Zero();

        double theta = 0;
        const unsigned int n = x.size();

        for (unsigned int i = 0; i < n; ++i)
        {
            theta += x[i];
            xN[0] = xv[0] + cos(theta) * linkLength_;
            xN[1] = xv[1] + sin(theta) * linkLength_;
            xv = xN;
        }
        xN[0] = xv[0] + cos(theta) * 0.001;
        xN[1] = xv[1] + sin(theta) * 0.001;

        out[0] = xN.norm() - 0.636911;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }

private:
    double linkLength_;
};

void chainPlanning(bool output, enum SPACE_TYPE space, enum PLANNER_TYPE planner, unsigned int links,
                   struct ConstrainedOptions &c_opt, struct AtlasOptions &a_opt)
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
    start = Eigen::VectorXd::Constant(links, boost::math::constants::pi<double>() / (double)(links + 1));
    goal = Eigen::VectorXd::Constant(links, -boost::math::constants::pi<double>() / (double)(links + 1));

    cp.setStartAndGoalStates(start, goal);
    cp.ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(cp.csi));

    cp.setPlanner(planner);
    cp.ss->setup();

    // Solve the problem
    ob::PlannerStatus stat = cp.ss->solve(5.);
    std::cout << std::endl;

    if (stat)
    {
        // Get solution and validate
        auto path = cp.ss->getSolutionPath();
        if (!path.check())
            OMPL_WARN("Path fails check!");

        if (stat == ob::PlannerStatus::APPROXIMATE_SOLUTION)
            OMPL_WARN("Solution is approximate.");

        // Simplify solution and validate simplified solution path.
        OMPL_INFORM("Simplifying solution...");
        cp.ss->simplifySolution(5.);

        auto simplePath = cp.ss->getSolutionPath();
        OMPL_INFORM("Simplified Path Length: %.3f -> %.3f", path.length(), simplePath.length());

        if (!simplePath.check())
            OMPL_WARN("Simplified path fails check!");

        if (output)
        {
            // Interpolate and validate interpolated solution path.
            OMPL_INFORM("Interpolating path...");
            simplePath.interpolate();

            if (!simplePath.check())
                OMPL_WARN("Interpolated path fails check!");

            std::ofstream pathfile(boost::str(boost::format("kinematic_path_%i.dat") % links).c_str());
            simplePath.printAsMatrix(pathfile);
            pathfile.close();
        }
    }
    else
        OMPL_WARN("No solution found.");

    // For atlas types, output information about size of atlas and amount of space explored
    if (space == AT || space == TB)
    {
        auto at = cp.css->as<ob::AtlasStateSpace>();
        OMPL_INFORM("Atlas has %zu charts", at->getChartCount());
        if (space == AT)
            OMPL_INFORM("Atlas is approximately %.3f%% open", at->estimateFrontierPercent());
    }
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) and environment to data files that can be rendered with "
                  "`KinematicChainPathPlot.py`.";
auto links_msg = "Number of links in kinematic chain.";

int main(int argc, char **argv)
{
    bool output;
    enum SPACE_TYPE space = PJ;
    enum PLANNER_TYPE planner = RRT;
    unsigned int links;

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
    desc.add_options()("links,l", po::value<unsigned int>(&links)->default_value(5), links_msg);

    addSpaceOption(desc, &space);
    addPlannerOption(desc, &planner);
    addConstrainedOptions(desc, &c_opt);
    addAtlasOptions(desc, &a_opt);

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    chainPlanning(output, space, planner, links, c_opt, a_opt);

    return 0;
}
