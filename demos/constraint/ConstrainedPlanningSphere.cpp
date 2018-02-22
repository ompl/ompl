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

#include "ConstrainedPlanningCommon.h"

class SphereConstraint : public ob::Constraint
{
public:
    SphereConstraint() : ob::Constraint(3, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        out[0] = x.norm() - 1;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }
};

class SphereProjection : public ob::ProjectionEvaluator
{
public:
    SphereProjection(const ob::StateSpacePtr &space) : ob::ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        return 2;
    }

    void defaultCellSizes() override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    void project(const ob::State *state, ob::EuclideanProjection &projection) const override
    {
        auto &&x = state->as<ob::ConstrainedStateSpace::StateType>()->constVectorView();
        projection(0) = atan2(x[1], x[0]);
        projection(1) = acos(x[2]);
    }
};

bool obstacles(const ob::State *state)
{
    auto &&x = state->as<ob::ConstrainedStateSpace::StateType>()->constVectorView();

    if (-0.80 < x[2] && x[2] < -0.55)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] > 0;
        return false;
    }
    else if (-0.15 < x[2] && x[2] < 0.15)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    else if (0.55 < x[2] && x[2] < 0.80)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] < 0;
        return false;
    }

    return true;
}

void spherePlanning(bool output, enum SPACE_TYPE space, enum PLANNER_TYPE planner, struct ConstrainedOptions &c_opt,
                    struct AtlasOptions &a_opt)
{
    // Create the ambient space state space for the problem.
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-2);
    bounds.setHigh(2);

    rvss->setBounds(bounds);

    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<SphereConstraint>();

    ConstrainedProblem cp(space, rvss, constraint);
    cp.setConstrainedOptions(c_opt);
    cp.setAtlasOptions(a_opt);

    cp.css->registerProjection("sphere", std::make_shared<SphereProjection>(cp.css));

    Eigen::VectorXd start(3), goal(3);
    start << 0, 0, -1;
    goal << 0, 0, 1;

    cp.setStartAndGoalStates(start, goal);
    cp.ss->setStateValidityChecker(obstacles);

    cp.setPlanner(planner, "sphere");
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

            OMPL_INFORM("Dumping path to `sphere_path.txt`.");
            std::ofstream pathfile("sphere_path.txt");
            simplePath.printAsMatrix(pathfile);
            pathfile.close();

            OMPL_INFORM("Dumping problem information to `sphere_info.txt`.");
            std::ofstream infofile("sphere_info.txt");
            infofile << space << std::endl;
            infofile.close();
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

    if (output)
    {
        OMPL_INFORM("Dumping planner graph to `sphere_graph.graphml`.");
        ob::PlannerData data(cp.csi);
        cp.pp->getPlannerData(data);

        std::ofstream graphfile("sphere_graph.graphml");
        data.printGraphML(graphfile);
        graphfile.close();

        if (space == AT || space == TB)
        {
            OMPL_INFORM("Dumping atlas to `sphere_atlas.ply`.");
            std::ofstream atlasfile("sphere_atlas.ply");
            cp.css->as<ob::AtlasStateSpace>()->printPLY(atlasfile);
            atlasfile.close();
        }
    }
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text and planning graph in GraphML to "
                  "`sphere_path.txt` and `sphere_graph.graphml` respectively.";

int main(int argc, char **argv)
{
    bool output;
    enum SPACE_TYPE space = PJ;
    enum PLANNER_TYPE planner = RRT;

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);

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

    spherePlanning(output, space, planner, c_opt, a_opt);

    return 0;
}
