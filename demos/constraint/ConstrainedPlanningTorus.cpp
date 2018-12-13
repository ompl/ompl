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

#include <ompl/util/PPM.h>
#include <boost/filesystem.hpp>

#include "ConstrainedPlanningCommon.h"

static const double pi2 = 2 * boost::math::constants::pi<double>();

/** Torus manifold. */
class TorusConstraint : public ompl::base::Constraint
{
public:
    const double outer;
    const double inner;

    TorusConstraint(const double outer, const double inner, const std::string &maze)
      : ompl::base::Constraint(3, 1), outer(outer), inner(inner), file_(maze)
    {
        ppm_.loadFile(maze.c_str());
    }

    void getStartAndGoalStates(Eigen::Ref<Eigen::VectorXd> start, Eigen::Ref<Eigen::VectorXd> goal) const
    {
        const double h = ppm_.getHeight() - 1;
        const double w = ppm_.getWidth() - 1;

        for (unsigned int x = 0; x <= w; ++x)
            for (unsigned int y = 0; y <= h; ++y)
            {
                Eigen::Vector2d p = {x / w, y / h};

                auto &c = ppm_.getPixel(x, y);
                if (c.red == 255 && c.blue == 0 && c.green == 0)
                    mazeToAmbient(p, start);

                else if (c.green == 255 && c.blue == 0 && c.red == 0)
                    mazeToAmbient(p, goal);
            }
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        Eigen::Vector3d c = {x[0], x[1], 0};
        out[0] = (x - outer * c.normalized()).norm() - inner;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        const double xySquaredNorm = x[0] * x[0] + x[1] * x[1];
        const double xyNorm = std::sqrt(xySquaredNorm);
        const double denom = std::sqrt(x[2] * x[2] + (xyNorm - outer) * (xyNorm - outer));
        const double c = (xyNorm - outer) * (xyNorm * xySquaredNorm) / (xySquaredNorm * xySquaredNorm * denom);
        out(0, 0) = x[0] * c;
        out(0, 1) = x[1] * c;
        out(0, 2) = x[2] / denom;
    }

    void ambientToMaze(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        Eigen::Vector3d c = {x[0], x[1], 0};

        const double h = ppm_.getHeight();
        const double w = ppm_.getWidth();

        out[0] = std::atan2(x[2], c.norm() - outer) / pi2;
        out[0] += (out[0] < 0);
        out[0] *= h;
        out[1] = std::atan2(x[1], x[0]) / pi2;
        out[1] += (out[1] < 0);
        out[1] *= w;
    }

    void mazeToAmbient(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        Eigen::Vector2d a = x * pi2;

        Eigen::Vector3d b = {std::cos(a[0]), 0, std::sin(a[0])};
        b *= inner;
        b[0] += outer;

        double norm = std::sqrt(b[0] * b[0] + b[1] * b[1]);
        out << std::cos(a[1]), std::sin(a[1]), 0;
        out *= norm;
        out[2] = b[2];
    }

    bool mazePixel(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        const double h = ppm_.getHeight();
        const double w = ppm_.getWidth();

        if (x[0] < 0 || x[0] >= w || x[1] < 0 || x[1] >= h)
            return false;

        const ompl::PPM::Color &c = ppm_.getPixel(x[0], x[1]);
        return !(c.red == 0 && c.blue == 0 && c.green == 0);
    }

    bool isValid(const ompl::base::State *state) const
    {
        auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();
        Eigen::Vector2d coords;
        ambientToMaze(x, coords);

        return mazePixel(coords);
    }

    void dump(std::ofstream &file) const
    {
        file << outer << std::endl;
        file << inner << std::endl;

        boost::filesystem::path path(file_);
        file << boost::filesystem::canonical(path).string() << std::endl;
    }

private:
    const std::string file_;
    ompl::PPM ppm_;
};

bool torusPlanningOnce(ConstrainedProblem &cp, enum PLANNER_TYPE planner, bool output)
{
    cp.setPlanner(planner);

    // Solve the problem
    ob::PlannerStatus stat = cp.solveOnce(output, "torus");

    if (output)
    {
        OMPL_INFORM("Dumping problem information to `torus_info.txt`.");
        std::ofstream infofile("torus_info.txt");
        infofile << cp.type << std::endl;
        dynamic_cast<TorusConstraint *>(cp.constraint.get())->dump(infofile);
        infofile.close();
    }

    cp.atlasStats();

    if (output)
        cp.dumpGraph("torus");

    return stat;
}

bool torusPlanningBench(ConstrainedProblem &cp, std::vector<enum PLANNER_TYPE> &planners)
{
    cp.setupBenchmark(planners, "torus");
    cp.runBenchmark();
    return false;
}

bool torusPlanning(bool output, enum SPACE_TYPE space, std::vector<enum PLANNER_TYPE> &planners,
                   struct ConstrainedOptions &c_opt, struct AtlasOptions &a_opt, bool bench, double outer, double inner,
                   const std::string &maze)
{
    // Create the ambient space state space for the problem.
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);

    ob::RealVectorBounds bounds(3);
    bounds.setLow(-(outer + inner));
    bounds.setHigh(outer + inner);

    rvss->setBounds(bounds);

    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<TorusConstraint>(outer, inner, maze);

    ConstrainedProblem cp(space, rvss, constraint);
    cp.setConstrainedOptions(c_opt);
    cp.setAtlasOptions(a_opt);

    Eigen::Vector3d start, goal;
    constraint->getStartAndGoalStates(start, goal);

    cp.setStartAndGoalStates(start, goal);
    cp.ss->setStateValidityChecker(std::bind(&TorusConstraint::isValid, constraint, std::placeholders::_1));

    if (!bench)
        return torusPlanningOnce(cp, planners[0], output);
    else
        return torusPlanningBench(cp, planners);
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text and planning graph in GraphML to "
                  "`torus_path.txt` and `torus_graph.graphml` respectively.";
auto bench_msg = "Do benchmarking on provided planner list.";
auto outer_msg = "Outer radius of torus.";
auto inner_msg = "Inner radius of torus.";
auto maze_msg = "Filename of maze image (in .ppm format) to use as obstacles on the surface of the torus.";

int main(int argc, char **argv)
{
    bool output, bench;
    enum SPACE_TYPE space = PJ;
    std::vector<enum PLANNER_TYPE> planners = {RRT};

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    double outer, inner;
    boost::filesystem::path path(__FILE__);
    std::string maze = (path.parent_path() / "mazes/thick.ppm").string();

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
    desc.add_options()("bench", po::bool_switch(&bench)->default_value(false), bench_msg);
    desc.add_options()("outer", po::value<double>(&outer)->default_value(2), outer_msg);
    desc.add_options()("inner", po::value<double>(&inner)->default_value(1), inner_msg);
    desc.add_options()("maze,m", po::value<std::string>(&maze), maze_msg);

    addSpaceOption(desc, &space);
    addPlannerOption(desc, &planners);
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

    if (maze == "")
    {
        OMPL_ERROR("--maze is a required.");
        return 1;
    }

    return static_cast<int>(torusPlanning(output, space, planners, c_opt, a_opt, bench, outer, inner, maze));
}
