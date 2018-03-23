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

class StewartBase
{
public:
    virtual void getStart(Eigen::VectorXd &x) = 0;
    virtual void getGoal(Eigen::VectorXd &x) = 0;
};

class StewartChain : public ompl::base::Constraint, public StewartBase
{
public:
    StewartChain(const unsigned int n, Eigen::VectorXd offset, unsigned int links, unsigned int id, double length = 1,
                 double jointSize = 0.2, unsigned int extra = 0)
      : ompl::base::Constraint(n, n - links)
      , offset_(offset)
      , links_(links)
      , id_(id)
      , length_(length)
      , jointSize_(jointSize)
      , extra_(extra)
    {
        if (links % 2 == 0)
            throw ompl::Exception("Number of links must be odd!");
    }

    void getConfiguration(Eigen::VectorXd &x, double angle)
    {
        unsigned int offset = 3 * links_ * id_;
        const Eigen::VectorXd axis = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) * offset_;

        const Eigen::VectorXd step = Eigen::Vector3d::UnitZ() * length_;
        Eigen::VectorXd joint = offset_ + Eigen::AngleAxisd(angle, axis) * step;

        unsigned int i = 0;
        for (; i < links_; ++i)
        {
            x.segment(3 * i + offset, 3) = joint;
            if (i < links_ - 2)
                joint += step;
            else
                joint += Eigen::AngleAxisd(-angle, axis) * step;
        }
    }

    void getStart(Eigen::VectorXd &x)
    {
        getConfiguration(x, M_PI / 16);
    }

    void getGoal(Eigen::VectorXd &x)
    {
        unsigned int offset = 3 * links_ * id_;

        Eigen::VectorXd nstep = offset_ * length_;
        Eigen::VectorXd estep = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) * offset_ * length_;
        Eigen::VectorXd sstep = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * offset_ * length_;
        Eigen::VectorXd wstep = Eigen::AngleAxisd(3 * M_PI / 2, Eigen::Vector3d::UnitZ()) * offset_ * length_;

        Eigen::VectorXd joint = offset_ + nstep;
        x.segment(3 * 0 + offset, 3) = joint;
        x.segment(3 * 1 + offset, 3) = x.segment(3 * 0 + offset, 3) + estep;
        x.segment(3 * 2 + offset, 3) = x.segment(3 * 1 + offset, 3) + estep;
        x.segment(3 * 3 + offset, 3) = x.segment(3 * 2 + offset, 3) + Eigen::Vector3d::UnitZ() * length_;
        x.segment(3 * 4 + offset, 3) = x.segment(3 * 3 + offset, 3) + sstep;
        x.segment(3 * 5 + offset, 3) = x.segment(3 * 4 + offset, 3) + sstep;
        x.segment(3 * 6 + offset, 3) = x.segment(3 * 5 + offset, 3) + wstep;

        /*     Eigen::VectorXd joint = offset_ + step; */

        /*     unsigned int i = 0; */
        /*     for (; i < links_ / 2; ++i, joint += step) */
        /*         x.segment(3 * i + offset, 3) = joint; */

        /*     joint += Eigen::Vector3d::UnitZ() * length_ - step; */
        /*     for (; i < links_; ++i, joint -= step) */
        /*         x.segment(3 * i + offset, 3) = joint; */
    }

    Eigen::Ref<const Eigen::VectorXd> getLink(const Eigen::VectorXd &x, const unsigned int idx) const
    {
        const unsigned int offset = 3 * links_ * id_;
        return x.segment(offset + 3 * idx, 3);
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        unsigned int idx = 0;

        Eigen::VectorXd j1 = offset_;
        for (unsigned int i = 0; i < links_; ++i)
        {
            const Eigen::VectorXd j2 = getLink(x, i);
            out[idx++] = (j1 - j2).norm() - length_;
            j1 = j2;
        }
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        const unsigned int offset = 3 * links_ * id_;
        out.setZero();

        Eigen::VectorXd plus(3 * (links_ + 1));
        plus.head(3 * links_) = x.segment(offset, 3 * links_);
        plus.tail(3) = Eigen::VectorXd::Zero(3);

        Eigen::VectorXd minus(3 * (links_ + 1));
        minus.head(3) = offset_;
        minus.tail(3 * links_) = x.segment(offset, 3 * links_);

        const Eigen::VectorXd diagonal = plus - minus;

        for (unsigned int i = 0; i < links_; i++)
            out.row(i).segment(3 * i + offset, 3) = diagonal.segment(3 * i, 3).normalized();

        out.block(1, offset, links_ - 1, 3 * links_ - 3) -= out.block(1, offset + 3, links_ - 1, 3 * links_ - 3);
    }

private:
    const Eigen::VectorXd offset_;
    const unsigned int links_;
    const unsigned int id_;
    const double length_;
    const double jointSize_;
    const unsigned int extra_;
};

class StewartPlatform : public ompl::base::Constraint, public StewartBase
{
public:
    StewartPlatform(const unsigned int n, unsigned int chains, unsigned int links, double radius = 1)
      : ompl::base::Constraint(n, n - chains), chains_(chains), links_(links), radius_(radius)
    {
        if (chains == 2)
            setManifoldDimension(k_ + 1);

        if (chains >= 4)
            setManifoldDimension(k_ - (chains - 3));
    }

    Eigen::Ref<const Eigen::VectorXd> getTip(const Eigen::VectorXd &x, unsigned int id) const
    {
        return x.segment(3 * links_ * ((id % chains_) + 1) - 3, 3);
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        if (chains_ == 2)
        {
            out[0] = (getTip(x, 0) - getTip(x, 1)).norm() - radius_ * 2;
            return;
        }

        unsigned int idx = 0;

        Eigen::VectorXd centroid = Eigen::VectorXd::Zero(3);
        for (unsigned int i = 0; i < chains_; ++i)
            centroid += getTip(x, i);
        centroid /= chains_;

        for (unsigned int i = 0; i < chains_; ++i)
            out[idx++] = (centroid - getTip(x, i)).norm() - radius_;

        for (int i = 0; i < static_cast<int>(chains_) - 3; ++i)
        {
            Eigen::Ref<const Eigen::Vector3d> ab = getTip(x, i + 1) - getTip(x, i);
            Eigen::Ref<const Eigen::Vector3d> ac = getTip(x, i + 2) - getTip(x, i);
            Eigen::Ref<const Eigen::Vector3d> ad = getTip(x, i + 3) - getTip(x, i);

            out[idx++] = ad.dot(ab.cross(ac));
        }
    }

    void getStart(Eigen::VectorXd &x)
    {
    }

    void getGoal(Eigen::VectorXd &x)
    {
    }

private:
    const unsigned int chains_;
    const unsigned int links_;
    const double radius_;
};

class StewartConstraint : public ompl::base::ConstraintIntersection
{
public:
    StewartConstraint(unsigned int chains, unsigned int links, unsigned int extra = 0, double radius = 1,
                      double length = 1, double jointSize = 0.2)
      : ompl::base::ConstraintIntersection(chains * links * 3, {})
      , chains_(chains)
      , links_(links)
      , radius_(radius)
      , length_(length)
      , jointSize_(jointSize)
    {
        const unsigned int dof = chains * links * 3;
        Eigen::VectorXd offset = Eigen::Vector3d::UnitX();
        for (unsigned int i = 0; i < chains_; ++i)
        {
            addConstraint(new StewartChain(dof, offset, links, i, length, jointSize, extra));
            offset = Eigen::AngleAxisd(2 * M_PI / static_cast<double>(chains), Eigen::Vector3d::UnitZ()) * offset;
        }

        addConstraint(new StewartPlatform(dof, chains, links, radius));
    }

    void getStart(Eigen::VectorXd &x)
    {
        for (unsigned int i = 0; i < constraints_.size(); ++i)
            dynamic_cast<StewartBase *>(constraints_[i])->getStart(x);
    }

    void getGoal(Eigen::VectorXd &x)
    {
        for (unsigned int i = 0; i < constraints_.size(); ++i)
            dynamic_cast<StewartBase *>(constraints_[i])->getGoal(x);
    }

    bool isValid(const ompl::base::State *state)
    {
        Eigen::Ref<const Eigen::VectorXd> x =
            state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView();

        for (unsigned int i = 0; i < links_ * chains_; ++i)
        {
            if (x.segment(3 * i, 3)[2] < 0)
                return false;
        }

        for (unsigned int i = 0; i < links_ * chains_ - 1; ++i)
        {
            if (x.segment(3 * i, 3).cwiseAbs().maxCoeff() < jointSize_)
                return false;

            for (unsigned int j = i + 1; j < links_ * chains_; ++j)
                if ((x.segment(3 * i, 3) - x.segment(3 * j, 3)).cwiseAbs().maxCoeff() < jointSize_)
                    return false;
        }

        return true;
    }

private:
    const unsigned int chains_;
    const unsigned int links_;
    const double radius_;
    const double length_;
    const double jointSize_;
};

bool chainPlanningOnce(ConstrainedProblem &cp, enum PLANNER_TYPE planner, bool output)
{
    cp.setPlanner(planner, "chain");

    // Solve the problem
    ob::PlannerStatus stat = cp.solveOnce(output, "chain");

    if (output)
    {
        OMPL_INFORM("Dumping problem information to `chain_info.txt`.");
        std::ofstream infofile("chain_info.txt");
        infofile << cp.type << std::endl;
        dynamic_cast<ChainConstraint *>(cp.constraint.get())->dump(infofile);
        infofile.close();
    }

    cp.atlasStats();

    return stat;
}

bool chainPlanningBench(ConstrainedProblem &cp, std::vector<enum PLANNER_TYPE> &planners)
{
    cp.setupBenchmark(planners, "chain");

    auto chain = dynamic_cast<ChainConstraint *>(cp.constraint.get());
    chain->addBenchmarkParameters(cp.bench);

    cp.runBenchmark();

    return 0;
}

bool chainPlanning(bool output, enum SPACE_TYPE space, std::vector<enum PLANNER_TYPE> &planners, unsigned int links,
                   unsigned int obstacles, unsigned int extra, struct ConstrainedOptions &c_opt,
                   struct AtlasOptions &a_opt, bool bench)
{
    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<ChainConstraint>(links, obstacles, extra);

    ConstrainedProblem cp(space, constraint->createSpace(), constraint);
    cp.setConstrainedOptions(c_opt);
    cp.setAtlasOptions(a_opt);

    cp.css->registerProjection("chain", constraint->getProjection(cp.css));

    Eigen::VectorXd start, goal;
    constraint->setStartAndGoalStates(start, goal);

    cp.setStartAndGoalStates(start, goal);
    cp.ss->setStateValidityChecker(std::bind(&ChainConstraint::isValid, constraint, std::placeholders::_1));

    if (!bench)
        return chainPlanningOnce(cp, planners[0], output);
    else
        return chainPlanningBench(cp, planners);
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text to `chain_path.txt`. "
                  "Problem information is dumped to `chain_info`.txt";
auto links_msg = "Number of links in the kinematic chain. Minimum is 4.";
auto obstacles_msg = "Number of `wall' obstacles on the surface of the sphere. Ranges from [0, 2]";
auto extra_msg = "Number of extra constraints to add to the chain. Extra constraints are as follows:\n"
                 "1: End-effector is constrained to be on the surface of a sphere of radius links - 2\n"
                 "2: (links-5)th and (links-4)th ball have the same z-value\n"
                 "3: (links-4)th and (links-3)th ball have the same x-value\n"
                 "4: (links-3)th and (links-2)th ball have the same z-value";
auto bench_msg = "Do benchmarking on provided planner list.";

int main(int argc, char **argv)
{
    bool output, bench;
    enum SPACE_TYPE space = PJ;
    std::vector<enum PLANNER_TYPE> planners = {RRT};

    unsigned int links = 5;
    unsigned int obstacles = 0;
    unsigned int extra = 1;

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
    desc.add_options()("links,l", po::value<unsigned int>(&links)->default_value(5), links_msg);
    desc.add_options()("obstacles,x", po::value<unsigned int>(&obstacles)->default_value(0), obstacles_msg);
    desc.add_options()("extra,e", po::value<unsigned int>(&extra)->default_value(1), extra_msg);
    desc.add_options()("bench", po::bool_switch(&bench)->default_value(false), bench_msg);

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

    chainPlanning(output, space, planners, links, obstacles, extra, c_opt, a_opt, bench);

    return 0;
}
