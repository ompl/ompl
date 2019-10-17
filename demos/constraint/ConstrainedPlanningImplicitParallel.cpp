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

#include <utility>

#include "ConstrainedPlanningCommon.h"

class ParallelBase
{
public:
    virtual void getStart(Eigen::VectorXd &x) = 0;
    virtual void getGoal(Eigen::VectorXd &x) = 0;
};

class ParallelChain : public ob::Constraint, public ParallelBase
{
public:
    ParallelChain(const unsigned int n, Eigen::Vector3d offset, unsigned int links, unsigned int chainNum,
                  double length = 1)
      : ob::Constraint(n, links), offset_(std::move(offset)), links_(links), chainNum_(chainNum), length_(length)
    {
        if (links % 2 == 0)
            throw ompl::Exception("Number of links must be odd!");
    }

    void getStart(Eigen::VectorXd &x) override
    {
        const double angle = boost::math::constants::pi<double>() / 16;
        const unsigned int offset = 3 * links_ * chainNum_;
        const Eigen::VectorXd axis =
            Eigen::AngleAxisd(boost::math::constants::pi<double>() / 2, Eigen::Vector3d::UnitZ()) * offset_;

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

    void getGoal(Eigen::VectorXd &x) override
    {
        unsigned int offset = 3 * links_ * chainNum_;

        if (links_ == 7)
        {
            Eigen::VectorXd nstep = offset_ * length_;
            Eigen::VectorXd estep =
                Eigen::AngleAxisd(boost::math::constants::pi<double>() / 2, Eigen::Vector3d::UnitZ()) * offset_ *
                length_;
            Eigen::VectorXd sstep =
                Eigen::AngleAxisd(boost::math::constants::pi<double>(), Eigen::Vector3d::UnitZ()) * offset_ * length_;
            Eigen::VectorXd wstep =
                Eigen::AngleAxisd(3 * boost::math::constants::pi<double>() / 2, Eigen::Vector3d::UnitZ()) * offset_ *
                length_;

            Eigen::VectorXd joint = offset_ + nstep;
            x.segment(3 * 0 + offset, 3) = joint;
            x.segment(3 * 1 + offset, 3) = x.segment(3 * 0 + offset, 3) + estep;
            x.segment(3 * 2 + offset, 3) = x.segment(3 * 1 + offset, 3) + estep;
            x.segment(3 * 3 + offset, 3) = x.segment(3 * 2 + offset, 3) + Eigen::Vector3d::UnitZ() * length_;
            x.segment(3 * 4 + offset, 3) = x.segment(3 * 3 + offset, 3) + sstep;
            x.segment(3 * 5 + offset, 3) = x.segment(3 * 4 + offset, 3) + sstep;
            x.segment(3 * 6 + offset, 3) = x.segment(3 * 5 + offset, 3) + wstep;
        }
        else
        {
            Eigen::VectorXd step = offset_ * length_;
            Eigen::VectorXd joint = offset_ + step;

            unsigned int i = 0;
            for (; i < links_ / 2; ++i, joint += step)
                x.segment(3 * i + offset, 3) = joint;

            joint += Eigen::Vector3d::UnitZ() * length_ - step;
            for (; i < links_; ++i, joint -= step)
                x.segment(3 * i + offset, 3) = joint;
        }
    }

    Eigen::Ref<const Eigen::VectorXd> getLink(const Eigen::VectorXd &x, const unsigned int idx) const
    {
        const unsigned int offset = 3 * links_ * chainNum_;
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
        const unsigned int offset = 3 * links_ * chainNum_;
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
    const Eigen::Vector3d offset_;
    const unsigned int links_;
    const unsigned int chainNum_;
    const double length_;
};

class ParallelPlatform : public ob::Constraint, public ParallelBase
{
public:
    ParallelPlatform(unsigned int links, unsigned int chains, double radius = 1)
      : ob::Constraint(3 * links * chains, chains), links_(links), chains_(chains), radius_(radius)
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

        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        for (unsigned int i = 0; i < chains_; ++i)
            centroid += getTip(x, i);
        centroid /= chains_;

        for (unsigned int i = 0; i < chains_; ++i)
            out[idx++] = (centroid - getTip(x, i)).norm() - radius_;

        for (unsigned int i = 0; i < chains_ - 3; ++i)
        {
            const Eigen::Vector3d ab = getTip(x, i + 1) - getTip(x, i);
            const Eigen::Vector3d ac = getTip(x, i + 2) - getTip(x, i);
            const Eigen::Vector3d ad = getTip(x, i + 3) - getTip(x, i);

            out[idx++] = ad.dot(ab.cross(ac));
        }
    }

    void getStart(Eigen::VectorXd &) override
    {
    }

    void getGoal(Eigen::VectorXd &) override
    {
    }

private:
    const unsigned int links_;
    const unsigned int chains_;
    const double radius_;
};

class ParallelConstraint : public ob::ConstraintIntersection, public ParallelBase
{
public:
    ParallelConstraint(unsigned int links, unsigned int chains, double radius = 1, double length = 1,
                       double jointRadius = 0.2)
      : ob::ConstraintIntersection(3 * links * chains, {})
      , links_(links)
      , chains_(chains)
      , radius_(radius)
      , length_(length)
      , jointRadius_(jointRadius)
    {
        Eigen::Vector3d offset = Eigen::Vector3d::UnitX();
        for (unsigned int i = 0; i < chains_; ++i)
        {
            addConstraint(std::make_shared<ParallelChain>(chains * links * 3, offset, links, i, length));
            offset =
                Eigen::AngleAxisd(2 * boost::math::constants::pi<double>() / (double)chains, Eigen::Vector3d::UnitZ()) *
                offset;
        }

        addConstraint(std::make_shared<ParallelPlatform>(links, chains, radius));
    }

    void getStart(Eigen::VectorXd &x) override
    {
        x = Eigen::VectorXd(3 * links_ * chains_);
        for (auto &constraint : constraints_)
            std::dynamic_pointer_cast<ParallelBase>(constraint)->getStart(x);
    }

    void getGoal(Eigen::VectorXd &x) override
    {
        x = Eigen::VectorXd(3 * links_ * chains_);
        for (auto &constraint : constraints_)
            std::dynamic_pointer_cast<ParallelBase>(constraint)->getGoal(x);
    }

    ob::StateSpacePtr createSpace() const
    {
        auto rvss = std::make_shared<ob::RealVectorStateSpace>(3 * links_ * chains_);
        ob::RealVectorBounds bounds(3 * links_ * chains_);

        for (unsigned int c = 0; c < chains_; ++c)
        {
            const unsigned int o = 3 * c * links_;
            for (int i = 0; i < (int)links_; ++i)
            {
                bounds.setLow(o + 3 * i + 0, -i - 2);
                bounds.setHigh(o + 3 * i + 0, i + 2);

                bounds.setLow(o + 3 * i + 1, -i - 2);
                bounds.setHigh(o + 3 * i + 1, i + 2);

                bounds.setLow(o + 3 * i + 2, -i - 2);
                bounds.setHigh(o + 3 * i + 2, i + 2);
            }
        }

        rvss->setBounds(bounds);
        return rvss;
    }

    bool isValid(const ob::State *state)
    {
        auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

        for (unsigned int i = 0; i < links_ * chains_; ++i)
        {
            if (x.segment(3 * i, 3)[2] < 0)
                return false;
        }

        for (unsigned int i = 0; i < links_ * chains_ - 1; ++i)
        {
            if (x.segment(3 * i, 3).cwiseAbs().maxCoeff() < jointRadius_)
                return false;

            for (unsigned int j = i + 1; j < links_ * chains_; ++j)
                if ((x.segment(3 * i, 3) - x.segment(3 * j, 3)).cwiseAbs().maxCoeff() < jointRadius_)
                    return false;
        }

        return true;
    }

    /** Create a projection evaluator for the parallel constraint. Finds the
     * centroid of the platform and project it to a one-dimensional space. */
    ob::ProjectionEvaluatorPtr getProjection(ob::StateSpacePtr space) const
    {
        class ParallelProjection : public ob::ProjectionEvaluator
        {
        public:
            ParallelProjection(const ob::StateSpacePtr &space, unsigned int links, unsigned int chains)
              : ob::ProjectionEvaluator(space), chains_(chains), links_(links)
            {
            }

            unsigned int getDimension() const override
            {
                return 1;
            }

            void defaultCellSizes() override
            {
                cellSizes_.resize(1);
                cellSizes_[0] = 0.1;
            }

            void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
            {
                auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

                for (unsigned int i = 0; i < chains_; ++i)
                    projection(0) = x[3 * (i + 1) * links_ - 1];

                projection(0) /= chains_;
            }

        private:
            const unsigned int chains_;
            const unsigned int links_;
        };

        return std::make_shared<ParallelProjection>(space, links_, chains_);
    }

    void dump(std::ofstream &file) const
    {
        file << links_ << std::endl;
        file << chains_ << std::endl;
        file << jointRadius_ << std::endl;
        file << length_ << std::endl;
        file << radius_ << std::endl;
    }

    void addBenchmarkParameters(ot::Benchmark *bench) const
    {
        bench->addExperimentParameter("links", "INTEGER", std::to_string(links_));
        bench->addExperimentParameter("chains", "INTEGER", std::to_string(chains_));
    }

private:
    const unsigned int links_;
    const unsigned int chains_;
    const double radius_;
    const double length_;
    const double jointRadius_;
};

bool parallelPlanningOnce(ConstrainedProblem &cp, enum PLANNER_TYPE planner, bool output)
{
    cp.setPlanner(planner, "parallel");

    // Solve the problem
    ob::PlannerStatus stat = cp.solveOnce(output, "parallel");

    if (output)
    {
        OMPL_INFORM("Dumping problem information to `parallel_info.txt`.");
        std::ofstream infofile("parallel_info.txt");
        infofile << cp.type << std::endl;
        dynamic_cast<ParallelConstraint *>(cp.constraint.get())->dump(infofile);
        infofile.close();
    }

    cp.atlasStats();

    return stat;
}

bool parallelPlanningBench(ConstrainedProblem &cp, std::vector<enum PLANNER_TYPE> &planners)
{
    cp.setupBenchmark(planners, "parallel");

    auto parallel = dynamic_cast<ParallelConstraint *>(cp.constraint.get());
    parallel->addBenchmarkParameters(cp.bench);

    cp.runBenchmark();

    return false;
}

bool parallelPlanning(bool output, enum SPACE_TYPE space, std::vector<enum PLANNER_TYPE> &planners, unsigned int links,
                      unsigned int chains, struct ConstrainedOptions &c_opt, struct AtlasOptions &a_opt, bool bench)
{
    // Create a shared pointer to our constraint.
    auto constraint = std::make_shared<ParallelConstraint>(links, chains);

    ConstrainedProblem cp(space, constraint->createSpace(), constraint);
    cp.setConstrainedOptions(c_opt);
    cp.setAtlasOptions(a_opt);

    cp.css->registerProjection("parallel", constraint->getProjection(cp.css));

    Eigen::VectorXd start, goal;
    constraint->getStart(start);
    constraint->getGoal(goal);

    cp.setStartAndGoalStates(start, goal);
    cp.ss->setStateValidityChecker(std::bind(&ParallelConstraint::isValid, constraint, std::placeholders::_1));

    if (!bench)
        return parallelPlanningOnce(cp, planners[0], output);
    else
        return parallelPlanningBench(cp, planners);
}

auto help_msg = "Shows this help message.";
auto output_msg = "Dump found solution path (if one exists) in plain text to `parallel_path.txt`. "
                  "Problem information is dumped to `parallel_info`.txt";
auto links_msg = "Number of links in each kinematic chain. Minimum is 3. Must be odd.";
auto chains_msg = "Number of chains in parallel mechanism. Minimum is 2.";
auto bench_msg = "Do benchmarking on provided planner list.";

int main(int argc, char **argv)
{
    bool output, bench;
    enum SPACE_TYPE space = PJ;
    std::vector<enum PLANNER_TYPE> planners = {RRT};

    unsigned int links = 3;
    unsigned int chains = 4;

    struct ConstrainedOptions c_opt;
    struct AtlasOptions a_opt;

    po::options_description desc("Options");
    desc.add_options()("help,h", help_msg);
    desc.add_options()("output,o", po::bool_switch(&output)->default_value(false), output_msg);
    desc.add_options()("links,l", po::value<unsigned int>(&links)->default_value(3), links_msg);
    desc.add_options()("chains,c", po::value<unsigned int>(&chains)->default_value(4), chains_msg);
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

    parallelPlanning(output, space, planners, links, chains, c_opt, a_opt, bench);

    return 0;
}
