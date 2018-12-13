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

class ChainConstraint : public ob::Constraint
{
private:
    /**
     * Container class for the "wall" obstacles that are on the surface of the
     * sphere constraint (when extra = 1). */
    class Wall
    {
    public:
        Wall(double offset, double thickness, double width, double joint_radius, unsigned int type)
          : offset_(offset), thickness_(thickness + joint_radius), width_(width + joint_radius), type_(type)
        {
        }

        /**
         * Checks if an x coordinate places the sphere within the boundary of
         * the wall. */
        bool within(double x) const
        {
            return !(x < (offset_ - thickness_) || x > (offset_ + thickness_));
        }

        bool checkJoint(const Eigen::Ref<const Eigen::VectorXd> &v) const
        {
            double x = v[0], y = v[1], z = v[2];

            if (!within(x))
                return true;

            if (z <= width_)
            {
                switch (type_)
                {
                    case 0:
                        if (y < 0)
                            return true;
                        break;

                    case 1:
                        if (y > 0)
                            return true;
                        break;
                }
            }

            return false;
        }

    private:
        const double offset_;
        const double thickness_;
        const double width_;
        const unsigned int type_;
    };

    const double WALL_WIDTH = 0.5;
    const double JOINT_RADIUS = 0.2;
    const double LINK_LENGTH = 1.0;

public:
    /**
     * An implicit kinematic chain, formed out of balls each in R^3, with
     * distance constraints between successive balls creating spherical joint
     * kinematics for the system.
     *
     * Extra constraints are as follows:
     * 1 - End-effector is constrained to be on the surface of a sphere of
     *     radius links - 2
     * 2 - The (links - 5)th and (links - 4)th ball have the same z-value
     * 3 - The (links - 4)th and (links - 3)th ball have the same x-value
     * 4 - The (links - 3)th and (links - 2)th ball have the same z-value
     */
    ChainConstraint(unsigned int links, unsigned int obstacles = 0, unsigned int extra = 1)
      : ob::Constraint(3 * links, links + extra)
      , links_(links)
      , length_(LINK_LENGTH)
      , width_(WALL_WIDTH)
      , radius_(links - 2)
      , jointRadius_(JOINT_RADIUS)
      , obstacles_(obstacles)
      , extra_(extra)
    {
        double step = 2 * radius_ / (double)(obstacles_ + 1);
        double current = -radius_ + step;

        for (unsigned int i = 0; i < obstacles_; i++, current += step)
            walls_.emplace_back(current, radius_ / 8, WALL_WIDTH, JOINT_RADIUS, i % 2);
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        Eigen::VectorXd joint1 = Eigen::VectorXd::Zero(3);
        for (unsigned int i = 0; i < links_; i++)
        {
            auto &&joint2 = x.segment(3 * i, 3);
            out[i] = (joint1 - joint2).norm() - length_;
            joint1 = joint2;
        }

        if (extra_ >= 1)
            out[links_] = x.tail(3).norm() - radius_;

        const unsigned int o = links_ - 5;

        if (extra_ >= 2)
            out[links_ + 1] = x[(o + 0) * 3 + 2] - x[(o + 1) * 3 + 2];
        if (extra_ >= 3)
            out[links_ + 2] = x[(o + 1) * 3 + 0] - x[(o + 2) * 3 + 0];
        if (extra_ >= 4)
            out[links_ + 3] = x[(o + 2) * 3 + 2] - x[(o + 3) * 3 + 2];
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out.setZero();

        Eigen::VectorXd plus(3 * (links_ + 1));
        plus.head(3 * links_) = x.segment(0, 3 * links_);
        plus.tail(3) = Eigen::VectorXd::Zero(3);

        Eigen::VectorXd minus(3 * (links_ + 1));
        minus.head(3) = Eigen::VectorXd::Zero(3);
        minus.tail(3 * links_) = x.segment(0, 3 * links_);

        auto &&diagonal = plus - minus;

        for (unsigned int i = 0; i < links_; i++)
            out.row(i).segment(3 * i + 0, 3) = diagonal.segment(3 * i, 3).normalized();

        out.block(1, 0, links_ - 1, 3 * links_ - 3) -= out.block(1, 3, links_ - 1, 3 * links_ - 3);

        if (extra_ >= 1)
            out.row(links_).tail(3) = -diagonal.tail(3).normalized().transpose();

        const unsigned int o = links_ - 5;

        if (extra_ >= 2)
        {
            out(links_ + 1, (o + 0) * 3 + 2) = 1;
            out(links_ + 1, (o + 1) * 3 + 2) = -1;
        }
        if (extra_ >= 3)
        {
            out(links_ + 2, (o + 1) * 3 + 0) = 1;
            out(links_ + 2, (o + 2) * 3 + 0) = -1;
        }
        if (extra_ >= 4)
        {
            out(links_ + 3, (o + 2) * 3 + 2) = 1;
            out(links_ + 3, (o + 3) * 3 + 2) = -1;
        }
    }

    /**
     * Checks if there are no self-collisions (of the joints themselves) or
     * collisions with the extra obstacles on the surface of the sphere. */
    bool isValid(const ob::State *state)
    {
        auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();

        for (unsigned int i = 0; i < links_; i++)
        {
            auto &&joint = x.segment(3 * i, 3);
            if (joint[2] < 0)
                return false;

            if (joint.norm() >= (radius_ - jointRadius_))
                for (auto wall : walls_)
                    if (!wall.checkJoint(joint))
                        return false;
        }

        for (unsigned int i = 0; i < links_ - 1; i++)
        {
            auto &&joint1 = x.segment(3 * i, 3);
            if (joint1.cwiseAbs().maxCoeff() < jointRadius_)
                return false;

            for (unsigned int j = i + 1; j < links_; j++)
            {
                auto &&joint2 = x.segment(3 * j, 3);
                if ((joint1 - joint2).cwiseAbs().maxCoeff() < jointRadius_)
                    return false;
            }
        }

        return true;
    }

    ob::StateSpacePtr createSpace() const
    {
        auto rvss = std::make_shared<ob::RealVectorStateSpace>(3 * links_);
        ob::RealVectorBounds bounds(3 * links_);

        for (int i = 0; i < (int)links_; ++i)
        {
            bounds.setLow(3 * i + 0, -i - 1);
            bounds.setHigh(3 * i + 0, i + 1);

            bounds.setLow(3 * i + 1, -i - 1);
            bounds.setHigh(3 * i + 1, i + 1);

            bounds.setLow(3 * i + 2, -i - 1);
            bounds.setHigh(3 * i + 2, i + 1);
        }

        rvss->setBounds(bounds);
        return rvss;
    }

    void setStartAndGoalStates(Eigen::VectorXd &start, Eigen::VectorXd &goal) const
    {
        start = Eigen::VectorXd(3 * links_);
        goal = Eigen::VectorXd(3 * links_);

        int i = 0;
        for (; i < (int)links_ - 3; ++i)
        {
            start[3 * i] = i + 1;
            start[3 * i + 1] = 0;
            start[3 * i + 2] = 0;

            goal[3 * i] = -(i + 1);
            goal[3 * i + 1] = 0;
            goal[3 * i + 2] = 0;
        }

        start[3 * i] = i;
        start[3 * i + 1] = -1;
        start[3 * i + 2] = 0;

        goal[3 * i] = -i;
        goal[3 * i + 1] = 1;
        goal[3 * i + 2] = 0;

        i++;

        start[3 * i] = i;
        start[3 * i + 1] = -1;
        start[3 * i + 2] = 0;

        goal[3 * i] = -i;
        goal[3 * i + 1] = 1;
        goal[3 * i + 2] = 0;

        i++;

        start[3 * i] = i - 1;
        start[3 * i + 1] = 0;
        start[3 * i + 2] = 0;

        goal[3 * i] = -(i - 1);
        goal[3 * i + 1] = 0;
        goal[3 * i + 2] = 0;
    }

    /** Create a projection evaluator for the chain constraint. Finds the
     * spherical coordinates of the end-effector on the surface of the sphere of
     * radius equal to that of the constraint (when extra = 1). */
    ob::ProjectionEvaluatorPtr getProjection(ob::StateSpacePtr space) const
    {
        class ChainProjection : public ob::ProjectionEvaluator
        {
        public:
            ChainProjection(const ob::StateSpacePtr &space, unsigned int links, double radius)
              : ob::ProjectionEvaluator(space), links_(links), radius_(radius)
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

            void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
            {
                auto &&x = *state->as<ob::ConstrainedStateSpace::StateType>();
                const unsigned int s = 3 * (links_ - 1);

                projection(0) = atan2(x[s + 1], x[s]);
                projection(1) = acos(x[s + 2] / radius_);
            }

        private:
            const unsigned int links_;  // Number of chain links.
            double radius_;             // Radius of sphere end-effector lies on (for extra = 1)
        };

        return std::make_shared<ChainProjection>(space, links_, radius_);
    }

    void dump(std::ofstream &file) const
    {
        file << links_ << std::endl;
        file << obstacles_ << std::endl;
        file << extra_ << std::endl;
        file << jointRadius_ << std::endl;
        file << length_ << std::endl;
        file << radius_ << std::endl;
        file << width_ << std::endl;
    }

    void addBenchmarkParameters(ot::Benchmark *bench) const
    {
        bench->addExperimentParameter("links", "INTEGER", std::to_string(links_));
        bench->addExperimentParameter("obstacles", "INTEGER", std::to_string(obstacles_));
        bench->addExperimentParameter("extra", "INTEGER", std::to_string(extra_));
    }

private:
    const unsigned int links_;      // Number of chain links.
    const double length_;           // Length of one link.
    const double width_;            // Width of obstacle wall.
    const double radius_;           // Radius of the sphere that the end effector is constrained to.
    const double jointRadius_;      // Size of joints
    const unsigned int obstacles_;  // Number of obstacles on sphere surface
    const unsigned int extra_;      // Number of extra constraints
    std::vector<Wall> walls_;       // Obstacles
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

    return false;
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

    if (vm.count("help") != 0u)
    {
        std::cout << desc << std::endl;
        return 1;
    }

    chainPlanning(output, space, planners, links, obstacles, extra, c_opt, a_opt, bench);

    return 0;
}
