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

#include <fstream>
#include <thread>

#include <ompl/base/ScopedState.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/AtlasChart.h>
#include <ompl/base/spaces/AtlasStateSpace.h>
#include <ompl/base/spaces/ProjectedStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>

/** Simple manifold example: the unit sphere. */
class EmptyConstraint : public ompl::base::Constraint
{
public:
    EmptyConstraint(const unsigned int n) : ompl::base::Constraint(n, n)
    {
    }

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        out.setZero();
    }

    void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out.setZero();
    }
};

/** Simple manifold example: the unit sphere. */
class SphereConstraint : public ompl::base::Constraint
{
public:
    SphereConstraint() : ompl::base::Constraint(3, 2)
    {
    }

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        out[0] = x.norm() - 1;
    }

    void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out = x.transpose().normalized();
    }
};

/** Simple manifold example: the xy plane. */
class PlaneConstraint : public ompl::base::Constraint
{
public:
    PlaneConstraint() : ompl::base::Constraint(3, 2)
    {
    }

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        out[0] = x[2];
    }

    void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out(0, 0) = 0;
        out(0, 1) = 0;
        out(0, 2) = 1;
    }
};

/** Torus manifold. */
class TorusConstraint : public ompl::base::Constraint
{
public:
    const double R1;
    const double R2;

    TorusConstraint(const double r1, const double r2) : ompl::base::Constraint(3, 2), R1(r1), R2(r2)
    {
    }

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        Eigen::VectorXd c(3);
        c << x[0], x[1], 0;
        out[0] = (x - R1 * c.normalized()).norm() - R2;
    }

    void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        const double xySquaredNorm = x[0] * x[0] + x[1] * x[1];
        const double xyNorm = std::sqrt(xySquaredNorm);
        const double denom = std::sqrt(x[2] * x[2] + (xyNorm - R1) * (xyNorm - R1));
        const double c = (xyNorm - R1) * (xyNorm * xySquaredNorm) / (xySquaredNorm * xySquaredNorm * denom);
        out(0, 0) = x[0] * c;
        out(0, 1) = x[1] * c;
        out(0, 2) = x[2] / denom;
    }
};

class KleinConstraint : public ompl::base::Constraint
{
public:
    KleinConstraint() : ompl::base::Constraint(3, 2)
    {
    }

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        const double p = x.squaredNorm() + 2 * x[1] - 1;
        const double n = x.squaredNorm() - 2 * x[1] - 1;
        const double u = n * n - 8 * x[2] * x[2];

        out[0] = p * u + 16 * x[0] * x[1] * n;
    }

    void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        const double p = x.squaredNorm() + 2 * x[1] - 1;
        const double n = x.squaredNorm() - 2 * x[1] - 1;
        const double u = n * n - 8 * x[2] * x[2];

        out(0, 0) = 32 * x[0] * x[0] * x[1] + 16 * x[1] * n + 4 * x[0] * n * p + 2 * x[0] * u;
        out(0, 1) = 32 * x[0] * x[1] * (x[1] - 1) + 16 * x[0] * n + 4 * (x[1] - 1) * n * p + 2 * (x[1] + 1) * u;
        out(0, 2) = 2 * x[2] * (16 * x[0] * x[1] + 2 * p * (n - 4) + u);
    }
};

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

        Eigen::VectorXd step = offset_ * length_;
        Eigen::VectorXd joint = offset_ + step;

        unsigned int i = 0;
        for (; i < links_ / 2; ++i, joint += step)
            x.segment(3 * i + offset, 3) = joint;

        joint += Eigen::Vector3d::UnitZ() * length_ - step;
        for (; i < links_; ++i, joint -= step)
            x.segment(3 * i + offset, 3) = joint;
    }

    Eigen::Ref<const Eigen::VectorXd> getLink(const Eigen::VectorXd &x, const unsigned int idx) const
    {
        const unsigned int offset = 3 * links_ * id_;
        return x.segment(offset + 3 * idx, 3);
    }

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
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

    void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
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

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
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

/** Kinematic chain manifold. */
class ChainConstraint : public ompl::base::Constraint
{
public:
    ChainConstraint(unsigned int links)
      : ompl::base::Constraint(3 * links, 2 * links - 1)
      , links_(links)
      , length_(1.0)
      , radius_((links - 4) + 2)
      , jointSize_(0.2)
    {
    }

    void function(const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        // Consecutive joints must be a fixed distance apart.
        Eigen::VectorXd joint1 = Eigen::VectorXd::Zero(3);
        for (unsigned int i = 0; i < links_; i++)
        {
            const Eigen::VectorXd joint2 = x.segment(3 * i, 3);
            out[i] = (joint1 - joint2).norm() - length_;
            joint1 = joint2;
        }

        out[links_] = x.tail(3).norm() - radius_;
    }

    void jacobian(const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
    {
        out.setZero();
        Eigen::VectorXd plus(3 * (links_ + 1));
        plus.head(3 * links_) = x;
        plus.tail(3) = Eigen::VectorXd::Zero(3);

        Eigen::VectorXd minus(3 * (links_ + 1));
        minus.head(3) = Eigen::VectorXd::Zero(3);
        minus.tail(3 * links_) = x;

        const Eigen::VectorXd diagonal = plus - minus;

        for (unsigned int i = 0; i < links_; i++)
            out.row(i).segment(3 * i, 3) = diagonal.segment(3 * i, 3).normalized();

        out.block(1, 0, links_, 3 * (links_ - 1)) -= out.block(1, 3, links_, 3 * (links_ - 1));

        out.row(links_).tail(3) = -diagonal.tail(3).normalized().transpose();
    }

    /** Joints may not get touch each other. If \a tough == true, then the end
    * effector may not occupy states on the sphere similar to the sphereValid()
    * obstacles. */
    bool isValid(double sleep, const ompl::base::State *state)
    {
        ompl::time::point wait = ompl::time::now() + ompl::time::seconds(sleep);
        while (ompl::time::now() < wait)
        {
        };

        Eigen::Ref<const Eigen::VectorXd> x =
            state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView();

        for (unsigned int i = 0; i < links_; i++)
        {
            if (x.segment(3 * i, 3)[2] < 0)
                return false;
        }

        for (unsigned int i = 0; i < links_ - 1; i++)
        {
            if (x.segment(3 * i, 3).cwiseAbs().maxCoeff() < jointSize_)
                return false;

            for (unsigned int j = i + 1; j < links_; j++)
            {
                if ((x.segment(3 * i, 3) - x.segment(3 * j, 3)).cwiseAbs().maxCoeff() < jointSize_)
                    return false;
            }
        }

        return true;
    }

private:
    const unsigned int links_;  // Number of chain links.
    const double length_;       // Length of one link.
    const double radius_;       // Radius of the sphere that the end effector is constrained to.
    const double jointSize_;    // Size of joints
};

class SphereProjection : public ompl::base::ProjectionEvaluator
{
public:
    SphereProjection(ompl::base::StateSpacePtr space) : ompl::base::ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension(void) const
    {
        return 2;
    }

    void defaultCellSizes(void)
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
    {
        Eigen::Ref<const Eigen::VectorXd> x =
            state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView();

        projection(0) = atan2(x[1], x[0]);
        projection(1) = acos(x[2]);
    }
};

class ChainProjection : public ompl::base::ProjectionEvaluator
{
public:
    ChainProjection(ompl::base::StateSpacePtr space, unsigned int dim, unsigned int links)
      : ompl::base::ProjectionEvaluator(space), dim_(dim), links_(links), radius_((links - 4) + 2)
    {
    }

    unsigned int getDimension(void) const
    {
        return 2;
    }

    void defaultCellSizes(void)
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.1;
        cellSizes_[1] = 0.1;
    }

    void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
    {
        Eigen::Ref<const Eigen::VectorXd> x =
            state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView();

        unsigned int s = dim_ * (links_ - 1);

        projection(0) = atan2(x[s + 1], x[s]);
        projection(1) = acos(x[s + 2] / radius_);
    }

private:
    const unsigned int dim_;    // Workspace dimension.
    const unsigned int links_;  // Number of chain links.
    double radius_;
};

class StewartProjection : public ompl::base::ProjectionEvaluator
{
public:
    StewartProjection(ompl::base::StateSpacePtr space, unsigned int links, unsigned int chains)
      : ompl::base::ProjectionEvaluator(space), chains_(chains), links_(links)
    {
    }

    unsigned int getDimension(void) const
    {
        return 1;
    }

    void defaultCellSizes(void)
    {
        cellSizes_.resize(1);
        cellSizes_[0] = 0.1;
        // cellSizes_[1] = 0.1;
        // cellSizes_[2] = 0.1;
    }

    void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const
    {
        Eigen::Ref<const Eigen::VectorXd> x =
            state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView();

        for (unsigned int i = 0; i < chains_; ++i)
            projection(0) = x[3 * (i + 1) * links_ - 1];

        projection(0) /= chains_;
    }

private:
    const unsigned int chains_;  // Workspace dimension.
    const unsigned int links_;   // Number of chain links.
};

/**
 * State validity checking functions implicitly define the free space where they return true.
 */

/** 3 ring-shaped obstacles on latitudinal lines, with a small gap in each. */
bool sphereValid_helper(const Eigen::Ref<const Eigen::VectorXd> x)
{
    if (-0.75 < x[2] && x[2] < -0.60)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] > 0;
        return false;
    }
    else if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    else if (0.60 < x[2] && x[2] < 0.75)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] < 0;
        return false;
    }
    return true;
}

/** Correct path on the sphere must snake around. */
bool sphereValid(double sleep, const ompl::base::State *state)
{
    std::this_thread::sleep_for(ompl::time::seconds(sleep));
    return sphereValid_helper(state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView());
}

/** Every state is valid. */
bool always(double sleep, const ompl::base::State *)
{
    std::this_thread::sleep_for(ompl::time::seconds(sleep));
    return true;
}

/** States surrounding the goal are invalid, making it unreachable. We can use this to build up an atlas
 * until time runs out, so we can see the big picture. */
bool unreachable(double sleep, const ompl::base::State *state, const Eigen::VectorXd &goal, const double radius)
{
    std::this_thread::sleep_for(ompl::time::seconds(sleep));
    return std::abs((state->as<ompl::base::ConstrainedStateSpace::StateType>()->constVectorView() - goal).norm() -
                    radius) > radius - 0.01;
}

/**
 * Problem initialization functions set the dimension, the manifold, start and goal points \a x and \a y,
 * and the validity checker \a isValid.
 */

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::Constraint *initPlaneSphereProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                               ompl::base::StateValidityCheckerFn &isValid,
                                               ompl::base::RealVectorBounds &bounds, double sleep)
{
    const std::size_t dim = 3;

    // Start and goal points
    x = Eigen::VectorXd(dim);
    x << 1, 0, 0;
    y = Eigen::VectorXd(dim);
    y << -1, 0, 0;

    // Validity checker
    isValid = std::bind(&always, sleep, std::placeholders::_1);

    // Atlas initialization (can use numerical methods to compute the Jacobian, but giving an explicit function is
    // faster)
    return new ompl::base::ConstraintIntersection(3, {new PlaneConstraint(), new SphereConstraint()});
}

ompl::base::Constraint *initEmptyProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                         ompl::base::StateValidityCheckerFn &isValid,
                                         ompl::base::RealVectorBounds &bounds, double sleep)
{
    const std::size_t dim = 3;

    x = Eigen::VectorXd(dim);
    x << 4, 4, 0;
    y = Eigen::VectorXd(dim);
    y << -4, -4, 0;

    isValid = std::bind(&always, sleep, std::placeholders::_1);

    bounds.resize(dim);
    bounds.setLow(-4);
    bounds.setLow(4);

    return new EmptyConstraint(3);
}

ompl::base::Constraint *initPlaneProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                         ompl::base::StateValidityCheckerFn &isValid,
                                         ompl::base::RealVectorBounds &bounds, double sleep)
{
    const std::size_t dim = 3;

    x = Eigen::VectorXd(dim);
    x << 4, 4, 0;
    y = Eigen::VectorXd(dim);
    y << -4, -4, 0;

    isValid = std::bind(&always, sleep, std::placeholders::_1);

    bounds.resize(dim);
    bounds.setLow(-4);
    bounds.setHigh(4);

    return new PlaneConstraint();
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::Constraint *initSphereProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                          ompl::base::StateValidityCheckerFn &isValid,
                                          ompl::base::RealVectorBounds &bounds, double sleep)
{
    const std::size_t dim = 3;

    // Start and goal points
    x = Eigen::VectorXd(dim);
    x << 0, 0, -1;
    y = Eigen::VectorXd(dim);
    y << 0, 0, 1;

    // Validity checker
    isValid = std::bind(&sphereValid, sleep, std::placeholders::_1);

    bounds.resize(dim);
    bounds.setLow(-1);
    bounds.setHigh(1);

    return new SphereConstraint();
}

/** Initialize the atlas for the torus problem and store the start and goal vectors. */
ompl::base::Constraint *initTorusProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                         ompl::base::StateValidityCheckerFn &isValid,
                                         ompl::base::RealVectorBounds &bounds, double sleep)
{
    const std::size_t dim = 3;

    // Start and goal points
    x = Eigen::VectorXd(dim);
    x << -3, 0, -1;
    y = Eigen::VectorXd(dim);
    y << 3, 0, 1;

    // Validity checker
    isValid = std::bind(&always, sleep, std::placeholders::_1);

    bounds.resize(dim);
    bounds.setLow(0, -4);
    bounds.setHigh(0, 4);
    bounds.setLow(1, -4);
    bounds.setHigh(1, 4);
    bounds.setLow(2, -2);
    bounds.setHigh(2, 2);

    return new TorusConstraint(3, 1);
}

/** Initialize the atlas for the sphere problem and store the start and goal vectors. */
ompl::base::Constraint *initKleinProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                         ompl::base::StateValidityCheckerFn &isValid,
                                         ompl::base::RealVectorBounds &bounds, double sleep)
{
    const std::size_t dim = 3;

    // Start and goal points
    x = Eigen::VectorXd(dim);
    x << -0.5, -0.25, 0.1892222244330081;
    y = Eigen::VectorXd(dim);
    y << 2.5, -1.5, 1.0221854181962458;

    // Validity checker
    isValid = std::bind(&always, sleep, std::placeholders::_1);

    bounds.resize(dim);
    bounds.setLow(-5);
    bounds.setHigh(5);

    return new KleinConstraint();
}

/** Initialize the atlas for the kinematic chain problem. */
ompl::base::Constraint *initChainProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                         ompl::base::StateValidityCheckerFn &isValid,
                                         ompl::base::RealVectorBounds &bounds, double sleep, int links = 20)
{
    const std::size_t dim = 3 * links;

    // Start and goal points (each triple is the 3D location of a joint)

    x = Eigen::VectorXd(dim);
    y = Eigen::VectorXd(dim);

    int i = 0;
    for (; i < links - 3; ++i)
    {
        x[3 * i] = i + 1;
        x[3 * i + 1] = 0;
        x[3 * i + 2] = 0;

        y[3 * i] = -(i + 1);
        y[3 * i + 1] = 0;
        y[3 * i + 2] = 0;
    }

    x[3 * i] = i;
    x[3 * i + 1] = -1;
    x[3 * i + 2] = 0;

    y[3 * i] = -i;
    y[3 * i + 1] = 1;
    y[3 * i + 2] = 0;

    i++;

    x[3 * i] = i;
    x[3 * i + 1] = -1;
    x[3 * i + 2] = 0;

    y[3 * i] = -i;
    y[3 * i + 1] = 1;
    y[3 * i + 2] = 0;

    i++;

    x[3 * i] = i - 1;
    x[3 * i + 1] = 0;
    x[3 * i + 2] = 0;

    y[3 * i] = -(i - 1);
    y[3 * i + 1] = 0;
    y[3 * i + 2] = 0;

    ChainConstraint *atlas = new ChainConstraint(links);
    isValid = std::bind(&ChainConstraint::isValid, atlas, sleep, std::placeholders::_1);

    bounds.resize(dim);
    for (unsigned int i = 0; i < links; ++i)
    {
        bounds.setLow(3 * i + 0, -i - 1);
        bounds.setHigh(3 * i + 0, i + 1);
        bounds.setLow(3 * i + 1, -i - 1);
        bounds.setHigh(3 * i + 1, i + 1);
        bounds.setLow(3 * i + 2, 0);
        bounds.setHigh(3 * i + 2, i + 1);
    }

    return atlas;
}

/** Initialize the atlas for the kinematic chain problem. */
ompl::base::Constraint *initStewartProblem(Eigen::VectorXd &x, Eigen::VectorXd &y,
                                           ompl::base::StateValidityCheckerFn &isValid,
                                           ompl::base::RealVectorBounds &bounds, double sleep, unsigned int links = 3,
                                           unsigned int chains = 4, unsigned int extra = 0)
{
    unsigned int dim = 3 * links * chains;
    x = Eigen::VectorXd(dim);
    y = Eigen::VectorXd(dim);

    StewartConstraint *constraint = new StewartConstraint(chains, links, extra);

    constraint->getStart(x);
    constraint->getGoal(y);

    isValid = std::bind(&StewartConstraint::isValid, constraint, std::placeholders::_1);

    bounds.resize(dim);
    for (unsigned int c = 0; c < chains; ++c)
    {
        const unsigned int o = 3 * c;
        for (unsigned int i = 0; i < links; ++i)
        {
            bounds.setLow(o + 3 * i + 0, -i - 2);
            bounds.setHigh(o + 3 * i + 0, i + 2);
            bounds.setLow(o + 3 * i + 1, -i - 2);
            bounds.setHigh(o + 3 * i + 1, i + 2);
            bounds.setLow(o + 3 * i + 2, 0);
            bounds.setHigh(o + 3 * i + 2, i + 1);
        }
    }

    return constraint;
}

/** Allocator function for a sampler for the atlas that only returns valid points. */
ompl::base::ValidStateSamplerPtr avssa(const ompl::base::SpaceInformation *si)
{
    return ompl::base::ValidStateSamplerPtr(new ompl::base::AtlasValidStateSampler(si));
}

/** Allocator function for a sampler for the atlas that only returns valid points. */
ompl::base::ValidStateSamplerPtr pvssa(const ompl::base::SpaceInformation *si)
{
    return ompl::base::ValidStateSamplerPtr(new ompl::base::ConstrainedValidStateSampler(si));
}

/** Print usage information. */
void printProblems(void)
{
    std::cout << "Available problems:\n";
    std::cout << "    empty plane sphere circle torus klein chain stewart\n";
}

/** Print usage information. */
void printPlanners(void)
{
    std::cout << "Available planners:\n";
    std::cout << "    EST RealEST BiRealEST SBL STRIDE\n";
    std::cout << "    RRT RRTintermediate RRTConnect RRTConnectIntermediate RRTstar LazyRRT TRRT\n";
    std::cout << "    LBTRRT KPIECE1 BKPIECE1 LBKPIECE1 PDST\n";
    std::cout << "    PRM PRMstar SPARS SPARStwo\n";
}

/** Initialize the problem specified in the string. */
ompl::base::Constraint *parseProblem(const char *const problem, Eigen::VectorXd &x, Eigen::VectorXd &y,
                                     ompl::base::StateValidityCheckerFn &isValid, ompl::base::RealVectorBounds &bounds,
                                     double sleep = 0, unsigned int links = 5, unsigned int chains = 2,
                                     unsigned int extra = 0)
{
    if (std::strcmp(problem, "plane") == 0)
        return initPlaneProblem(x, y, isValid, bounds, sleep);
    else if (std::strcmp(problem, "sphere") == 0)
        return initSphereProblem(x, y, isValid, bounds, sleep);
    else if (std::strcmp(problem, "circle") == 0)
        return initPlaneSphereProblem(x, y, isValid, bounds, sleep);
    else if (std::strcmp(problem, "torus") == 0)
        return initTorusProblem(x, y, isValid, bounds, sleep);
    else if (std::strcmp(problem, "klein") == 0)
        return initKleinProblem(x, y, isValid, bounds, sleep);
    else if (std::strcmp(problem, "chain") == 0)
        return initChainProblem(x, y, isValid, bounds, sleep, links);
    else if (std::strcmp(problem, "stewart") == 0)
        return initStewartProblem(x, y, isValid, bounds, sleep, links, chains, extra);
    if (std::strcmp(problem, "empty") == 0)
        return initEmptyProblem(x, y, isValid, bounds, sleep);
    else
        return NULL;
}

/** Initialize the planner specified in the string. */
ompl::base::Planner *parsePlanner(const char *const planner, const ompl::base::SpaceInformationPtr &si)
{
    if (std::strcmp(planner, "EST") == 0)
        return new ompl::geometric::EST(si);

    else if (std::strcmp(planner, "BiEST") == 0)
        return new ompl::geometric::BiEST(si);

    else if (std::strcmp(planner, "ProjEST") == 0)
        return new ompl::geometric::ProjEST(si);

    else if (std::strcmp(planner, "RRT") == 0)
        return new ompl::geometric::RRT(si);

    else if (std::strcmp(planner, "RRTintermediate") == 0)
        return new ompl::geometric::RRT(si, true);

    else if (std::strcmp(planner, "RRTConnect") == 0)
        return new ompl::geometric::RRTConnect(si);

    else if (std::strcmp(planner, "RRTConnectIntermediate") == 0)
        return new ompl::geometric::RRTConnect(si, true);

    else if (std::strcmp(planner, "RRTstar") == 0)
        return new ompl::geometric::RRTstar(si);

    else if (std::strcmp(planner, "LazyRRT") == 0)
        return new ompl::geometric::LazyRRT(si);

    else if (std::strcmp(planner, "TRRT") == 0)
        return new ompl::geometric::TRRT(si);

    else if (std::strcmp(planner, "LBTRRT") == 0)
        return new ompl::geometric::LBTRRT(si);

    else if (std::strcmp(planner, "KPIECE1") == 0)
        return new ompl::geometric::KPIECE1(si);

    else if (std::strcmp(planner, "BKPIECE1") == 0)
        return new ompl::geometric::BKPIECE1(si);

    else if (std::strcmp(planner, "LBKPIECE1") == 0)
        return new ompl::geometric::LBKPIECE1(si);

    else if (std::strcmp(planner, "PDST") == 0)
        return new ompl::geometric::PDST(si);

    else if (std::strcmp(planner, "PRM") == 0)
        return new ompl::geometric::PRM(si);

    else if (std::strcmp(planner, "PRMstar") == 0)
        return new ompl::geometric::PRMstar(si);

    else if (std::strcmp(planner, "SBL") == 0)
        return new ompl::geometric::SBL(si);

    else if (std::strcmp(planner, "SPARS") == 0)
        return new ompl::geometric::SPARS(si);

    else if (std::strcmp(planner, "SPARStwo") == 0)
        return new ompl::geometric::SPARStwo(si);

    else if (std::strcmp(planner, "STRIDE") == 0)
        return new ompl::geometric::STRIDE(si);

    else
        return NULL;
}
