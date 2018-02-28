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

/* Author: Caleb Voss */

#include "ompl/base/spaces/constraint/AtlasChart.h"
#include <boost/math/constants/constants.hpp>
#include <eigen3/Eigen/Dense>

/// AtlasChart::Halfspace

/// Public

ompl::base::AtlasChart::Halfspace::Halfspace(const AtlasChart *owner, const AtlasChart *neighbor) : owner_(owner)
{
    // Project neighbor's chart center onto our chart.
    Eigen::VectorXd u(owner_->k_);
    owner_->psiInverse(*neighbor->getOrigin(), u);

    // Compute the halfspace equation, which is the perpendicular bisector
    // between 0 and u (plus 5% to reduce cracks, see Jaillet et al.).
    setU(1.05 * u);
}

bool ompl::base::AtlasChart::Halfspace::contains(const Eigen::Ref<const Eigen::VectorXd> &v) const
{
    return v.dot(u_) <= rhs_;
}

void ompl::base::AtlasChart::Halfspace::checkNear(const Eigen::Ref<const Eigen::VectorXd> &v) const
{
    // Threshold is 10% of the distance from the boundary to the origin.
    if (distanceToPoint(v) < 1.0 / 20)
    {
        Eigen::VectorXd x(owner_->n_);
        owner_->psi(v, x);
        complement_->expandToInclude(x);
    }
}

bool ompl::base::AtlasChart::Halfspace::circleIntersect(const double r, Eigen::Ref<Eigen::VectorXd> v1,
                                                        Eigen::Ref<Eigen::VectorXd> v2) const
{
    if (owner_->getManifoldDimension() != 2)
        throw ompl::Exception("ompl::base::AtlasChart::Halfspace::circleIntersect() "
                              "Only works on 2D manifolds.");

    // Check if there will be no solutions.
    double discr = 4 * r * r - usqnorm_;
    if (discr < 0)
        return false;
    discr = std::sqrt(discr);

    // Compute the 2 solutions (possibly 1 repeated solution).
    double unorm = std::sqrt(usqnorm_);
    v1[0] = -u_[1] * discr;
    v1[1] = u_[0] * discr;
    v2 = -v1;
    v1 += u_ * unorm;
    v2 += u_ * unorm;
    v1 /= 2 * unorm;
    v2 /= 2 * unorm;

    return true;
}

/// Public static

void ompl::base::AtlasChart::Halfspace::intersect(const Halfspace &l1, const Halfspace &l2,
                                                  Eigen::Ref<Eigen::VectorXd> out)
{
    if (l1.owner_ != l2.owner_)
        throw ompl::Exception("Cannot intersect linear inequalities on different charts.");
    if (l1.owner_->getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::Halfspace::intersect() only works on 2D manifolds.");

    // Computer the intersection point of these lines.
    Eigen::MatrixXd A(2, 2);
    A.row(0) = l1.u_.transpose();
    A.row(1) = l2.u_.transpose();
    out[0] = l1.u_.squaredNorm();
    out[1] = l2.u_.squaredNorm();
    out = 0.5 * A.inverse() * out;
}

/// Private

void ompl::base::AtlasChart::Halfspace::setU(const Eigen::Ref<const Eigen::VectorXd> &u)
{
    u_ = u;

    // Precompute the squared norm of u.
    usqnorm_ = u_.squaredNorm();

    // Precompute the right-hand side of the linear inequality.
    rhs_ = usqnorm_ / 2;
}

double ompl::base::AtlasChart::Halfspace::distanceToPoint(const Eigen::Ref<const Eigen::VectorXd> &v) const
{
    // Result is a scalar factor of u_.
    return (0.5 - v.dot(u_)) / usqnorm_;
}

void ompl::base::AtlasChart::Halfspace::expandToInclude(const Eigen::Ref<const Eigen::VectorXd> &x)
{
    // Compute how far v = psiInverse(x) lies past the boundary, if at all.
    Eigen::VectorXd v(owner_->k_);
    owner_->psiInverse(x, v);
    const double t = -distanceToPoint(v);

    // Move u_ further out by twice that much.
    if (t > 0)
        setU((1 + 2 * t) * u_);
}

/// AtlasChart

/// Public

ompl::base::AtlasChart::AtlasChart(const AtlasStateSpace *atlas, const AtlasStateSpace::StateType *state)
  : constraint_(atlas->getConstraint().get())
  , n_(atlas->getAmbientDimension())
  , k_(atlas->getManifoldDimension())
  , state_(state)
  , bigPhi_([&]() -> const Eigen::MatrixXd {
      Eigen::MatrixXd j(n_ - k_, n_);
      constraint_->jacobian(*state_, j);

      Eigen::FullPivLU<Eigen::MatrixXd> decomp = j.fullPivLu();
      if (!decomp.isSurjective())
          throw ompl::Exception("Cannot compute full-rank tangent space.");

      // Compute the null space and orthonormalize, which is a basis for the tangent space.
      return decomp.kernel().householderQr().householderQ() * Eigen::MatrixXd::Identity(n_, k_);
  }())
  , radius_(atlas->getRho_s())
{
}

ompl::base::AtlasChart::~AtlasChart()
{
    clear();
}

void ompl::base::AtlasChart::clear()
{
    for (auto h : polytope_)
        delete h;

    polytope_.clear();
}

void ompl::base::AtlasChart::phi(const Eigen::Ref<const Eigen::VectorXd> &u, Eigen::Ref<Eigen::VectorXd> out) const
{
    out = *state_ + bigPhi_ * u;
}

bool ompl::base::AtlasChart::psi(const Eigen::Ref<const Eigen::VectorXd> &u, Eigen::Ref<Eigen::VectorXd> out) const
{
    // Initial guess for Newton's method
    Eigen::VectorXd x0(n_);
    phi(u, x0);

    // Newton-Raphson to solve Ax = b
    unsigned int iter = 0;
    double norm = 0;
    Eigen::MatrixXd A(n_, n_);
    Eigen::VectorXd b(n_);

    const double tolerance = constraint_->getTolerance();
    const double squaredTolerance = tolerance * tolerance;

    // Initialize output to initial guess
    out = x0;

    // Initialize A with orthonormal basis (constant)
    A.block(n_ - k_, 0, k_, n_) = bigPhi_.transpose();

    // Initialize b with initial f(out) = b
    constraint_->function(out, b.head(n_ - k_));
    b.tail(k_).setZero();

    while ((norm = b.squaredNorm()) > squaredTolerance && iter++ < constraint_->getMaxIterations())
    {
        // Recompute the Jacobian at the new guess.
        constraint_->jacobian(out, A.block(0, 0, n_ - k_, n_));

        // Move in the direction that decreases F(out) and is perpendicular to
        // the chart.
        out -= A.partialPivLu().solve(b);

        // Recompute b with new guess.
        constraint_->function(out, b.head(n_ - k_));
        b.tail(k_) = bigPhi_.transpose() * (out - x0);
    }

    return norm < squaredTolerance;
}

void ompl::base::AtlasChart::psiInverse(const Eigen::Ref<const Eigen::VectorXd> &x,
                                        Eigen::Ref<Eigen::VectorXd> out) const
{
    out = bigPhi_.transpose() * (x - *state_);
}

bool ompl::base::AtlasChart::inPolytope(const Eigen::Ref<const Eigen::VectorXd> &u, const Halfspace *const ignore1,
                                        const Halfspace *const ignore2) const
{
    if (u.norm() > radius_)
        return false;

    for (Halfspace *h : polytope_)
    {
        if (h == ignore1 || h == ignore2)
            continue;

        if (!h->contains(u))
            return false;
    }

    return true;
}

void ompl::base::AtlasChart::borderCheck(const Eigen::Ref<const Eigen::VectorXd> &v) const
{
    for (Halfspace *h : polytope_)
        h->checkNear(v);
}

const ompl::base::AtlasChart *ompl::base::AtlasChart::owningNeighbor(const Eigen::Ref<const Eigen::VectorXd> &x) const
{
    Eigen::VectorXd projx(n_), proju(k_);
    for (Halfspace *h : polytope_)
    {
        // Project onto the neighboring chart.
        const AtlasChart *c = h->getComplement()->getOwner();
        c->psiInverse(x, proju);
        c->phi(proju, projx);

        // Check if it's within the validity region and polytope boundary.
        const bool withinTolerance = (projx - x).norm();
        const bool inPolytope = c->inPolytope(proju);

        if (withinTolerance && inPolytope)
            return c;
    }

    return nullptr;
}

bool ompl::base::AtlasChart::toPolygon(std::vector<Eigen::VectorXd> &vertices) const
{
    if (k_ != 2)
        throw ompl::Exception("AtlasChart::toPolygon() only works on 2D manifold/charts.");

    // Compile a list of all the vertices in P and all the times the border
    // intersects the circle.
    Eigen::VectorXd v(2);
    Eigen::VectorXd intersection(n_);
    vertices.clear();
    for (std::size_t i = 0; i < polytope_.size(); i++)
    {
        for (std::size_t j = i + 1; j < polytope_.size(); j++)
        {
            // Check if intersection of the lines is a part of the boundary and
            // within the circle.
            Halfspace::intersect(*polytope_[i], *polytope_[j], v);
            phi(v, intersection);
            if (inPolytope(v, polytope_[i], polytope_[j]))
                vertices.push_back(intersection);
        }

        // Check if intersection with circle is part of the boundary.
        Eigen::VectorXd v1(2), v2(2);
        if ((polytope_[i])->circleIntersect(radius_, v1, v2))
        {
            if (inPolytope(v1, polytope_[i]))
            {
                phi(v1, intersection);
                vertices.push_back(intersection);
            }
            if (inPolytope(v2, polytope_[i]))
            {
                phi(v2, intersection);
                vertices.push_back(intersection);
            }
        }
    }

    // Include points approximating the circle, if they're inside the polytope.
    bool is_frontier = false;
    Eigen::VectorXd v0(2);
    v0 << radius_, 0;
    const double step = boost::math::constants::pi<double>() / 32.;
    for (double a = 0.; a < 2. * boost::math::constants::pi<double>(); a += step)
    {
        const Eigen::VectorXd vn = Eigen::Rotation2Dd(a) * v0;

        if (inPolytope(vn))
        {
            is_frontier = true;
            phi(vn, intersection);
            vertices.push_back(intersection);
        }
    }

    // Put all the points in order.
    std::sort(vertices.begin(), vertices.end(),
              [&](const Eigen::Ref<const Eigen::VectorXd> &x1, const Eigen::Ref<const Eigen::VectorXd> &x2) -> bool {
                  // Check the angles to see who should come first.
                  Eigen::VectorXd v1(2), v2(2);
                  psiInverse(x1, v1);
                  psiInverse(x2, v2);
                  return std::atan2(v1[1], v1[0]) < std::atan2(v2[1], v2[0]);
              });

    return is_frontier;
}

bool ompl::base::AtlasChart::estimateIsFrontier() const
{
    RNG rng;
    Eigen::VectorXd ru(k_);
    for (int k = 0; k < 1000; k++)
    {
        for (int i = 0; i < ru.size(); i++)
            ru[i] = rng.gaussian01();
        ru *= radius_ / ru.norm();
        if (inPolytope(ru))
            return true;
    }
    return false;
}

/// Public Static

void ompl::base::AtlasChart::generateHalfspace(AtlasChart *c1, AtlasChart *c2)
{
    if (c1 == c2)
        throw ompl::Exception("ompl::base::AtlasChart::generateHalfspace(): "
                              "Must use two different charts.");

    // c1, c2 will delete l1, l2, respectively, upon destruction.
    Halfspace *l1, *l2;
    l1 = new Halfspace(c1, c2);
    l2 = new Halfspace(c2, c1);
    l1->setComplement(l2);
    l2->setComplement(l1);
    c1->addBoundary(l1);
    c2->addBoundary(l2);
}

/// Protected

void ompl::base::AtlasChart::addBoundary(Halfspace *halfspace)
{
    polytope_.push_back(halfspace);
}
