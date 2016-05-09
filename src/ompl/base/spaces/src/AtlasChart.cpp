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

#include "ompl/base/spaces/AtlasChart.h"

#include <eigen3/Eigen/Dense>

/// AtlasChart::Halfspace

/// Public

ompl::base::AtlasChart::Halfspace::Halfspace (const AtlasChart &owner,
                                              const AtlasChart &neighbor)
    : owner_(owner)
{
    // Project neighbor's chart center onto our chart.
    Eigen::VectorXd u(owner_.k_);
    owner_.psiInverse(neighbor.getXorigin(), u);
    // Compute the halfspace equation, which is the perpendicular bisector
    // between 0 and u (plus 5% to reduce cracks, see Jaillet et al.).
    setU(1.05*u);
}

void ompl::base::AtlasChart::Halfspace::setComplement (Halfspace *complement)
{
    complement_ = complement;
}

ompl::base::AtlasChart::Halfspace *ompl::base::AtlasChart::Halfspace::getComplement (void) const
{
    return complement_;
}

const ompl::base::AtlasChart &ompl::base::AtlasChart::Halfspace::getOwner (void) const
{
    return owner_;
}

bool ompl::base::AtlasChart::Halfspace::contains (Eigen::Ref<const Eigen::VectorXd> v) const
{
    return v.dot(u_) <= rhs_;
}

void ompl::base::AtlasChart::Halfspace::checkNear (Eigen::Ref<const Eigen::VectorXd> v) const
{
    // Threshold is 10% of the distance from the boundary to the origin.
    if (distanceToPoint(v) < 1.0/20)
    {
        Eigen::VectorXd x(owner_.n_);
        owner_.psi(v,x);
        complement_->expandToInclude(x);
    }
}

bool ompl::base::AtlasChart::Halfspace::circleIntersect (
    const double r, Eigen::Ref<Eigen::VectorXd> v1, Eigen::Ref<Eigen::VectorXd> v2) const
{
    if (owner_.atlas_.getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::Halfspace::circleIntersect() only works on 2D manifolds.");

    // Check if there will be no solutions.
    double discr = 4*r*r - u_.squaredNorm();
    if (discr < 0)
        return false;
    discr = std::sqrt(discr);

    // Compute the 2 solutions (possibly 1 repeated solution).
    v1[0] = -u_[1] * discr;
    v1[1] = u_[0] * discr;
    v2 = -v1;
    v1 += u_ * u_.norm();
    v2 += u_ * u_.norm();
    v1 /= 2*u_.norm();
    v2 /= 2*u_.norm();
    
    return true;
}

/// Public static

void ompl::base::AtlasChart::Halfspace::intersect (
    const Halfspace &l1, const Halfspace &l2, Eigen::Ref<Eigen::VectorXd> out)
{
    if (&l1.owner_ != &l2.owner_)
        throw ompl::Exception("Cannot intersect linear inequalities on different charts.");
    if (l1.owner_.atlas_.getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::Halfspace::intersect() only works on 2D manifolds.");

    // Computer the intersection point of these lines.
    Eigen::MatrixXd A(2,2);
    A.row(0) = l1.u_.transpose(); A.row(1) = l2.u_.transpose();
    out[0] = l1.u_.squaredNorm(); out[1] = l2.u_.squaredNorm();
    out = 0.5 * A.inverse() * out;
}

/// Private

void ompl::base::AtlasChart::Halfspace::setU (const Eigen::VectorXd &u)
{
    u_ = u;
    // Precompute the right-hand side of the linear inequality.
    rhs_ = u_.squaredNorm()/2;
}

double ompl::base::AtlasChart::Halfspace::distanceToPoint (Eigen::Ref<const Eigen::VectorXd> v) const
{
    // Result is a scalar factor of u_.
    return (0.5 - v.dot(u_) / u_.squaredNorm());
}

void ompl::base::AtlasChart::Halfspace::expandToInclude (Eigen::Ref<const Eigen::VectorXd> x)
{
    // Compute how far v = psiInverse(x) lies past the boundary, if at all.
    Eigen::VectorXd v(owner_.k_);
    owner_.psiInverse(x, v);
    const double t = -distanceToPoint(v);
    
    // Move u_ further out by twice that much.
    if (t > 0)
        setU((1 + 2*t) * u_);
}

/// AtlasChart

/// Public

ompl::base::AtlasChart::AtlasChart (const AtlasStateSpace &atlas,
                                    Eigen::Ref<const Eigen::VectorXd> xorigin,
                                    const bool isAnchor)
: atlas_(atlas), n_(atlas_.getAmbientDimension()), k_(atlas_.getManifoldDimension()),
  xorigin_(xorigin), radius_(atlas_.getRho()), isAnchor_(isAnchor)
{
    // Decompose the Jacobian at xorigin.
    Eigen::MatrixXd j(n_-k_,n_);
    atlas_.jacobianFunction(xorigin_, j);
    Eigen::FullPivLU<Eigen::MatrixXd> decomp = j.fullPivLu();
    if (!decomp.isSurjective()) {
        OMPL_WARN("AtlasChart::AtlasChart(): Jacobian not surjective! Possible singularity?");
        // AtlasStateSpace will deal with this exception.
        throw ompl::Exception("Cannot compute full-rank tangent space.");
    }
    // Compute the null space, which is a basis for the tangent space.
    Eigen::HouseholderQR<Eigen::MatrixXd> nullDecomp = decomp.kernel().householderQr();
    // Orthonormalize.
    bigPhi_ = nullDecomp.householderQ() * Eigen::MatrixXd::Identity(n_, k_);
}

ompl::base::AtlasChart::~AtlasChart (void)
{
    clear();
}

void ompl::base::AtlasChart::clear (void)
{
    for (Halfspace *h : polytope_)
        delete h;
    polytope_.clear();
}

Eigen::Ref<const Eigen::VectorXd> ompl::base::AtlasChart::getXorigin (void) const
{
    return xorigin_;
}

const Eigen::VectorXd *ompl::base::AtlasChart::getXoriginPtr (void) const
{
    return &xorigin_;
}

void ompl::base::AtlasChart::phi (Eigen::Ref<const Eigen::VectorXd> u,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
    out = xorigin_ + bigPhi_ * u;
}

void ompl::base::AtlasChart::psi (Eigen::Ref<const Eigen::VectorXd> u,
                                  Eigen::Ref<Eigen::VectorXd> out) const
{
    // Initial guess for Newton's method
    Eigen::VectorXd x0(n_);
    phi(u, x0);
    psiFromAmbient(x0, out);
}

void ompl::base::AtlasChart::psiFromAmbient (Eigen::Ref<const Eigen::VectorXd> x0,
                                             Eigen::Ref<Eigen::VectorXd> out) const
{
    // Newton's method.
    out = x0;
    unsigned int iter = 0;
    // b holds info about constraint satisfaction and orthogonality of the
    // projection to the chart.
    Eigen::VectorXd b(n_);
    atlas_.constraintFunction(out, b.head(n_-k_));
    b.tail(k_).setZero();
    // A is the derivative used in Newton's method.
    Eigen::MatrixXd A(n_, n_);
    A.block(n_-k_, 0, k_, n_) = bigPhi_.transpose();    // This part is constant.
    while (b.norm() > atlas_.getProjectionTolerance() &&
           iter++ < atlas_.getProjectionMaxIterations())
    {
        // Recompute the Jacobian at the new guess.
        atlas_.jacobianFunction(out, A.block(0, 0, n_-k_, n_));
        
        // Move in the direction that decreases F(out) and is perpendicular to
        // the chart.
        out += A.householderQr().solve(-b);

        // Recompute b with new guess.
        atlas_.constraintFunction(out, b.head(n_-k_));
        b.tail(k_) = bigPhi_.transpose() * (out - x0);
    }
}

void ompl::base::AtlasChart::psiInverse (Eigen::Ref<const Eigen::VectorXd> x,
                                         Eigen::Ref<Eigen::VectorXd> out) const
{
    out = bigPhi_.transpose() * (x - xorigin_);
}

bool ompl::base::AtlasChart::inPolytope (Eigen::Ref<const Eigen::VectorXd> u,
                                         const Halfspace *const ignore1,
                                         const Halfspace *const ignore2) const
{
    for (Halfspace *h : polytope_)
    {
        if (h == ignore1 || h == ignore2)
            continue;
        if (!h->contains(u))
            return false;
    }
    return true;
}

void ompl::base::AtlasChart::borderCheck (Eigen::Ref<const Eigen::VectorXd> v) const
{
    for (Halfspace *h : polytope_)
        h->checkNear(v);
}

const ompl::base::AtlasChart *ompl::base::AtlasChart::owningNeighbor (
    Eigen::Ref<const Eigen::VectorXd> x) const
{
    Eigen::VectorXd projx(n_), proju(k_);
    for (Halfspace *h : polytope_)
    {
        // Project onto the neighboring chart.
        const AtlasChart &c = h->getComplement()->getOwner();
        c.psiInverse(x, proju);
        c.phi(proju, projx);
        // Check if it's within the validity region and polytope boundary.
        if ((projx - x).norm() < atlas_.getEpsilon() &&
            proju.norm() < atlas_.getRho() &&
            c.inPolytope(proju))
            return &c;
    }
    
    return nullptr;
}

void ompl::base::AtlasChart::setID (unsigned int id)
{
    id_ = id;
}

unsigned int ompl::base::AtlasChart::getID (void) const
{
    return id_;
}

bool ompl::base::AtlasChart::isAnchor (void) const
{
    return isAnchor_;
}

bool ompl::base::AtlasChart::toPolygon (std::vector<Eigen::VectorXd> &vertices) const
{
    if (atlas_.getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::toPolygon() only works on 2D manifold/charts.");
    
    // Compile a list of all the vertices in P and all the times the border
    // intersects the circle.
    Eigen::VectorXd v(2);
    Eigen::VectorXd intersection(n_);
    for (std::size_t i = 0; i < polytope_.size(); i++)
    {
        for (std::size_t j = i+1; j < polytope_.size(); j++)
        {
            // Check if intersection of the lines is a part of the boundary and
            // within the circle.
            Halfspace::intersect(*polytope_[i], *polytope_[j], v);
            phi(v, intersection);
            if (v.norm() <= radius_ && inPolytope(v, polytope_[i], polytope_[j]))
                vertices.push_back(intersection);
        }
        
        // Check if intersection with circle is part of the boundary.
        Eigen::VectorXd v1(2), v2(2);
        if ((polytope_[i])->circleIntersect(radius_, v1, v2))
        {
            if (inPolytope(v1, polytope_[i])) {
                phi(v1, intersection);
                vertices.push_back(intersection);
            }
            if (inPolytope(v2, polytope_[i])) {
                phi(v2, intersection);
                vertices.push_back(intersection);
            }
        }
    }
    
    // Include points approximating the circle, if they're inside the polytope.
    bool is_frontier = false;
    Eigen::VectorXd v0(2); v0 << radius_, 0;
    const double step = M_PI/16;
    for (double a = 0; a < 2*M_PI; a += step)
    {
        const Eigen::VectorXd v = Eigen::Rotation2Dd(a)*v0;
        
        if (inPolytope(v)) {
            is_frontier = true;
            phi(v, intersection);
            vertices.push_back(intersection);
        }
    }
    
    // Put all the points in order.
    std::sort(vertices.begin(), vertices.end(),
              [&] (Eigen::Ref<const Eigen::VectorXd> x1,
                   Eigen::Ref<const Eigen::VectorXd> x2) -> bool
              {
                  // Check the angles to see who should come first.
                  Eigen::VectorXd v1(2), v2(2);
                  psiInverse(x1, v1);
                  psiInverse(x2, v2);
                  return std::atan2(v1[1], v1[0]) < std::atan2(v2[1], v2[0]);
              });

    return is_frontier;
}

bool ompl::base::AtlasChart::estimateIsFrontier () const {
    RNG rng;
    Eigen::VectorXd ru(atlas_.getManifoldDimension());
    for (int k = 0; k < 1000; k++) {
        for (int i = 0; i < ru.size(); i++)
            ru[i] = rng.gaussian01();
        ru *= atlas_.getRho() / ru.norm();
        if (inPolytope(ru))
            return true;
    }
    return false;
}

/// Public Static

void ompl::base::AtlasChart::generateHalfspace (AtlasChart &c1, AtlasChart &c2)
{
    if (&c1.atlas_ != &c2.atlas_)
        throw ompl::Exception("generateHalfspace() must be called on charts in the same atlas!");
    
    // c1, c2 will delete l1, l2, respectively, upon destruction.
    Halfspace *l1, *l2;
    l1 = new Halfspace(c1, c2);
    l2 = new Halfspace(c2, c1);
    l1->setComplement(l2);
    l2->setComplement(l1);
    c1.addBoundary(l1);
    c2.addBoundary(l2);
}

/// Protected

void ompl::base::AtlasChart::addBoundary (Halfspace *halfspace)
{
    polytope_.push_back(halfspace);
}
