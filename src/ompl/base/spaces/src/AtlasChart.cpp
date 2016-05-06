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

#include <boost/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/lock_guard.hpp>

#include <eigen3/Eigen/Dense>

/// AtlasChart::Halfspace

/// Public
ompl::base::AtlasChart::Halfspace::Halfspace (const AtlasChart &owner,
                                              const AtlasChart &neighbor)
    : owner_(owner), complement_(nullptr)
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
ompl::base::AtlasChart::AtlasChart (const AtlasStateSpace &atlas, Eigen::Ref<const Eigen::VectorXd> xorigin, const bool anchor)
: atlas_(atlas), n_(atlas_.getAmbientDimension()), k_(atlas_.getManifoldDimension()),
  xorigin_(xorigin), id_(0), anchor_(anchor), radius_(atlas_.getRho())
{
    Eigen::VectorXd f(n_-k_);
    
    // Initialize basis by computing the null space of the Jacobian and orthonormalizing
    Eigen::MatrixXd j(n_-k_,n_);
    atlas_.bigJ(xorigin_, j);
    Eigen::FullPivLU<Eigen::MatrixXd> decomp = j.fullPivLu();
    if (!decomp.isSurjective()) {
        OMPL_WARN("AtlasChart::AtlasChart(): Jacobian not surjective! Possible singularity?");
        throw ompl::Exception("Computing tangent basis here would cause floating-point exception.");
    }
    Eigen::HouseholderQR<Eigen::MatrixXd> nullDecomp = decomp.kernel().householderQr();
    bigPhi_ = nullDecomp.householderQ() * Eigen::MatrixXd::Identity(n_, k_);
    bigPhi_t_ = bigPhi_.transpose();
    
    init();
}

ompl::base::AtlasChart::~AtlasChart (void)
{
    for (std::size_t i = 0; i < bigL_.size(); i++)
        delete bigL_[i];
}

void ompl::base::AtlasChart::clear (void)
{
    for (std::size_t i = 0; i < bigL_.size(); i++)
        delete bigL_[i];
    bigL_.clear();
    
    radius_ = atlas_.getRho();
    
    init();
}

Eigen::Ref<const Eigen::VectorXd> ompl::base::AtlasChart::getXorigin (void) const
{
    return xorigin_;
}

const Eigen::VectorXd *ompl::base::AtlasChart::getXoriginPtr (void) const
{
    return &xorigin_;
}

void ompl::base::AtlasChart::phi (Eigen::Ref<const Eigen::VectorXd> u, Eigen::Ref<Eigen::VectorXd> out) const
{
    out = xorigin_ + bigPhi_ * u;
}

void ompl::base::AtlasChart::psiFromGuess (Eigen::Ref<const Eigen::VectorXd> x_0, Eigen::Ref<Eigen::VectorXd> out) const
{
    out = x_0;
    
    unsigned int iter = 0;
    Eigen::VectorXd b(n_);
    atlas_.bigF(out, b.head(n_-k_));
    b.tail(k_).setZero();
    Eigen::MatrixXd A(n_, n_);
    A.block(n_-k_, 0, k_, n_) = bigPhi_t_;
    while (b.norm() > atlas_.getProjectionTolerance() && iter++ < atlas_.getProjectionMaxIterations())
    {
        atlas_.bigJ(out, A.block(0, 0, n_-k_, n_));
        
        // Move in the direction that decreases F(out) and is perpendicular to the chart plane
        out += A.householderQr().solve(-b);
        
        atlas_.bigF(out, b.head(n_-k_));
        b.tail(k_) = bigPhi_t_ * (out - x_0);
    }
}

void ompl::base::AtlasChart::psi (Eigen::Ref<const Eigen::VectorXd> u, Eigen::Ref<Eigen::VectorXd> out) const
{
    // Initial guess for Newton's method
    Eigen::VectorXd x_0(n_);
    phi(u, x_0);
    psiFromGuess(x_0, out);
}

void ompl::base::AtlasChart::psiInverse (Eigen::Ref<const Eigen::VectorXd> x, Eigen::Ref<Eigen::VectorXd> out) const
{
    out = bigPhi_t_ * (x - xorigin_);
}

bool ompl::base::AtlasChart::inP (Eigen::Ref<const Eigen::VectorXd> u, const Halfspace *const ignore1,
                                  const Halfspace *const ignore2) const
{
    for (std::size_t i = 0; i < bigL_.size(); i++)
    {
        if (bigL_[i] == ignore1 || bigL_[i] == ignore2)
            continue;
        
        if (!bigL_[i]->contains(u))
            return false;
    }
    
    return true;
}

void ompl::base::AtlasChart::borderCheck (Eigen::Ref<const Eigen::VectorXd> v) const
{
    for (std::size_t i = 0; i < bigL_.size(); i++)
        bigL_[i]->checkNear(v);
}

const ompl::base::AtlasChart *ompl::base::AtlasChart::owningNeighbor (Eigen::Ref<const Eigen::VectorXd> x) const
{
    Eigen::VectorXd tempx(n_), tempu(k_);
    for (std::size_t i = 0; i < bigL_.size(); i++)
    {
        const Halfspace *comp = bigL_[i]->getComplement();
        
        // Project onto the chart and check if it's in the validity region and polytope
        const AtlasChart &c = comp->getOwner();
        c.psiInverse(x, tempu);
        c.phi(tempu, tempx);
        if ((tempx - x).norm() < atlas_.getEpsilon() && tempu.norm() < atlas_.getRho() && c.inP(tempu))
            return &c;
    }
    
    return nullptr;
}

void ompl::base::AtlasChart::approximateMeasure (void)
{
    /*
    // Perform Monte Carlo integration to estimate measure
    unsigned int countInside = 0;
    const std::vector<Eigen::VectorXd> &samples = atlas_.getMonteCarloSamples();
    for (std::size_t i = 0; i < samples.size(); i++)
    {
        // Take a sample and check if it's inside P \intersect k-Ball
        if (inP(samples[i]*radius_, nullptr))
            countInside++;
    }
    
    // Update measure with new estimate. If 0 samples, then pretend we are just a sphere.
    measure_ = atlas_.getMeasureKBall() * std::pow(radius_, k_);
    if (samples.size() > 0)
        measure_ = countInside * (measure_ / samples.size());
    atlas_.updateMeasure(*this);
    */
}

double ompl::base::AtlasChart::getMeasure (void) const
{
    return 1;
    //return measure_;
}

void ompl::base::AtlasChart::shrinkRadius (void) const
{
//     if (radius_ > atlas_.getDelta())
//         radius_ *= 0.8;
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
    return anchor_;
}

bool ompl::base::AtlasChart::toPolygon (std::vector<Eigen::VectorXd> &vertices) const
{
    if (atlas_.getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::toPolygon() only works on 2D manifold/charts.");
    
    // Compile a list of all the vertices in P and all the times the border intersects the circle
    vertices.clear();
    Eigen::VectorXd v(2);
    Eigen::VectorXd intersection(n_);
    for (std::size_t i = 0; i < bigL_.size(); i++)
    {
        for (std::size_t j = i+1; j < bigL_.size(); j++)
        {
            // Check if intersection of the lines is a part of the boundary and within the circle
            Halfspace::intersect(*bigL_[i], *bigL_[j], v);
            phi(v, intersection);
            if (v.norm() <= radius_ && inP(v, bigL_[i], bigL_[j]))
                vertices.push_back(intersection);
        }
        
        // Check if intersection with circle is part of the boundary
        Eigen::VectorXd v1(2), v2(2);
        if ((bigL_[i])->circleIntersect(radius_, v1, v2))
        {
            if (inP(v1, bigL_[i])) {
                phi(v1, intersection);
                vertices.push_back(intersection);
            }
            if (inP(v2, bigL_[i])) {
                phi(v2, intersection);
                vertices.push_back(intersection);
            }
        }
    }
    
    // Throw in points approximating the circle, if they're inside P
    bool is_frontier = false;
    Eigen::VectorXd v0(2); v0 << radius_, 0;
    const double step = M_PI/16;
    for (double a = 0; a < 2*M_PI; a += step)
    {
        const Eigen::VectorXd v = Eigen::Rotation2Dd(a)*v0;
        
        if (inP(v)) {
            is_frontier = true;
            phi(v, intersection);
            vertices.push_back(intersection);
        }
    }
    
    // Put them in order
    std::sort(vertices.begin(), vertices.end(), boost::bind(&AtlasChart::angleCompare, this, _1, _2));

    return is_frontier;
}

bool ompl::base::AtlasChart::estimateIsFrontier () const {
    Eigen::VectorXd ru(atlas_.getManifoldDimension());
    for (int k = 0; k < 1000; k++) {
        for (int i = 0; i < ru.size(); i++)
            ru[i] = rng_.gaussian01();
        ru *= atlas_.getRho() / ru.norm();
        if (inP(ru))
            return true;
    }
    return false;
}

/// Public Static
void ompl::base::AtlasChart::generateHalfspace (AtlasChart &c1, AtlasChart &c2)
{
    if (&c1.atlas_ != &c2.atlas_)
        throw ompl::Exception("generateHalfspace() must be called on charts in the same atlas!");
    
    // c1, c2 will delete l1, l2, respectively, upon destruction
    Halfspace *l1, *l2;
    l1 = new Halfspace(c1, c2);
    l2 = new Halfspace(c2, c1);
    l1->setComplement(l2);
    l2->setComplement(l1);
    c1.addBoundary(*l1);
    c2.addBoundary(*l2);
}

double ompl::base::AtlasChart::distanceBetweenCenters (AtlasChart *c1, AtlasChart *c2)
{
    return (c1->getXorigin() - c2->getXorigin()).norm();
}

/// Protected
void ompl::base::AtlasChart::addBoundary (Halfspace &halfspace)
{
    bigL_.push_back(&halfspace);
    
    // Update the measure estimate
    approximateMeasure();
}

/// Private
void ompl::base::AtlasChart::init (void)
{
    // Initialize set of linear inequalities so the polytope is the k-dimensional cube of side
    //  length 2*rho so it completely contains the ball of radius rho
    /*Eigen::VectorXd e = Eigen::VectorXd::Zero(k_);
    for (unsigned int i = 0; i < k_; i++)
    {
        e[i] = 2 * atlas_.getRho();
        bigL_.push_back(new Halfspace(*this, e));
        e[i] *= -1;
        bigL_.push_back(new Halfspace(*this, e));
        e[i] = 0;
    }
    */
    measure_ = 1;//atlas_.getMeasureKBall() * std::pow(radius_, k_);
}

bool ompl::base::AtlasChart::angleCompare (Eigen::Ref<const Eigen::VectorXd> x1, Eigen::Ref<const Eigen::VectorXd> x2) const
{
    Eigen::VectorXd v1(k_), v2(k_);
    psiInverse(x1, v1);
    psiInverse(x2, v2);
    return std::atan2(v1[1], v1[0]) < std::atan2(v2[1], v2[0]);
}
