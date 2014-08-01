/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/// AtlasChart::LinearInequality

/// Public
ompl::base::AtlasChart::LinearInequality::LinearInequality (const AtlasChart &c, const AtlasChart &neighbor)
: owner_(c), complement_(NULL)
{
    // u_ should be neighbor's center projected onto our chart
    setU(1.05*owner_.psiInverse(neighbor.phi(Eigen::VectorXd::Zero(owner_.k_))));
}

ompl::base::AtlasChart::LinearInequality::LinearInequality (const AtlasChart &c, const Eigen::VectorXd &u)
: owner_(c), complement_(NULL)
{
    setU(u);
}

void ompl::base::AtlasChart::LinearInequality::setComplement (LinearInequality *const complement)
{
    complement_ = complement;
}

ompl::base::AtlasChart::LinearInequality *ompl::base::AtlasChart::LinearInequality::getComplement (void) const
{
    return complement_;
}

const ompl::base::AtlasChart &ompl::base::AtlasChart::LinearInequality::getOwner (void) const
{
    return owner_;
}

bool ompl::base::AtlasChart::LinearInequality::accepts (const Eigen::VectorXd &v) const
{
    return v.dot(u_) <= rhs_;
}

void ompl::base::AtlasChart::LinearInequality::checkNear (const Eigen::VectorXd &v) const
{
    // Threshold is 10% of the distance from the origin to the inequality
    if (complement_ && distanceToPoint(v) < 1.0/20)
        complement_->expandToInclude(owner_.psi(v));
}

bool ompl::base::AtlasChart::LinearInequality::circleIntersect (const double r, Eigen::VectorXd &v1, Eigen::VectorXd &v2) const
{
    if (owner_.atlas_.getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::LinearInequality::circleIntersect() only works on 2D manifolds.");
    
    const double discr = 4*r*r - u_.squaredNorm();
    if (discr < 0)
        return false;
    
    Eigen::VectorXd uRev(2); uRev << -u_[1], u_[0];
    v1 = uRev * std::sqrt(discr);
    v2 = -v1;
    v1 += u_ * u_.norm();
    v2 += u_ * u_.norm();
    v1 /= 2*u_.norm();
    v2 /= 2*u_.norm();
    
    return true;
}

/// Public static
Eigen::VectorXd ompl::base::AtlasChart::LinearInequality::intersect (const LinearInequality &l1, const LinearInequality &l2)
{
    if (&l1.owner_ != &l2.owner_)
        throw ompl::Exception("Cannot intersect linear inequalities on different charts.");
    if (l1.owner_.atlas_.getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::LinearInequality::intersect() only works on 2D manifolds.");
    
    Eigen::MatrixXd A(2,2);
    A.row(0) = l1.u_.transpose(); A.row(1) = l2.u_.transpose();
    Eigen::VectorXd b(2); b << l1.u_.squaredNorm(), l2.u_.squaredNorm();
    return 0.5 * A.inverse() * b;
}

/// Private
void ompl::base::AtlasChart::LinearInequality::setU (const Eigen::VectorXd &u)
{
    u_ = u;
    rhs_ = u_.squaredNorm()/2;
}

double ompl::base::AtlasChart::LinearInequality::distanceToPoint (const Eigen::VectorXd &v) const
{
    return (0.5 - v.dot(u_) / u_.squaredNorm());
}

void ompl::base::AtlasChart::LinearInequality::expandToInclude (const Eigen::VectorXd &x)
{
    // Compute how far v = psiInverse(x) lies outside the inequality, if at all
    const double t = -distanceToPoint(owner_.psiInverse(x));
    
    // Move u_ away by twice that much
    if (t > 0)
        setU((1 + 2*t) * u_);
}

/// AtlasChart

/// Public
ompl::base::AtlasChart::AtlasChart (const AtlasStateSpace &atlas, const Eigen::VectorXd &xorigin, const bool anchor)
: atlas_(atlas), n_(atlas_.getAmbientDimension()), k_(atlas_.getManifoldDimension()),
  xorigin_(xorigin), id_(atlas_.getChartCount()), anchor_(anchor), radius_(atlas_.getRho())
{
    if (atlas_.bigF(xorigin_).norm() > 10*atlas_.getProjectionTolerance())
        OMPL_DEBUG("AtlasChart created at point not on the manifold!");
    
    // Initialize basis by computing the null space of the Jacobian and orthonormalizing
    Eigen::FullPivLU<Eigen::MatrixXd> decomp = atlas_.bigJ(xorigin_).fullPivLu();
    Eigen::HouseholderQR<Eigen::MatrixXd> nullDecomp = decomp.kernel().householderQr();
    bigPhi_ = nullDecomp.householderQ() * Eigen::MatrixXd::Identity(n_, k_);
    bigPhi_t_ = bigPhi_.transpose();
    
    // Initialize set of linear inequalities so the polytope is the k-dimensional cube of side
    //  length 2*rho so it completely contains the ball of radius rho
    Eigen::VectorXd e = Eigen::VectorXd::Zero(k_);
    for (unsigned int i = 0; i < k_; i++)
    {
        e[i] = 2 * atlas_.getRho();
        bigL_.push_front(new LinearInequality(*this, e));
        e[i] *= -1;
        bigL_.push_front(new LinearInequality(*this, e));
        e[i] = 0;
    }
    measure_ = atlas_.getMeasureKBall() * std::pow(radius_, k_);
}

ompl::base::AtlasChart::~AtlasChart (void)
{
    for (std::list<LinearInequality *>::iterator l = bigL_.begin(); l != bigL_.end(); l++)
        delete *l;
}

Eigen::VectorXd ompl::base::AtlasChart::phi (const Eigen::VectorXd &u) const
{
    return xorigin_ + bigPhi_ * u;
}

Eigen::VectorXd ompl::base::AtlasChart::psi (const Eigen::VectorXd &u) const
{
    // Initial guess for Newton's method
    const Eigen::VectorXd x_0 = phi(u);
    Eigen::VectorXd x = x_0;
    
    unsigned int iter = 0;
    Eigen::VectorXd b(n_);
    b.head(n_-k_) = -atlas_.bigF(x);
    b.tail(k_) = Eigen::VectorXd::Zero(k_);
    while (b.norm() > atlas_.getProjectionTolerance() && iter++ < atlas_.getProjectionMaxIterations())
    {
        Eigen::MatrixXd A(n_, n_);
        A.block(0, 0, n_-k_, n_) = atlas_.bigJ(x);
        A.block(n_-k_, 0, k_, n_) = bigPhi_t_;
        
        // Move in the direction that decreases F(x) and is perpendicular to the chart plane
        x += A.colPivHouseholderQr().solve(b);
        
        b.head(n_-k_) = -atlas_.bigF(x);
        b.tail(k_) = bigPhi_t_ * (x_0 - x);
    }
    return x;
}

Eigen::VectorXd ompl::base::AtlasChart::psiInverse (const Eigen::VectorXd &x) const
{
    return bigPhi_t_ * (x - xorigin_);
}

bool ompl::base::AtlasChart::inP (const Eigen::VectorXd &u, const LinearInequality *const ignore1,
                                  const LinearInequality *const ignore2) const
{
    std::list<LinearInequality *>::const_iterator b, e;
    {
        boost::lock_guard<boost::mutex> lock(mutices_.bigL_);
        b = bigL_.begin();
        e = bigL_.end();
    }
    for (std::list<LinearInequality *>::const_iterator l = b; l != e; l++)
    {
        if (*l == ignore1 || *l == ignore2)
            continue;
        
        if (!(*l)->accepts(u))
            return false;
    }
    
    return true;
}

void ompl::base::AtlasChart::borderCheck (const Eigen::VectorXd &v) const
{
    std::list<LinearInequality *>::const_iterator b, e;
    {
        boost::lock_guard<boost::mutex> lock(mutices_.bigL_);
        b = bigL_.begin();
        e = bigL_.end();
    }
    for (std::list<LinearInequality *>::const_iterator l = b; l != e; l++)
        (*l)->checkNear(v);
}

void ompl::base::AtlasChart::own (ompl::base::AtlasStateSpace::StateType *const state) const
{
    boost::lock_guard<boost::mutex> lock(mutices_.owned_);
    assert(state != NULL);
    owned_.push_front(state);
}

void ompl::base::AtlasChart::disown (ompl::base::AtlasStateSpace::StateType *const state) const
{
    boost::lock_guard<boost::mutex> lock(mutices_.owned_);
    for (std::list<ompl::base::AtlasStateSpace::StateType *>::iterator s = owned_.begin(); s != owned_.end(); s++)
    {
        if (*s == state)
        {
            owned_.erase(s);
            break;
        }
    }
}

void ompl::base::AtlasChart::substituteChart (const AtlasChart &replacement) const
{
    boost::lock_guard<boost::mutex> lock(mutices_.owned_);
    while (owned_.size() != 0)
    {
        owned_.front()->setChart(replacement, true);
        owned_.pop_front();
    }
}

const ompl::base::AtlasChart *ompl::base::AtlasChart::owningNeighbor (const Eigen::VectorXd &x) const
{
    std::list<LinearInequality *>::const_iterator b, e;
    {
        boost::lock_guard<boost::mutex> lock(mutices_.bigL_);
        b = bigL_.begin();
        e = bigL_.end();
    }
    const AtlasChart *bestC = NULL;
    double best = std::numeric_limits<double>::infinity();
    for (std::list<LinearInequality *>::const_iterator l = b; l != e; l++)
    {
        const LinearInequality *const comp = (*l)->getComplement();
        if (!comp)
            continue;
        
        // Project onto the chart and check if it's in the validity region and polytope
        const AtlasChart &c = comp->getOwner();
        const Eigen::VectorXd psiInvX = c.psiInverse(x);
        const Eigen::VectorXd psiPsiInvX = c.psi(psiInvX);
        if ((c.phi(psiInvX) - psiPsiInvX).norm() < atlas_.getEpsilon() && psiInvX.norm() < radius_ && c.inP(psiInvX))
        {
            // The closer the point to where the chart puts it, the better
            double err = (psiPsiInvX - x).norm();
            if (err < best)
            {
                bestC = &c;
                best = err;
            }
        }
    }
    
    return bestC;
}

void ompl::base::AtlasChart::approximateMeasure (void) const
{
    // Perform Monte Carlo integration to estimate measure
    unsigned int countInside = 0;
    const std::vector<Eigen::VectorXd> &samples = atlas_.getMonteCarloSamples();
    for (std::size_t i = 0; i < samples.size(); i++)
    {
        // Take a sample and check if it's inside P \intersect k-Ball
        if (inP(samples[i]*radius_, NULL))
            countInside++;
    }
    
    // Update measure with new estimate. If 0 samples, then pretend we are just a sphere.
    measure_ = atlas_.getMeasureKBall() * std::pow(radius_, k_);
    if (samples.size() > 0)
        measure_ = countInside * (measure_ / samples.size());
    atlas_.updateMeasure(*this);
}

double ompl::base::AtlasChart::getMeasure (void) const
{
    return measure_;
}

void ompl::base::AtlasChart::shrinkRadius (void) const
{
    if (radius_ > atlas_.getDelta())
        radius_ *= 0.8;
}
        
unsigned int ompl::base::AtlasChart::getID (void) const
{
    return id_;
}

bool ompl::base::AtlasChart::isAnchor (void) const
{
    return anchor_;
}

void ompl::base::AtlasChart::toPolygon (std::vector<Eigen::VectorXd> &vertices) const
{
    std::list<LinearInequality *>::const_iterator b, e;
    {
        boost::lock_guard<boost::mutex> lock(mutices_.bigL_);
        b = bigL_.begin();
        e = bigL_.end();
    }
    if (atlas_.getManifoldDimension() != 2)
        throw ompl::Exception("AtlasChart::toPolygon() only works on 2D manifold/charts.");
    
    // Compile a list of all the vertices in P and all the times the border intersects the circle
    vertices.clear();
    for (std::list<LinearInequality *>::const_iterator l1 = b; l1 != e; l1++)
    {
        for (std::list<LinearInequality *>::const_iterator l2 = boost::next(l1); l2 != e; l2++)
        {
            // Check if intersection of the lines is a part of the boundary and within the circle
            Eigen::VectorXd v = LinearInequality::intersect(**l1, **l2);
            if (v.norm() <= radius_ && inP(v, *l1, *l2))
                vertices.push_back(phi(v));
        }
        
        // Check if intersection with circle is part of the boundary
        Eigen::VectorXd v1, v2;
        if ((*l1)->circleIntersect(radius_, v1, v2))
        {
            if (inP(v1, *l1))
                vertices.push_back(phi(v1));
            if (inP(v2, *l1))
                vertices.push_back(phi(v2));
        }
    }
    
    // Throw in points approximating the circle, if they're inside P
    Eigen::VectorXd v0(2); v0 << radius_, 0;
    const double step = M_PI/16;
    for (double a = 0; a < 2*M_PI; a += step)
    {
        const Eigen::VectorXd v = Eigen::Rotation2Dd(a)*v0;
        if (inP(v))
            vertices.push_back(phi(v));
    }
    
    // Put them in order
    std::sort(vertices.begin(), vertices.end(), boost::bind(&AtlasChart::angleCompare, this, _1, _2));
}

/// Public Static
void ompl::base::AtlasChart::generateHalfspace (AtlasChart &c1, AtlasChart &c2)
{
    if (&c1.atlas_ != &c2.atlas_)
        throw ompl::Exception("generateHalfspace() must be called on charts in the same atlas!");
    
    // c1, c2 will delete l1, l2, respectively, upon destruction
    LinearInequality *l1, *l2;
    l1 = new LinearInequality(c1, c2);
    l2 = new LinearInequality(c2, c1);
    l1->setComplement(l2);
    l2->setComplement(l1);
    c1.addBoundary(*l1);
    c2.addBoundary(*l2);
}

/// Protected
void ompl::base::AtlasChart::addBoundary (LinearInequality &halfspace) const
{
    {
        boost::lock_guard<boost::mutex> lock(mutices_.bigL_);
        bigL_.push_front(&halfspace);
    }
    
    // Update the measure estimate
    approximateMeasure();
    
    // Find tracked states which need to be moved to a different chart
    boost::lock_guard<boost::mutex> lock(mutices_.owned_);
    const bool fast = true;
    for (std::list<ompl::base::AtlasStateSpace::StateType *>::iterator s = owned_.begin(); s != owned_.end(); s++)
    {
        assert(*s != NULL);
        if (!halfspace.accepts(psiInverse((*s)->toVector())))
        {
            const LinearInequality *const comp = halfspace.getComplement();
            assert(comp);
            (*s)->setChart(comp->getOwner(), fast);
            
            // Manually disown here because it's faster since we already have the iterator
            s = boost::prior(owned_.erase(s));
        }
    }
}

// Private
bool ompl::base::AtlasChart::angleCompare (const Eigen::VectorXd &x1, const Eigen::VectorXd &x2) const
{
    const Eigen::VectorXd v1 = psiInverse(x1);
    const Eigen::VectorXd v2 = psiInverse(x2);
    return std::atan2(v1[1], v1[0]) < std::atan2(v2[1], v2[0]);
}
