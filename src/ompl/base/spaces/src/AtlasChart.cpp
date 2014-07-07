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

#include <eigen3/Eigen/Dense>

/// AtlasChart::LinearInequality

/// Public
ompl::base::AtlasChart::LinearInequality::LinearInequality (const AtlasChart &c, const AtlasChart &neighbor)
: owner_(c), complement_(NULL)
{
    // u_ should be neighbor's center projected onto our chart
    setU(owner_.psiInverse(neighbor.phi(Eigen::VectorXd::Zero(owner_.k_))));
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
    // Equation (10) in the Jaillet paper
    return v.dot(u_) <= rhs_;
}

void ompl::base::AtlasChart::LinearInequality::checkNear (const Eigen::VectorXd &v) const
{
    // Threshold is 10% of the distance from the origin to the inequality
    if (complement_ && distanceToPoint(v) < 1.0/20)
        complement_->expandToInclude(owner_.psi(v));
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
ompl::base::AtlasChart::AtlasChart (const AtlasStateSpace &atlas, const Eigen::VectorXd &xorigin)
: atlas_(atlas), n_(atlas_.getAmbientDimension()), k_(atlas_.getManifoldDimension()),
  xorigin_(xorigin), id_(atlas_.getChartCount())
{
    if (atlas_.bigF(xorigin_).norm() > 10*atlas_.getProjectionTolerance())
        OMPL_WARN("AtlasChart created at point not on the manifold!");
    
    // Initialize basis by computing the null space of the Jacobian and orthonormalizing
    Eigen::MatrixXd nullJ = atlas_.bigJ(xorigin_).fullPivLu().kernel();
    bigPhi_ = nullJ.householderQr().householderQ() * Eigen::MatrixXd::Identity(n_, k_);
    bigPhi_t_ = bigPhi_.transpose();
    
    // Initialize set of linear inequalities so the polytope is the k_-dimensional cube of side
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
    measure_ = atlas_.getMeasureRhoKBall();
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
    
    unsigned iter = 0;
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

bool ompl::base::AtlasChart::inP (const Eigen::VectorXd &u, std::size_t *const solitary) const
{
    bool inPolytope = true;
    if (solitary)
        *solitary = bigL_.size();
    
    std::size_t i = 0;
    for (std::list<LinearInequality *>::const_iterator l = bigL_.begin(); l != bigL_.end(); l++, i++)
    {
        if (!(*l)->accepts(u))
        {
            // We can stop early if we're not interested in more information
            inPolytope = false;
            if (!solitary)
                break;
            if (*solitary != bigL_.size())
            {
                // This is the second violation; give up
                *solitary = bigL_.size();
                break;
            }
            
            // This could be the solitary violation
            *solitary = i;
        }
    }
    
    return inPolytope;
}

void ompl::base::AtlasChart::borderCheck (const Eigen::VectorXd &v) const
{
    for (std::list<LinearInequality *>::const_iterator l = bigL_.begin(); l != bigL_.end(); l++)
        (*l)->checkNear(v);
}

const ompl::base::AtlasChart *ompl::base::AtlasChart::owningNeighbor (const Eigen::VectorXd &x) const
{
    const AtlasChart *bestC = NULL;
    double best = std::numeric_limits<double>::infinity();
    for (std::list<LinearInequality *>::const_iterator l = bigL_.begin(); l != bigL_.end(); l++)
    {
        const LinearInequality *const comp = (*l)->getComplement();
        if (!comp)
            continue;
        
        // Project onto the chart and check if it's in the validity region and polytope
        const AtlasChart &c = comp->getOwner();
        const Eigen::VectorXd psiInvX = c.psiInverse(x);
        const Eigen::VectorXd psiPsiInvX = c.psi(psiInvX);
        if ((c.phi(psiInvX) - psiPsiInvX).norm() < atlas_.getEpsilon() && psiInvX.norm() < atlas_.getRho() && c.inP(psiInvX))
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

void ompl::base::AtlasChart::approximateMeasure (void)
{
    addBoundary();
}

double ompl::base::AtlasChart::getMeasure (void) const
{
    return measure_;
}

unsigned int ompl::base::AtlasChart::getID (void) const
{
    return id_;
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
    c1.addBoundary(l1);
    c2.addBoundary(l2);
    
    // TODO What about existing states that need to move to a different chart because of the boundary change?
    // It hasn't been an issue, but the paper calls for it.
}

/// Protected
void ompl::base::AtlasChart::addBoundary (LinearInequality *const halfspace)
{
    if (halfspace)
        bigL_.push_front(halfspace);
    
    // Initialize list of inequalities marked for pruning
    std::vector<bool> pruneCandidates(bigL_.size() + 1);  // dummy at the end for convenience
    for (std::size_t i = 0; i < bigL_.size(); i++)
        pruneCandidates[i] = true;
    
    // Perform Monte Carlo integration to estimate volume
    unsigned int countInside = 0;
    const std::vector<Eigen::VectorXd> &samples = atlas_.getMonteCarloSamples();
    for (std::size_t i = 0; i < samples.size(); i++)
    {
        // Take a sample and check if it's inside P \intersect k-Ball
        std::size_t soleViolation;
        if (inP(samples[i], &soleViolation))
            countInside++;
        
        // If there was a solitary violation, that inequalitiy is too important to prune
        pruneCandidates[soleViolation] = false;
    }
    
    // Prune at most one candidate (If two inequalities are too close together, we won't sample
    //  between them, so we don't know which is redundant and which is important.)
    std::size_t i = 0;
    for (std::list<LinearInequality *>::iterator l = bigL_.begin(); l != bigL_.end(); l++, i++)
    {
        if (pruneCandidates[i])
        {
            LinearInequality *const comp = (*l)->getComplement();
            if (comp)
                comp->setComplement(NULL);
            delete *l;
            bigL_.erase(l);
            break;
        }
    }
    
    // Update measure with new estimate
    measure_ = countInside * (atlas_.getMeasureRhoKBall() / samples.size());
    atlas_.updateMeasure(*this);
}
