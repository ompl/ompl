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

#include "ompl/base/spaces/AtlasStateSpace.h"

#include "ompl/base/PlannerDataGraph.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/AtlasChart.h"
#include "ompl/util/Exception.h"

#include <signal.h>

#include <boost/foreach.hpp>
#include <boost/math/special_functions/gamma.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/thread/lock_guard.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/// AtlasStateSampler

/// Public
ompl::base::AtlasStateSampler::AtlasStateSampler (const AtlasStateSpace &atlas)
: StateSampler(&atlas), atlas_(atlas)
{
}

void ompl::base::AtlasStateSampler::sampleUniform (State *state)
{
    Eigen::Ref<Eigen::VectorXd> rx = state->as<AtlasStateSpace::StateType>()->vectorView();
    Eigen::VectorXd ry(atlas_.getAmbientDimension());
    Eigen::VectorXd ru(atlas_.getManifoldDimension());
    AtlasChart *c;
    
    // Rejection sampling to find a point on the manifold
    Eigen::VectorXd f(atlas_.getAmbientDimension()-atlas_.getManifoldDimension());
    int tries = 0;
    do
    {
        // Rejection sampling to find a point inside a chart's polytope
        do
        {
            tries++;
            // Pick a chart.
            c = &atlas_.sampleChart();
            
            // Sample a point within rho_s of the center. This is done by sampling uniformly on the surface
            // and multiplying by a distance whose distribution is biased according to spherical volume
            for (int i = 0; i < ru.size(); i++)
                ru[i] = rng_.gaussian01();
            ru *= atlas_.getRho_s() * std::pow(rng_.uniform01(), 1.0/ru.size()) / ru.norm();
        }
        while (!c->inP(ru));
        
        c->psi(ru, rx);
    }
    while (rx.hasNaN() || (atlas_.bigF(rx, f), f.norm() > atlas_.getProjectionTolerance()));

    // Extend polytope of neighboring chart wherever point is near the border
    c->psiInverse(rx, ru);
    c->borderCheck(ru);
    state->as<AtlasStateSpace::StateType>()->setChart(atlas_.owningChart(rx));
}

void ompl::base::AtlasStateSampler::sampleUniformNear (State *state, const State *near, const double distance)
{
    // Find the chart that the starting point is on.
    AtlasStateSpace::StateType *astate = state->as<AtlasStateSpace::StateType>();
    const AtlasStateSpace::StateType *anear = near->as<AtlasStateSpace::StateType>();
    Eigen::Ref<const Eigen::VectorXd> n = anear->constVectorView();
    Eigen::VectorXd rx(atlas_.getAmbientDimension()), ru(atlas_.getManifoldDimension());
    AtlasChart *c = anear->getChart();
    if (!c)
    {
        c = atlas_.owningChart(n);
        if (!c)
        {
            // Chart creation can fail.
            try
            {
                c = &atlas_.newChart(n);
            }
            catch (ompl::Exception &e)
            {
                // This is really bad, because we need a chart. Default to a uniform sample.
                OMPL_WARN("AtlasStateSpace::sampleUniformNear(): Failed to sample because chart creation at state failed!");
                sampleUniform(state);
                return;
            }
        }
        anear->setChart(c);
    }
    
    // Rejection sampling to find a point that can be projected onto the manifold
    c->psiInverse(n, ru);
    int tries = 100;
    Eigen::VectorXd f(atlas_.getAmbientDimension()-atlas_.getManifoldDimension());
    do
    {
        tries--;
        // Sample within distance
        Eigen::VectorXd uoffset(atlas_.getManifoldDimension());
        for (int i = 0; i < uoffset.size(); i++)
            uoffset[i] = rng_.gaussian01();
        uoffset *=  distance * std::pow(rng_.uniform01(), 1.0/uoffset.size()) / uoffset.norm();

#define ORTHOPROJECT 0
#if ORTHOPROJECT // Option 1: project orthogonally using chart c
        c->psi(ru + uoffset, rx);
    }
    while (tries > 0 && (rx.hasNaN() || (atlas_.bigF(rx, f), f.norm() > atlas_.getProjectionTolerance())));
#else // Option 2: ordinary gradient descent
        c->phi(ru + uoffset, rx);
    }
    while (tries > 0 && !atlas_.project(rx));
#endif

    if (tries == 0)
    {
        OMPL_WARN("AtlasStateSpace::sampleUniformNear() got stuck. Returning initial point.");
        rx = n;
    }

    // Be lazy about determining the new chart if we are not in the old one
    if (c->psiInverse(rx, ru), !c->inP(ru))
        c = NULL;
    else
        c->borderCheck(ru);
    astate->setRealState(rx, c);
}

void ompl::base::AtlasStateSampler::sampleGaussian (State *state, const State *mean, const double stdDev)
{
    AtlasStateSpace::StateType *astate = state->as<AtlasStateSpace::StateType>();
    const AtlasStateSpace::StateType *amean = mean->as<AtlasStateSpace::StateType>();
    Eigen::Ref<const Eigen::VectorXd> m = amean->constVectorView();
    const std::size_t k = atlas_.getManifoldDimension();
    Eigen::VectorXd rx(atlas_.getAmbientDimension()), ru(k);
    AtlasChart *c = amean->getChart();
    if (!c)
    {
        c = atlas_.owningChart(m);
        if (!c)
        {
            // Chart creation can fail.
            try
            {
                c = &atlas_.newChart(m);
            }
            catch (ompl::Exception &e)
            {
                // This is really bad, because we need a chart. Default to a uniform sample.
                OMPL_WARN("AtlasStateSpace::sampleGaussian(): Failed to sample because chart creation at state failed!");
                sampleUniform(state);
                return;
            }
        }
        amean->setChart(c);
    }
    c->psiInverse(m, ru);
    
    // Rejection sampling to find a point on the manifold
    do
    {
        Eigen::VectorXd rand(k);
        const double s = stdDev / std::sqrt(k);
        for (std::size_t i = 0; i < k; i++)
            rand[i] = rng_.gaussian(0, s);
        c->phi(ru + rand, rx);
    }
    while (!atlas_.project(rx));
    
    // Be lazy about determining the new chart if we are not in the old one
    if (c->psiInverse(rx, ru), !c->inP(ru))
        c = NULL;
    else
        c->borderCheck(ru);
    astate->setRealState(rx, c);
}

/// AtlasValidStateSampler

/// Public

ompl::base::AtlasValidStateSampler::AtlasValidStateSampler (const AtlasStateSpacePtr &atlas, const SpaceInformation *si)
: ValidStateSampler(si), sampler_(*atlas)
{
}

bool ompl::base::AtlasValidStateSampler::sample (State *state)
{
    unsigned int fails = 0;
    bool valid;
    do
        sampler_.sampleUniform(state);
    while (!(valid = si_->isValid(state)) && ++fails < attempts_);
    
    return valid;
}

bool ompl::base::AtlasValidStateSampler::sampleNear (State *state, const State *near, const double distance)
{
    unsigned int fails = 0;
    bool valid;
    do
        sampler_.sampleUniformNear(state, near, distance);
    while (!(valid = si_->isValid(state)) && ++fails < attempts_);
    
    return valid;
}

/// AtlasMotionValidator

/// Public
ompl::base::AtlasMotionValidator::AtlasMotionValidator (SpaceInformation *si)
: MotionValidator(si), atlas_(*si->getStateSpace()->as<AtlasStateSpace>())
{
    checkSpace();
}

ompl::base::AtlasMotionValidator::AtlasMotionValidator (const SpaceInformationPtr &si)
: MotionValidator(si), atlas_(*si->getStateSpace()->as<AtlasStateSpace>())
{
    checkSpace();
}

bool ompl::base::AtlasMotionValidator::checkMotion (const State *s1, const State *s2) const
{
    // Simply invoke the manifold-traversing algorithm of the atlas
    return atlas_.followManifold(s1->as<AtlasStateSpace::StateType>(), s2->as<AtlasStateSpace::StateType>());
}

bool ompl::base::AtlasMotionValidator::checkMotion (const State *s1, const State *s2, std::pair<State *, double> &lastValid) const
{
    // Invoke the of the manifold-traversing algorithm to save intermediate states
    std::vector<AtlasStateSpace::StateType *> stateList;
    const AtlasStateSpace::StateType *const as1 = s1->as<AtlasStateSpace::StateType>();
    const AtlasStateSpace::StateType *const as2 = s2->as<AtlasStateSpace::StateType>();
    bool reached = atlas_.followManifold(as1, as2, false, &stateList);

    // XXX We are supposed to be able to assume that s1 is valid. However, it's not sometimes, and I
    // don't know why. We shouldn't have to check that stateList is non-empty.
    if (stateList.size() > 0) {
        for (std::size_t i = 0; i < stateList.size()-1; i++)
            atlas_.freeState(stateList[i]);
    
        // Check if manifold traversal stopped early and set its final state as lastValid
        if (!reached && lastValid.first)
            atlas_.copyState(lastValid.first, stateList.back());
        atlas_.freeState(stateList.back());
    }
    
    // Compute the interpolation parameter of the last valid state
    // (although if you then interpolate, you probably won't get this state back)
    if (!reached)
    {
        Eigen::Ref<const Eigen::VectorXd> x = lastValid.first->as<AtlasStateSpace::StateType>()->constVectorView();
        Eigen::Ref<const Eigen::VectorXd> a = as1->constVectorView();
        Eigen::Ref<const Eigen::VectorXd> b = as2->constVectorView();
        lastValid.second = (x-a).dot(b-a) / (b-a).squaredNorm();
    }
    
    return reached;
}

/// Private
void ompl::base::AtlasMotionValidator::checkSpace (void)
{
    if (!dynamic_cast<AtlasStateSpace *>(si_->getStateSpace().get()))
        throw ompl::Exception("AtlasMotionValidator's SpaceInformation needs to use an AtlasStateSpace!");
}

/// AtlasStateSpace::StateType

/// Public
ompl::base::AtlasStateSpace::StateType::StateType (const unsigned int dimension)
: RealVectorStateSpace::StateType(), chart_(NULL), dimension_(dimension)
{
    // Mimic what RealVectorStateSpace::allocState() would have done
    values = new double[dimension_];
}

ompl::base::AtlasStateSpace::StateType::~StateType(void)
{
    // Mimic what RealVectorStateSpace::freeState() would have done
    delete [] values;
}

void ompl::base::AtlasStateSpace::StateType::setRealState (const Eigen::VectorXd &x, AtlasChart *const c)
{
    setChart(c);
    for (std::size_t i = 0; i < dimension_; i++)
        (*this)[i]  = x[i];
}

Eigen::Map<Eigen::VectorXd> ompl::base::AtlasStateSpace::StateType::vectorView (void) const
{
    return Eigen::Map<Eigen::VectorXd>(values, dimension_);
}

Eigen::Map<const Eigen::VectorXd> ompl::base::AtlasStateSpace::StateType::constVectorView (void) const
{
    return Eigen::Map<const Eigen::VectorXd>(values, dimension_);
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::StateType::getChart (void) const
{
    return chart_;
}

void ompl::base::AtlasStateSpace::StateType::setChart (AtlasChart *const c) const
{
    chart_ = c;
}

/// AtlasStateSpace

/// Public
ompl::base::AtlasStateSpace::AtlasStateSpace (const unsigned int ambient, const unsigned int manifold)
: RealVectorStateSpace(ambient),
    n_(ambient), k_(manifold), delta_(0.02), epsilon_(0.1), exploration_(0.5), lambda_(2),
    projectionTolerance_(1e-8), projectionMaxIterations_(300), maxChartsPerExtension_(200), monteCarloSampleCount_(0), setup_(false), noAtlas_(false)
{
    setName("Atlas" + RealVectorStateSpace::getName());
        
    setRho(0.1);
    setAlpha(M_PI/16);
    setMonteCarloSampleCount(100);
    
    ballMeasure_ = std::pow(std::sqrt(M_PI), k_) / boost::math::tgamma(k_/2.0 + 1);
    
    chartNN_.setDistanceFunction(boost::bind(&chartNNDistanceFunction, _1, _2));
}

ompl::base::AtlasStateSpace::~AtlasStateSpace (void)
{
    for (std::size_t i = 0; i < charts_.size(); i++)
        delete charts_[i];
}

void ompl::base::AtlasStateSpace::bigJ (const Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out) const
{
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::VectorXd t1(n_-k_);
    Eigen::VectorXd t2(n_-k_);
    
    // Use a 7-point central difference stencil on each column
    for (std::size_t j = 0; j < n_; j++)
    {
        // Make step size as small as possible while still giving usable accuracy
        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (x[j] >= 1 ? x[j] : 1);
        
        y1[j] += h; y2[j] -= h;
        const Eigen::VectorXd m1 = (bigF(y1, t1),  bigF(y2, t2), (t1 - t2) / (y1[j]-y2[j]));   // Can't assume y1[j]-y2[j] == 2*h because of precision errors
        y1[j] += h; y2[j] -= h;
        const Eigen::VectorXd m2 = (bigF(y1, t1),  bigF(y2, t2), (t1 - t2) / (y1[j]-y2[j]));
        y1[j] += h; y2[j] -= h;
        const Eigen::VectorXd m3 = (bigF(y1, t1),  bigF(y2, t2), (t1 - t2) / (y1[j]-y2[j]));
        
        out.col(j) = 1.5*m1 - 0.6*m2 + 0.1*m3;
        
        // Reset for next iteration
        y1[j] = y2[j] = x[j];
    }
}

void ompl::base::AtlasStateSpace::stopBeingAnAtlas (const bool yes)
{
    noAtlas_ = yes;
}

void ompl::base::AtlasStateSpace::setup (void)
{
    if (setup_)
        return;
    
    setup_ = true;
    
    if (!si_)
        throw ompl::Exception("Must associate a SpaceInformation object to the AtlasStateSpace via setStateInformation() before use.");
    setDelta(delta_);   // This makes some setup-related calls
    RealVectorStateSpace::setup();
}

void ompl::base::AtlasStateSpace::clear (void)
{
    // Delete the non-anchor charts
    std::vector<AtlasChart *> anchorCharts;
    for (std::size_t i = 0; i < charts_.size(); i++)
    {
        if (charts_[i]->isAnchor())
            anchorCharts.push_back(charts_[i]);
        else
            delete charts_[i];
    }
    
    charts_.clear();
    chartNN_.clear();
    
    // Reinstate the anchor charts
    for (std::size_t i = 0; i < anchorCharts.size(); i++)
    {
        AtlasChart &c = *anchorCharts[i];
        c.clear();
        c.setID(charts_.size());
        
        for (std::size_t j = 0; j < charts_.size(); j++)
            AtlasChart::generateHalfspace(*charts_[j], c);
    
        charts_.add(&c, c.getMeasure());
        chartNN_.add(std::make_pair<>(c.getXoriginPtr(), charts_.size()-1));
    }
}

void ompl::base::AtlasStateSpace::setSpaceInformation (const SpaceInformationPtr &si)
{
    // Check that the object is valid
    if (!si)
        throw ompl::Exception("SpaceInformationPtr associated to the AtlasStateSpace was NULL.");
    if (si->getStateSpace().get() != this)
        throw ompl::Exception("SpaceInformation for AtlasStateSpace must be constructed from the same space object.");
    
    // Save only a raw pointer to prevent a cycle
    si_ = si.get();
    
    si_->setStateValidityCheckingResolution(delta_);
}

void ompl::base::AtlasStateSpace::setDelta (const double delta)
{
    if (delta <= 0)
        throw ompl::Exception("Please specify a positive delta.");
    delta_  = delta;
    
    if (setup_)
    {
        setLongestValidSegmentFraction(delta_ / getMaximumExtent());
    }
}

void ompl::base::AtlasStateSpace::setEpsilon (const double epsilon)
{
    if (epsilon <= 0)
        throw ompl::Exception("Please specify a positive epsilon.");
    epsilon_ = epsilon;
}

void ompl::base::AtlasStateSpace::setRho (const double rho)
{
    if (rho <= 0)
        throw ompl::Exception("Please specify a positive rho.");
    rho_ = rho;
    rho_s_ = rho_ / std::pow(1 - exploration_, 1.0/k_);
}

void ompl::base::AtlasStateSpace::setAlpha (const double alpha)
{
    if (alpha <= 0 || alpha >= M_PI_2)
        throw ompl::Exception("Please specify an alpha within the range (0,pi/2).");
    cos_alpha_ = std::cos(alpha);
}

void ompl::base::AtlasStateSpace::setExploration (const double exploration)
{
    if (exploration >= 1)
        throw ompl::Exception("Please specify an exploration value within the range [0,1).");
    exploration_ = exploration;
    
    // Update sampling radius
    setRho(rho_);
}

void ompl::base::AtlasStateSpace::setLambda (const double lambda)
{
    if (lambda <= 1)
        throw ompl::Exception("Please specify a lambda greater than 1.");
    lambda_ = lambda;
}

void ompl::base::AtlasStateSpace::setProjectionTolerance (const double tolerance)
{
    if (tolerance <= 0)
        throw ompl::Exception("Please specify a projection tolerance greater than 0.");
    projectionTolerance_ = tolerance;
}

void ompl::base::AtlasStateSpace::setProjectionMaxIterations (const unsigned int iterations)
{
    if (iterations == 0)
        throw ompl::Exception("Please specify a positive maximum projection iteration count.");
    projectionMaxIterations_ = iterations;
}

void ompl::base::AtlasStateSpace::setMaxChartsPerExtension (const unsigned int charts)
{
    maxChartsPerExtension_ = charts;
}

void ompl::base::AtlasStateSpace::setMonteCarloSampleCount (const unsigned int count)
{
    samples_.resize(count);
    // Generate random samples within the ball
    for (std::size_t i = monteCarloSampleCount_; i < samples_.size(); i++)
    {
        samples_[i].resize(k_);
        for (int j = 0; j < samples_[i].size(); j++)
            samples_[i][j] = rng_.gaussian01();
        samples_[i] *= std::pow(rng_.uniform01(), 1.0/samples_[i].size()) / samples_[i].norm();
    }
    monteCarloSampleCount_ = count;
}

double ompl::base::AtlasStateSpace::getDelta (void) const
{
    return delta_;
}

double ompl::base::AtlasStateSpace::getEpsilon (void) const
{
    return epsilon_;
}

double ompl::base::AtlasStateSpace::getRho (void) const
{
    return rho_;
}

double ompl::base::AtlasStateSpace::getAlpha (void) const
{
    return std::acos(cos_alpha_);
}

double ompl::base::AtlasStateSpace::getExploration (void) const
{
    return exploration_;
}

double ompl::base::AtlasStateSpace::getLambda (void) const
{
    return lambda_;
}

double ompl::base::AtlasStateSpace::getRho_s (void) const
{
    return rho_s_;
}

double ompl::base::AtlasStateSpace::getProjectionTolerance (void) const
{
    return projectionTolerance_;
}

unsigned int ompl::base::AtlasStateSpace::getProjectionMaxIterations (void) const
{
    return projectionMaxIterations_;
}

unsigned int ompl::base::AtlasStateSpace::getMaxChartsPerExtension (void) const
{
    return maxChartsPerExtension_;
}

unsigned int ompl::base::AtlasStateSpace::getMonteCarloSampleCount (void) const
{
    return monteCarloSampleCount_;
}

unsigned int ompl::base::AtlasStateSpace::getAmbientDimension (void) const
{
    return n_;
}

unsigned int ompl::base::AtlasStateSpace::getManifoldDimension (void) const
{
    return k_;
}

ompl::base::AtlasChart &ompl::base::AtlasStateSpace::anchorChart (const Eigen::VectorXd &xorigin) const
{
    // This could fail with an exception. We cannot recover if that happens.
    return newChart(xorigin, true);
}

ompl::base::AtlasChart &ompl::base::AtlasStateSpace::sampleChart (void) const
{
    double r = rng_.uniform01();
    
    if (charts_.size() < 1)
        throw ompl::Exception("Atlas sampled before any charts were made. Use AtlasStateSpace::anchorChart() first.");
    return *charts_.sample(r);
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::owningChart (const Eigen::VectorXd &x) const
{
    // Search through all nearby charts for a match
    std::vector<NNElement> nearbyCharts;
    chartNN_.nearestR(std::make_pair(&x, 0), rho_, nearbyCharts);
    Eigen::VectorXd tempx(n_), tempu(k_);
    for (std::size_t i = 0; i < nearbyCharts.size(); i++)
    {
        // The point must lie in the chart's validity region and polytope
        AtlasChart *c = charts_[nearbyCharts[i].second];
        c->psiInverse(x, tempu);
        c->phi(tempu, tempx);
        if ((tempx - x).norm() < epsilon_ && tempu.norm() < rho_ && c->inP(tempu))
            return c;
    }
    
    return NULL;
}

// This function can throw an ompl::Exception if the manifold misbehaves at xorigin!
ompl::base::AtlasChart &ompl::base::AtlasStateSpace::newChart (const Eigen::VectorXd &xorigin, const bool anchor) const
{
    AtlasChart &addedC = *new AtlasChart(*this, xorigin, anchor);
    addedC.setID(charts_.size());
    
    // Ensure all charts respect boundaries of the new one, and vice versa, but only look at nearby ones
    std::vector<NNElement> nearbyCharts;
    chartNN_.nearestR(std::make_pair(addedC.getXoriginPtr(), 0), 2*rho_, nearbyCharts);
    for (std::size_t i = 0; i < nearbyCharts.size(); i++)
        AtlasChart::generateHalfspace(*charts_[nearbyCharts[i].second], addedC);
    
    charts_.add(&addedC, addedC.getMeasure());
    chartNN_.add(std::make_pair<>(addedC.getXoriginPtr(), charts_.size()-1));
    
    return addedC;
}

double ompl::base::AtlasStateSpace::chartNNDistanceFunction (const NNElement &e1, const NNElement &e2)
{
    return (*e1.first - *e2.first).norm();
}
        
void ompl::base::AtlasStateSpace::dichotomicSearch (const AtlasChart &c, const Eigen::VectorXd &xinside, const Eigen::VectorXd &xoutside,
                                                    Eigen::Ref<Eigen::VectorXd> out) const
{
    // Cut the distance in half, moving toward xinside until we are inside the chart
    out = xoutside;
    Eigen::VectorXd u(k_);
    while (c.psiInverse(out, u), !c.inP(u))
        out = 0.5 * (xinside + out);
}

void ompl::base::AtlasStateSpace::updateMeasure (const AtlasChart &c) const
{
    // It's possible we're not tracking this chart yet, in which case don't worry about it
    std::size_t index = c.getID();
    if (index >= charts_.size())
        return;
    charts_.update(charts_.getElements()[index], c.getMeasure());
}

double ompl::base::AtlasStateSpace::getMeasureKBall (void) const
{
    return ballMeasure_;
}

const std::vector<Eigen::VectorXd> &ompl::base::AtlasStateSpace::getMonteCarloSamples (void) const
{
    return samples_;
}

std::size_t ompl::base::AtlasStateSpace::getChartCount (void) const
{
    return charts_.size();
}

/** \brief Traverse the manifold from \a from toward \a to. Returns true if we reached \a to, and false if
    * we stopped early for any reason, such as a collision or traveling too far. No collision checking is performed
    * if \a interpolate is true. If \a stateList is not NULL, the sequence of intermediates is saved to it, including
    * a copy of \a from, as well as the final state. */
bool ompl::base::AtlasStateSpace::followManifold (const StateType *from, const StateType *to, const bool interpolate,
                                                  std::vector<StateType *> *stateList) const
{
    unsigned int chartsCreated = 0;
    Eigen::VectorXd x_b = to->constVectorView();
    Eigen::Ref<const Eigen::VectorXd> x_a = from->constVectorView();
    AtlasChart *c = from->getChart();
    if (!c)
    {
        c = owningChart(x_a);
        if (!c)
        {
            // Chart creation can fail.
            try
            {
                c = &newChart(x_a);
            }
            catch (ompl::Exception &e)
            {
                return false;
            }
        }
        from->setChart(c);
    }
    const StateValidityCheckerPtr &svc = si_->getStateValidityChecker();
    StateType *currentState = allocState()->as<StateType>();
    currentState->setRealState(x_a, c);
    Eigen::Ref<Eigen::VectorXd> x_j = currentState->vectorView();
    
    // Collision check unless interpolating
    if (!interpolate && !svc->isValid(from)) {
        std::cout << "Warning: 'from' state not valid!\n";
        return false;
    }
        
    // Save a copy of the from state
    if (stateList)
    {
        stateList->clear();
        stateList->push_back(si_->cloneState(from)->as<StateType>());
    }
    
    Eigen::VectorXd u_j(k_), u_b(k_);
    
    // We will stop if we exit the ball of radius d_0 centered at x_a
    double d_0 = (x_a - x_b).norm();
    double d = 0;
    
    // Project from and to points onto the chart
    c->psiInverse(x_j, u_j);
    c->psiInverse(x_b, u_b);
    //u_b = u_j + (d_0 - d)*(u_b - u_j).normalized();
    //c->phi(u_b, x_b);

    Eigen::VectorXd tempx(n_);
    while (((u_b - u_j).squaredNorm() > delta_*delta_))
    {
        // Step by delta toward the target and project
        u_j = u_j + delta_*(u_b - u_j).normalized();    // Note the difference to pseudocode (line 13): a similar mistake to line 8
        c->psi(u_j, tempx);
        double d_s = (tempx - x_j).norm();
        x_j = tempx;
        d += d_s;
        
        // Collision check unless interpolating
        currentState->setChart(c);
        if (!interpolate && !svc->isValid(currentState))
            break;
        
        // Check stopping criteria regarding how far we've gone
        if ((x_j - x_a).squaredNorm() > d_0*d_0 || d > lambda_*d_0 || chartsCreated > maxChartsPerExtension_)
            break;

        c->phi(u_j, tempx);
        if (((x_j - tempx).squaredNorm() > epsilon_*epsilon_ || delta_/d_s < cos_alpha_
             || u_j.squaredNorm() > rho_*rho_) || !c->inP(u_j))
        {
            // Left the validity region or polytope of the chart; find or make a new one
            // The paper says we should always make (never find) one here, but, empirically, that's not always the case
            c = owningChart(x_j);
            if (!c)
            {
                // Chart creation can fail.
                try
                {
                    c = &newChart(x_j);
                }
                catch (ompl::Exception &e)
                {
                    // Quit.
                    freeState(currentState);
                    return false;
                }
                chartsCreated++;
            }

            // Re-project onto the different chart
            c->psiInverse(x_j, u_j);
            c->psiInverse(x_b, u_b);
            //u_b = u_j + (d_0 - d)*(u_b - u_j).normalized();
            //c->phi(u_b, x_b);
        }
        
        // Keep the state in a list, if requested
        if (stateList)
            stateList->push_back(si_->cloneState(currentState)->as<StateType>());
    }

    if (chartsCreated > maxChartsPerExtension_)
        OMPL_DEBUG("Stopping extension early b/c too many charts created.");
    // Reached goal if final point is within delta and both current and goal are valid.
    const bool currentValid = interpolate || svc->isValid(currentState);
    const bool goalValid = interpolate || svc->isValid(to);
    const bool reached = ((x_b - x_j).squaredNorm() <= delta_*delta_) && currentValid && goalValid;
    
    // Append a copy of the target state, since we're within delta, but didn't hit it exactly
    if (reached && stateList)
    {
        StateType *toCopy = si_->cloneState(to)->as<StateType>();
        toCopy->setChart(c);
        stateList->push_back(toCopy);
    }
    
    freeState(currentState);
    return reached;
}

int ompl::base::AtlasStateSpace::estimateFrontierPercent () const {
    int frontier = 0;
    for (std::size_t i = 0; i < charts_.size(); i++) {
        frontier += charts_[i]->estimateIsFrontier();
    }
    return (100 * frontier) / charts_.size();
}

void ompl::base::AtlasStateSpace::dumpMesh (std::ostream &out) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;
    std::vector<Eigen::VectorXd> vertices;
    for (std::size_t i = 0; i < charts_.size(); i++)
    {
        // Write the vertices and the faces
        std::cout << "\rDumping chart " << i << std::flush;
        const AtlasChart &c = *charts_[i];
        c.toPolygon(vertices);
        std::stringstream poly;
        std::size_t fvcount = 0;
        for (std::size_t j = 0; j < vertices.size(); j++)
        {
            v << vertices[j].transpose() << "\n";
            poly << vcount++ << " ";
            fvcount++;
        }
        
        if (fvcount > 2)
        {
            f << fvcount << " " << poly.str() << "\n";
            fcount += 1;
        }
    }
    std::cout << "\n";
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}

void ompl::base::AtlasStateSpace::dumpGraph (const PlannerData::Graph &graph, std::ostream &out, const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;
    
    BOOST_FOREACH (PlannerData::Graph::Edge edge, boost::edges(graph))
    {
        std::vector<StateType *> stateList;
        const State *const source = boost::get(vertex_type, graph, boost::source(edge, graph))->getState();
        const State *const target = boost::get(vertex_type, graph, boost::target(edge, graph))->getState();
        
        if (!asIs)
            followManifold(source->as<StateType>(), target->as<StateType>(), true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            v << target->as<StateType>()->constVectorView().transpose() << "\n";
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount-3 << " " << vcount-2 << " " << vcount-1 << "\n";
            fcount++;
            for (std::size_t j = 0; j < stateList.size(); j++)
                freeState(stateList[j]);
            continue;
        }
        StateType *to, *from = stateList[0];
        v << from->constVectorView().transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i];
            from = stateList[i-1];
            v << to->constVectorView().transpose() << "\n";
            v << from->constVectorView().transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount-3 : vcount-4) << " " << vcount-2 << " " << vcount-1 << "\n";
            fcount++;
            freeState(stateList[i-1]);
            reset = false;
        }
        freeState(stateList.back());
    }
    
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}

void ompl::base::AtlasStateSpace::dumpPath (ompl::geometric::PathGeometric &path, std::ostream &out, const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;
    
    const std::vector<State *> &waypoints = path.getStates();
    for (std::size_t i = 0; i < waypoints.size()-1; i++)
    {
        std::vector<StateType *> stateList;
        const State *const source = waypoints[i];
        const State *const target = waypoints[i+1];
        
        if (!asIs)
            followManifold(source->as<StateType>(), target->as<StateType>(), true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            v << target->as<StateType>()->constVectorView().transpose() << "\n";
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount-3 << " " << vcount-2 << " " << vcount-1 << "\n";
            fcount++;
            for (std::size_t j = 0; j < stateList.size(); j++)
                freeState(stateList[j]);
            continue;
        }
        StateType *to, *from = stateList[0];
        v << from->constVectorView().transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i];
            from = stateList[i-1];
            v << to->constVectorView().transpose() << "\n";
            v << from->constVectorView().transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount-3 : vcount-4) << " " << vcount-2 << " " << vcount-1 << "\n";
            fcount++;
            freeState(stateList[i-1]);
            reset = false;
        }
        freeState(stateList.back());
    }
    
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}

bool ompl::base::AtlasStateSpace::project (Eigen::Ref<Eigen::VectorXd> x) const
{
    // Newton's method
    unsigned int iter = 0;
    Eigen::VectorXd f(n_-k_);
    Eigen::MatrixXd j(n_-k_,n_);
    while ((bigF(x, f), f.norm() > projectionTolerance_) && iter++ < projectionMaxIterations_)
    {
        // Compute pseudoinverse of Jacobian
        bigJ(x, j);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd = j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        const double tolerance = std::numeric_limits<double>::epsilon() * getAmbientDimension() * svd.singularValues().array().abs().maxCoeff();
        x -= svd.matrixV()
            * Eigen::MatrixXd((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).asDiagonal()
            * svd.matrixU().adjoint()
            * f;
    }
    
    if (iter > projectionMaxIterations_)
        return false;
    
    return true;
}

void ompl::base::AtlasStateSpace::interpolate (const State *from, const State *to, const double t, State *state) const
{
     // Interpolate like a real vector space
     RealVectorStateSpace::interpolate(from, to, t, state);
     if (noAtlas_)
         return;
     
    // Find or make a chart for the point
    StateType *const astate = state->as<StateType>();
    AtlasChart *c = owningChart(astate->constVectorView());
    if (!c)
    {
        project(astate->vectorView());
        // Chart creation can fail.
        try
        {
            c = &newChart(astate->constVectorView());
        }
        catch (ompl::Exception &e) {}
    }
    astate->setChart(c);
    
    // Project using this chart
    Eigen::VectorXd x = astate->constVectorView();
    c->psiFromGuess(x, astate->vectorView()); 
}

void ompl::base::AtlasStateSpace::fastInterpolate (const std::vector<StateType *> &stateList, const double t, State *state) const
{
    std::size_t n = stateList.size();
    double *d = new double[n];

    // Compute partial sums of distances between intermediate states
    d[0] = 0;
    for (std::size_t i = 1; i < n; i++)
        d[i] = d[i-1] + distance(stateList[i-1], stateList[i]);
    
    // Find the two adjacent states between which lies t
    std::size_t i = 0;
    double tt;
    if (d[n-1] == 0)
    {
        // Corner case where total distance is near 0; prevents division by 0
        i = n-1;
        tt = t;
    }
    else
    {
        while (i < n-1 && d[i]/d[n-1] <= t)
            i++;
        tt = t-d[i-1]/d[n-1];
    }
    
    // Interpolate between these two states
    RealVectorStateSpace::interpolate(stateList[i > 0 ? i-1 : 0], stateList[i], tt, state);
    delete [] d;
    
    // Set the correct chart, guessing it might be one of the adjacent charts first
    StateType *astate = state->as<StateType>();
    Eigen::Ref<const Eigen::VectorXd> x = astate->constVectorView();
    AtlasChart &c1 = *stateList[i > 0 ? i-1 : 0]->getChart();
    AtlasChart &c2 = *stateList[i]->getChart();
    Eigen::VectorXd u(k_);
    if (c1.psiInverse(x, u), c1.inP(u))
        astate->setChart(&c1);
    else if (c2.psiInverse(x, u), c2.inP(u))
        astate->setChart(&c2);
    else
    {
        AtlasChart *c = owningChart(x);
        if (!c)
        {
            // Chart creation can fail.
            try
            {
                c = &newChart(x);
            }
            catch (ompl::Exception &e) {}
        }
        astate->setChart(c);
    }
}

bool ompl::base::AtlasStateSpace::hasSymmetricInterpolate (void) const
{
    return true;
}

void ompl::base::AtlasStateSpace::copyState (State *destination, const State *source) const
{
    RealVectorStateSpace::copyState(destination, source);
    destination->as<StateType>()->setChart(source->as<StateType>()->getChart());
}

ompl::base::StateSamplerPtr ompl::base::AtlasStateSpace::allocDefaultStateSampler (void) const
{
    if (noAtlas_)
        return RealVectorStateSpace::allocDefaultStateSampler();
    return StateSamplerPtr(new AtlasStateSampler(*this));
}

ompl::base::State *ompl::base::AtlasStateSpace::allocState (void) const
{
    return new StateType(n_);
}

void ompl::base::AtlasStateSpace::freeState (State *state) const
{
    StateType *const astate = state->as<StateType>();
    delete astate;
}
