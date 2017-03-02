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

#include <boost/graph/iteration_macros.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/// AtlasStateSampler

/// Public

ompl::base::AtlasStateSampler::AtlasStateSampler(const SpaceInformation *si)
  : StateSampler(si->getStateSpace().get()), atlas_(*si->getStateSpace()->as<AtlasStateSpace>())
{
    AtlasStateSpace::checkSpace(si);
}

ompl::base::AtlasStateSampler::AtlasStateSampler(const AtlasStateSpace &atlas) : StateSampler(&atlas), atlas_(atlas)
{
    // TODO (cav2): inline some small things.
}

void ompl::base::AtlasStateSampler::sampleUniform(State *state)
{
    Eigen::Ref<Eigen::VectorXd> rx = state->as<AtlasStateSpace::StateType>()->vectorView();
    Eigen::VectorXd ry(atlas_.getAmbientDimension());
    Eigen::VectorXd ru(atlas_.getManifoldDimension());
    AtlasChart *c;

    // Sampling a point on the manifold.
    Eigen::VectorXd f(atlas_.getAmbientDimension() - atlas_.getManifoldDimension());
    int tries = 100;
    do
    {
        // Rejection sampling to find a point inside a chart's polytope.
        do
        {
            // Pick a chart.
            c = atlas_.sampleChart();

            // Sample a point within rho_s of the center. This is done by
            // sampling uniformly on the surface and multiplying by a distance
            // whose distribution is biased according to spherical volume.
            for (int i = 0; i < ru.size(); i++)
                ru[i] = rng_.gaussian01();
            ru *= atlas_.getRho_s() * std::pow(rng_.uniform01(), 1.0 / ru.size()) / ru.norm();
            tries--;
        } while (tries > 0 && !c->inPolytope(ru));

        // Project. Will need to try again if this fails.
        c->psi(ru, rx);
        atlas_.getConstraint()->function(rx, f);
    } while (tries > 0 && (!rx.allFinite() || f.norm() > atlas_.getConstraint()->getProjectionTolerance()));

    if (tries == 0)
    {
        // Consider decreasing rho and/or the exploration paramter if this
        // becomes a problem.
        OMPL_WARN("ompl::base::AtlasStateSpace::sampleUniform(): "
                  "Took too long; returning center of a random chart.");
        rx = c->getXorigin();
    }

    // Extend polytope of neighboring chart wherever point is near the border.
    c->psiInverse(rx, ru);
    c->borderCheck(ru);
    state->as<AtlasStateSpace::StateType>()->setChart(atlas_.owningChart(rx));
}

void ompl::base::AtlasStateSampler::sampleUniformNear(State *state, const State *near, const double distance)
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
            c = atlas_.newChart(n);
        if (!c)
        {
            OMPL_ERROR("ompl::base::AtlasStateSpace::sampleUniformNear(): "
                       "Sampling failed because chart creation failed! Falling back to uniform sample.");
            sampleUniform(state);
            return;
        }
        anear->setChart(c);
    }

    // Sample a point from the starting chart.
    c->psiInverse(n, ru);
    int tries = 100;
    Eigen::VectorXd f(atlas_.getAmbientDimension() - atlas_.getManifoldDimension());
    do
    {
        tries--;
        // Sample within distance
        Eigen::VectorXd uoffset(atlas_.getManifoldDimension());
        for (int i = 0; i < uoffset.size(); i++)
            uoffset[i] = rng_.gaussian01();
        uoffset *= distance * std::pow(rng_.uniform01(), 1.0 / uoffset.size()) / uoffset.norm();
        c->phi(ru + uoffset, rx);
    } while (tries > 0 && !atlas_.getConstraint()->project(rx));  // Try again if we can't project.

    if (tries == 0)
    {
        // Consider decreasing the distance argument if this becomes a
        // problem. Check planner code to see how it gets chosen.
        OMPL_WARN("ompl::base:::AtlasStateSpace::sampleUniformNear(): "
                  "Took too long; returning initial point.");
        rx = n;
    }

    // Be lazy about determining the new chart if we are not in the old one
    if (c->psiInverse(rx, ru), !c->inPolytope(ru))
        c = nullptr;
    else
        c->borderCheck(ru);
    astate->setRealState(rx, c);
}

void ompl::base::AtlasStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
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
            c = atlas_.newChart(m);
        if (!c)
        {
            OMPL_ERROR("ompl::base::AtlasStateSpace::sampleGaussian(): "
                       "Sampling failed because chart creation failed! Falling back to uniform sample.");
            sampleUniform(state);
            return;
        }
        amean->setChart(c);
    }
    c->psiInverse(m, ru);

    // Sample a point in a normal distribution on the starting chart.
    int tries = 100;
    do
    {
        tries--;
        Eigen::VectorXd rand(k);
        const double s = stdDev / std::sqrt(k);
        for (std::size_t i = 0; i < k; i++)
            rand[i] = rng_.gaussian(0, s);
        c->phi(ru + rand, rx);
    } while (tries > 0 && !atlas_.getConstraint()->project(rx));  // Try again if we can't project.

    if (tries == 0)
    {
        OMPL_WARN("ompl::base::AtlasStateSpace::sampleUniforGaussian(): "
                  "Took too long; returning initial point.");
        rx = m;
    }

    // Be lazy about determining the new chart if we are not in the old one
    if (c->psiInverse(rx, ru), !c->inPolytope(ru))
        c = nullptr;
    else
        c->borderCheck(ru);
    astate->setRealState(rx, c);
}

/// AtlasValidStateSampler

/// Public

ompl::base::AtlasValidStateSampler::AtlasValidStateSampler(const SpaceInformation *si)
  : ValidStateSampler(si), sampler_(si)
{
    AtlasStateSpace::checkSpace(si);
}

bool ompl::base::AtlasValidStateSampler::sample(State *state)
{
    // Rejection sample for at most attempts_ tries.
    unsigned int tries = 0;
    bool valid;
    do
        sampler_.sampleUniform(state);
    while (!(valid = si_->isValid(state)) && ++tries < attempts_);

    return valid;
}

bool ompl::base::AtlasValidStateSampler::sampleNear(State *state, const State *near, const double distance)
{
    // Rejection sample for at most attempts_ tries.
    unsigned int tries = 0;
    bool valid;
    do
        sampler_.sampleUniformNear(state, near, distance);
    while (!(valid = si_->isValid(state)) && ++tries < attempts_);

    return valid;
}

/// AtlasMotionValidator

/// Public

ompl::base::AtlasMotionValidator::AtlasMotionValidator(SpaceInformation *si)
  : MotionValidator(si), atlas_(*si->getStateSpace()->as<AtlasStateSpace>())
{
    AtlasStateSpace::checkSpace(si);
}

ompl::base::AtlasMotionValidator::AtlasMotionValidator(const SpaceInformationPtr &si)
  : MotionValidator(si), atlas_(*si->getStateSpace()->as<AtlasStateSpace>())
{
    AtlasStateSpace::checkSpace(si.get());
}

bool ompl::base::AtlasMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    // Simply invoke the manifold-traversing algorithm of the atlas
    return atlas_.traverseManifold(s1->as<AtlasStateSpace::StateType>(), s2->as<AtlasStateSpace::StateType>());
}

bool ompl::base::AtlasMotionValidator::checkMotion(const State *s1, const State *s2,
                                                   std::pair<State *, double> &lastValid) const
{
    // Invoke the manifold-traversing algorithm to save intermediate states
    std::vector<AtlasStateSpace::StateType *> stateList;
    const AtlasStateSpace::StateType *const as1 = s1->as<AtlasStateSpace::StateType>();
    const AtlasStateSpace::StateType *const as2 = s2->as<AtlasStateSpace::StateType>();
    bool reached = atlas_.traverseManifold(as1, as2, false, &stateList);

    // We are supposed to be able to assume that s1 is valid. However, it's not
    // on rare occasions, and I don't know why. This makes stateList empty.
    if (stateList.empty())
    {
        if (lastValid.first)
            atlas_.copyState(lastValid.first, as1);
        lastValid.second = 0;
        return false;
    }

    double distanceTraveled = 0;
    for (std::size_t i = 0; i < stateList.size() - 1; i++)
    {
        if (!reached)
            distanceTraveled += atlas_.distance(stateList[i], stateList[i + 1]);
        atlas_.freeState(stateList[i]);
    }

    if (!reached && lastValid.first)
    {
        // Check if manifold traversal stopped early and set its final state as
        // lastValid.
        atlas_.copyState(lastValid.first, stateList.back());
        // Compute the interpolation parameter of the last valid
        // state. (Although if you then interpolate, you probably won't get this
        // exact state back.)
        double approxDistanceRemaining = atlas_.distance(lastValid.first, as2);
        lastValid.second = distanceTraveled / (distanceTraveled + approxDistanceRemaining);
    }

    atlas_.freeState(stateList.back());
    return reached;
}

/// AtlasStateSpace::StateType

/// Public

ompl::base::AtlasStateSpace::StateType::StateType(const unsigned int &dimension)
  : RealVectorStateSpace::StateType(), dimension_(dimension)
{
    // Do what RealVectorStateSpace::allocState() would have done.
    values = new double[dimension_];
}

ompl::base::AtlasStateSpace::StateType::~StateType()
{
    // Do what RealVectorStateSpace::freeState() would have done.
    delete[] values;
}

void ompl::base::AtlasStateSpace::StateType::copyFrom(const StateType *source)
{
    for (unsigned int i = 0; i < dimension_; ++i)
        (*this)[i] = (*source)[i];
    chart_ = source->chart_;
}

// TODO (cav2): give this a better name.
void ompl::base::AtlasStateSpace::StateType::setRealState(const Eigen::VectorXd &x, AtlasChart *c)
{
    for (std::size_t i = 0; i < dimension_; i++)
        (*this)[i] = x[i];
    chart_ = c;
}

Eigen::Map<Eigen::VectorXd> ompl::base::AtlasStateSpace::StateType::vectorView() const
{
    return Eigen::Map<Eigen::VectorXd>(values, dimension_);
}

Eigen::Map<const Eigen::VectorXd> ompl::base::AtlasStateSpace::StateType::constVectorView() const
{
    return Eigen::Map<const Eigen::VectorXd>(values, dimension_);
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::StateType::getChart() const
{
    return chart_;
}

void ompl::base::AtlasStateSpace::StateType::setChart(AtlasChart *c) const
{
    chart_ = c;
}

/// AtlasStateSpace

/// Public

ompl::base::AtlasStateSpace::AtlasStateSpace(const StateSpacePtr space, const ConstraintPtr constraint)
  : RealVectorStateSpace(space->getDimension())
  , si_(nullptr)
  , ss_(space)
  , constraint_(constraint)
  , n_(space->getDimension())
  , k_(constraint_->getManifoldDimension())
  , delta_(0.02)
  , epsilon_(0.1)
  , exploration_(0.5)
  , lambda_(2)
  , maxChartsPerExtension_(200)
  , setup_(false)
{
    setName("Atlas" + RealVectorStateSpace::getName());

    setRho(0.1);
    setAlpha(M_PI / 16);

    chartNN_.setDistanceFunction(&chartNNDistanceFunction);
}

ompl::base::AtlasStateSpace::~AtlasStateSpace()
{
    for (AtlasChart *c : charts_)
        delete c;
}

void ompl::base::AtlasStateSpace::setup()
{
    if (setup_)
        return;

    if (!si_)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setup(): "
                              "Must associate a SpaceInformation object to the AtlasStateSpace via "
                              "setStateInformation() before use.");

    setup_ = true;
    setDelta(delta_);  // This makes some setup-related calls

    RealVectorStateSpace::setup();
}

/// Static.
void ompl::base::AtlasStateSpace::checkSpace(const SpaceInformation *si)
{
    if (!dynamic_cast<AtlasStateSpace *>(si->getStateSpace().get()))
        throw ompl::Exception("ompl::base::AtlasMotionValidator(): "
                              "si needs to use an AtlasStateSpace!");
}

void ompl::base::AtlasStateSpace::clear()
{
    // Delete the non-anchor charts
    std::vector<AtlasChart *> anchorCharts;
    for (AtlasChart *c : charts_)
    {
        if (c->isAnchor())
            anchorCharts.push_back(c);
        else
            delete c;
    }

    charts_.clear();
    chartNN_.clear();

    // Reinstate the anchor charts
    for (AtlasChart *anchor : anchorCharts)
    {
        anchor->clear();

        for (AtlasChart *c : charts_)
            AtlasChart::generateHalfspace(c, anchor);

        anchor->setID(charts_.size());
        chartNN_.add(std::make_pair<>(&anchor->getXorigin(), charts_.size()));
        charts_.push_back(anchor);
    }
}

void ompl::base::AtlasStateSpace::setSpaceInformation(const SpaceInformationPtr &si)
{
    // Check that the object is valid
    if (!si.get())
        throw ompl::Exception("ompl::base::AtlasStateSpace::setSpaceInformation(): "
                              "si is nullptr.");
    if (si->getStateSpace().get() != this)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setSpaceInformation(): "
                              "si for AtlasStateSpace must be constructed from the same sta space object.");

    // Save only a raw pointer to prevent a cycle
    si_ = si.get();

    si_->setStateValidityCheckingResolution(delta_);
}

void ompl::base::AtlasStateSpace::setDelta(const double delta)
{
    if (delta <= 0)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setDelta(): "
                              "delta must be positive.");
    delta_ = delta;

    if (setup_)
    {
        setLongestValidSegmentFraction(delta_ / getMaximumExtent());
    }
}

void ompl::base::AtlasStateSpace::setEpsilon(const double epsilon)
{
    if (epsilon <= 0)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setEpsilon(): "
                              "epsilon must be positive.");
    epsilon_ = epsilon;
}

void ompl::base::AtlasStateSpace::setRho(const double rho)
{
    if (rho <= 0)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setRho(): "
                              "rho must be positive.");
    rho_ = rho;
    rho_s_ = rho_ / std::pow(1 - exploration_, 1.0 / k_);
}

void ompl::base::AtlasStateSpace::setAlpha(const double alpha)
{
    if (alpha <= 0 || alpha >= M_PI_2)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setAlpha(): "
                              "alpha must be in (0, pi/2).");
    cos_alpha_ = std::cos(alpha);
}

void ompl::base::AtlasStateSpace::setExploration(const double exploration)
{
    if (exploration >= 1)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setExploration(): "
                              "exploration must be in [0, 1).");
    exploration_ = exploration;

    // Update sampling radius
    setRho(rho_);
}

void ompl::base::AtlasStateSpace::setLambda(const double lambda)
{
    if (lambda <= 1)
        throw ompl::Exception("ompl::base::AtlasStateSpace::setLambda(): "
                              "lambda must be > 1.");
    lambda_ = lambda;
}

void ompl::base::AtlasStateSpace::setMaxChartsPerExtension(const unsigned int charts)
{
    maxChartsPerExtension_ = charts;
}

double ompl::base::AtlasStateSpace::getDelta() const
{
    return delta_;
}

double ompl::base::AtlasStateSpace::getEpsilon() const
{
    return epsilon_;
}

double ompl::base::AtlasStateSpace::getRho() const
{
    return rho_;
}

double ompl::base::AtlasStateSpace::getAlpha() const
{
    return std::acos(cos_alpha_);
}

double ompl::base::AtlasStateSpace::getExploration() const
{
    return exploration_;
}

double ompl::base::AtlasStateSpace::getLambda() const
{
    return lambda_;
}

double ompl::base::AtlasStateSpace::getRho_s() const
{
    return rho_s_;
}

unsigned int ompl::base::AtlasStateSpace::getMaxChartsPerExtension() const
{
    return maxChartsPerExtension_;
}

unsigned int ompl::base::AtlasStateSpace::getAmbientDimension() const
{
    return n_;
}

unsigned int ompl::base::AtlasStateSpace::getManifoldDimension() const
{
    return k_;
}

ompl::base::ConstraintPtr ompl::base::AtlasStateSpace::getConstraint() const
{
    return constraint_;
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::anchorChart(const Eigen::VectorXd &xorigin) const
{
    // This could fail with an exception. We cannot recover if that happens.
    AtlasChart *c = newChart(xorigin);
    if (!c)
    {
        throw ompl::Exception("ompl::base::AtlasStateSpace::anchorChart(): "
                              "Initial chart creation failed. Cannot proceed.");
    }
    c->makeAnchor();
    return c;
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::newChart(const Eigen::VectorXd &xorigin) const
{
    AtlasChart *addedC;
    try
    {
        addedC = new AtlasChart(constraint_, rho_, epsilon_, xorigin);
    }
    catch (ompl::Exception &e)
    {
        OMPL_ERROR("ompl::base::AtlasStateSpace::newChart(): "
                   "Failed because manifold looks degenerate here.");
        return nullptr;
    }

    // Ensure all charts respect boundaries of the new one, and vice versa, but
    // only look at nearby ones (within 2*rho).
    std::vector<NNElement> nearbyCharts;
    chartNN_.nearestR(std::make_pair(&addedC->getXorigin(), 0), 2 * rho_, nearbyCharts);
    for (auto &near : nearbyCharts)
        AtlasChart::generateHalfspace(charts_[near.second], addedC);

    addedC->setID(charts_.size());
    chartNN_.add(std::make_pair<>(&addedC->getXorigin(), charts_.size()));
    charts_.push_back(addedC);

    return addedC;
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::sampleChart() const
{
    if (charts_.empty())
        throw ompl::Exception("ompl::base::AtlasStateSpace::sampleChart(): "
                              "Atlas sampled before any charts were made. Use AtlasStateSpace::anchorChart() first.");

    return charts_[rng_.uniformInt(0, charts_.size() - 1)];
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::owningChart(const Eigen::VectorXd &x) const
{
    // Search through all nearby charts for a match
    std::vector<NNElement> nearbyCharts;
    chartNN_.nearestR(std::make_pair(&x, 0), rho_, nearbyCharts);
    Eigen::VectorXd tempx(n_), tempu(k_);
    for (auto &near : nearbyCharts)
    {
        // The point must lie in the chart's validity region and polytope
        AtlasChart *c = charts_[near.second];
        c->psiInverse(x, tempu);
        c->phi(tempu, tempx);
        if ((tempx - x).norm() < epsilon_ && tempu.norm() < rho_ && c->inPolytope(tempu))
            return c;
    }

    return nullptr;
}

// Static
double ompl::base::AtlasStateSpace::chartNNDistanceFunction(const NNElement &e1, const NNElement &e2)
{
    return (*e1.first - *e2.first).norm();
}

std::size_t ompl::base::AtlasStateSpace::getChartCount() const
{
    return charts_.size();
}

bool ompl::base::AtlasStateSpace::traverseManifold(const StateType *from, const StateType *to, const bool interpolate,
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
            c = newChart(x_a);
        if (!c)
        {
            OMPL_DEBUG("ompl::base::AtlasStateSpace::traverseManifold(): "
                       "'from' state has no chart!");
            return false;
        }
        from->setChart(c);
    }
    const StateValidityCheckerPtr &svc = si_->getStateValidityChecker();
    StateType *currentState = allocState()->as<StateType>();
    currentState->setRealState(x_a, c);
    Eigen::Ref<Eigen::VectorXd> x_j = currentState->vectorView();

    // Collision check unless interpolating.
    if (!interpolate && (!x_a.allFinite() || !svc->isValid(from)))
    {
        OMPL_DEBUG("ompl::base::AtlasStateSpace::traverseManifold(): "
                   "'from' state not valid!");
        freeState(currentState);
        return false;
    }

    // Save a copy of the from state.
    if (stateList)
    {
        stateList->clear();
        stateList->push_back(si_->cloneState(from)->as<StateType>());
    }

    Eigen::VectorXd u_j(k_), u_b(k_);

    // We will stop if we exit the ball of radius d_0 centered at x_a.
    double d_0 = (x_a - x_b).norm();
    double d = 0;

    // Project from and to points onto the chart
    c->psiInverse(x_j, u_j);
    c->psiInverse(x_b, u_b);

    Eigen::VectorXd tempx(n_);
    while (((u_b - u_j).squaredNorm() > delta_ * delta_))
    {
        // Step by delta toward the target and project.
        // Note the correction to the pseudocode line 13 (Jaillet et al.).
        u_j = u_j + delta_ * (u_b - u_j).normalized();
        c->psi(u_j, tempx);
        double d_s = (tempx - x_j).norm();
        x_j = tempx;
        d += d_s;

        // Collision check unless interpolating.
        currentState->setChart(c);
        if (!interpolate && (!x_j.allFinite() || !svc->isValid(currentState)))
            break;

        // Check stopping criteria regarding how far we've gone.
        if ((x_j - x_a).squaredNorm() > d_0 * d_0 || d > lambda_ * d_0 || chartsCreated > maxChartsPerExtension_)
            break;

        // Check if we left the validity region or polytope of the chart.
        c->phi(u_j, tempx);
        if (((x_j - tempx).squaredNorm() > epsilon_ * epsilon_ || delta_ / d_s < cos_alpha_ ||
             u_j.squaredNorm() > rho_ * rho_) ||
            !c->inPolytope(u_j))
        {
            // Find or make a new chart.
            c = owningChart(x_j);
            if (!c)
            {
                c = newChart(x_j);
                chartsCreated++;
            }
            if (!c)
            {
                // Pretend like we hit an obstacle.
                OMPL_ERROR("ompl::base::AtlasStateSpace::traverseManifold(): "
                           "Treating singularity as an obstacle.");
                break;
            }

            // Re-project onto the next chart.
            c->psiInverse(x_j, u_j);
            c->psiInverse(x_b, u_b);
        }

        // Keep the state in a list, if requested.
        if (stateList)
            stateList->push_back(si_->cloneState(currentState)->as<StateType>());
    }

    if (chartsCreated > maxChartsPerExtension_)
        OMPL_WARN("ompl::base::AtlasStateSpace::traverseManifold(): "
                  "Too many new charts. Stopping extension early.");

    // Reached goal if final point is within delta and both current and goal are valid.
    const bool currentValid = interpolate || (x_j.allFinite() && svc->isValid(currentState));
    const bool goalValid = interpolate || svc->isValid(to);
    const bool reached = ((x_b - x_j).squaredNorm() <= delta_ * delta_) && currentValid && goalValid;

    // Append a copy of the target state, since we're within delta, but didn't hit it exactly.
    if (reached && stateList)
    {
        stateList->push_back(si_->cloneState(to)->as<StateType>());
        if (!to->getChart())
            stateList->back()->setChart(c);
    }

    freeState(currentState);
    return reached;
}

void ompl::base::AtlasStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    // Get the list of intermediate states along the manifold.
    std::vector<StateType *> stateList;
    bool succeeded = traverseManifold(from->as<StateType>(), to->as<StateType>(), true, &stateList);
    if (!succeeded)
        stateList.push_back(si_->cloneState(to)->as<StateType>());
    piecewiseInterpolate(stateList, t, state);
    for (StateType *state : stateList)
        freeState(state);
}

void ompl::base::AtlasStateSpace::piecewiseInterpolate(const std::vector<StateType *> &stateList, const double t,
                                                       State *state) const
{
    std::size_t n = stateList.size();
    auto d = new double[n];

    // Compute partial sums of distances between intermediate states.
    d[0] = 0;
    for (std::size_t i = 1; i < n; i++)
        d[i] = d[i - 1] + distance(stateList[i - 1], stateList[i]);

    // Find the two adjacent states that t lies between.
    std::size_t i = 0;
    double tt;
    if (d[n - 1] == 0)
    {
        // Corner case where total distance is 0.
        i = n - 1;
        tt = t;
    }
    else
    {
        while (i < n - 1 && d[i] / d[n - 1] <= t)
            i++;
        tt = t - d[i - 1] / d[n - 1];
    }

    // Linearly interpolate between these two states.
    RealVectorStateSpace::interpolate(stateList[i > 0 ? i - 1 : 0], stateList[i], tt, state);
    delete[] d;

    // Set the correct chart, guessing it might be one of the adjacent charts.
    StateType *astate = state->as<StateType>();
    Eigen::Ref<const Eigen::VectorXd> x = astate->constVectorView();
    AtlasChart *c1 = stateList[i > 0 ? i - 1 : 0]->getChart();
    AtlasChart *c2 = stateList[i]->getChart();
    Eigen::VectorXd u(k_);
    if (c1->psiInverse(x, u), c1->inPolytope(u))
        astate->setChart(c1);
    else if (c2->psiInverse(x, u), c2->inPolytope(u))
        astate->setChart(c2);
    else
    {
        AtlasChart *c = owningChart(x);
        if (!c)
            c = newChart(x);
        astate->setChart(c);
    }
}

bool ompl::base::AtlasStateSpace::hasSymmetricInterpolate() const
{
    return true;
}

void ompl::base::AtlasStateSpace::copyState(State *destination, const State *source) const
{
    StateType *adest = destination->as<StateType>();
    const StateType *asrc = source->as<StateType>();
    adest->copyFrom(asrc);
}

ompl::base::StateSamplerPtr ompl::base::AtlasStateSpace::allocDefaultStateSampler() const
{
    return StateSamplerPtr(new AtlasStateSampler(*this));
}

ompl::base::State *ompl::base::AtlasStateSpace::allocState() const
{
    return new StateType(n_);
}

void ompl::base::AtlasStateSpace::freeState(State *state) const
{
    StateType *const astate = state->as<StateType>();
    delete astate;
}

int ompl::base::AtlasStateSpace::estimateFrontierPercent() const
{
    int frontier = 0;
    for (AtlasChart *c : charts_)
        frontier += c->estimateIsFrontier();
    return (100 * frontier) / charts_.size();
}

void ompl::base::AtlasStateSpace::dumpMesh(std::ostream &out) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;
    std::vector<Eigen::VectorXd> vertices;
    int i = 0;
    for (AtlasChart *c : charts_)
    {
        // Write the vertices and the faces
        std::cout << "\rDumping chart " << i++ << std::flush;
        vertices.clear();
        c->toPolygon(vertices);
        std::stringstream poly;
        std::size_t fvcount = 0;
        for (Eigen::VectorXd &vert : vertices)
        {
            v << vert.transpose() << "\n";
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

void ompl::base::AtlasStateSpace::dumpGraph(const PlannerData::Graph &graph, std::ostream &out, const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;

    BGL_FORALL_EDGES(edge, graph, PlannerData::Graph)
    {
        std::vector<StateType *> stateList;
        const State *const source = boost::get(vertex_type, graph, boost::source(edge, graph))->getState();
        const State *const target = boost::get(vertex_type, graph, boost::target(edge, graph))->getState();

        if (!asIs)
            traverseManifold(source->as<StateType>(), target->as<StateType>(), true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            v << target->as<StateType>()->constVectorView().transpose() << "\n";
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount - 3 << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            for (StateType *state : stateList)
                freeState(state);
            continue;
        }
        StateType *to, *from = stateList[0];
        v << from->constVectorView().transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i];
            from = stateList[i - 1];
            v << to->constVectorView().transpose() << "\n";
            v << from->constVectorView().transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount - 3 : vcount - 4) << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            freeState(stateList[i - 1]);
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

void ompl::base::AtlasStateSpace::dumpPath(ompl::geometric::PathGeometric &path, std::ostream &out,
                                           const bool asIs) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;

    const std::vector<State *> &waypoints = path.getStates();
    for (std::size_t i = 0; i < waypoints.size() - 1; i++)
    {
        std::vector<StateType *> stateList;
        const State *const source = waypoints[i];
        const State *const target = waypoints[i + 1];

        if (!asIs)
            traverseManifold(source->as<StateType>(), target->as<StateType>(), true, &stateList);
        if (asIs || stateList.size() == 1)
        {
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            v << target->as<StateType>()->constVectorView().transpose() << "\n";
            v << source->as<StateType>()->constVectorView().transpose() << "\n";
            vcount += 3;
            f << 3 << " " << vcount - 3 << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            for (StateType *state : stateList)
                freeState(state);
            continue;
        }
        StateType *to, *from = stateList[0];
        v << from->constVectorView().transpose() << "\n";
        vcount++;
        bool reset = true;
        for (std::size_t i = 1; i < stateList.size(); i++)
        {
            to = stateList[i];
            from = stateList[i - 1];
            v << to->constVectorView().transpose() << "\n";
            v << from->constVectorView().transpose() << "\n";
            vcount += 2;
            f << 3 << " " << (reset ? vcount - 3 : vcount - 4) << " " << vcount - 2 << " " << vcount - 1 << "\n";
            fcount++;
            freeState(stateList[i - 1]);
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
