/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metron, Inc.
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
 *   * Neither the name of the Metron, Inc. nor the names of its
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

/* Author: Mark Moll */

#include "ompl/base/spaces/OwenStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/tools/toms748_solve.hpp>

using namespace ompl::base;

namespace
{
    constexpr double twopi = 2. * boost::math::constants::pi<double>();

    // tolerance for boost root finding algorithm
    boost::math::tools::eps_tolerance<double> TOLERANCE(20);

    // max # iterations for searching for roots
    constexpr std::uintmax_t MAX_ITER = 32;
}  // namespace

OwenStateSpace::OwenStateSpace(double turningRadius, double maxPitch)
  : rho_(turningRadius), tanMaxPitch_(std::tan(maxPitch)), dubinsSpace_(turningRadius)
{
    setName("Owen" + getName());
    type_ = STATE_SPACE_OWEN;
    addSubspace(std::make_shared<RealVectorStateSpace>(3), 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 0.5);
    lock();
}

State *OwenStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void OwenStateSpace::registerProjections()
{
    class OwenDefaultProjection : public ProjectionEvaluator
    {
    public:
        OwenDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 3;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(3);
            bounds_ = space_->as<OwenStateSpace>()->getBounds();
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection = Eigen::Map<const Eigen::VectorXd>(
                state->as<OwenStateSpace::StateType>()->as<RealVectorStateSpace::StateType>(0)->values, 3);
        }
    };

    registerDefaultProjection(std::make_shared<OwenDefaultProjection>(this));
}

OwenStateSpace::OwenPath OwenStateSpace::owenPath(const State *state1, const State *state2) const
{
    auto s1 = state1->as<StateType>();
    auto s2 = state2->as<StateType>();
    auto path = dubinsSpace_.dubins(state1, state2, rho_);
    double dz = (*s2)[2] - (*s1)[2], len = rho_ * path.length();
    if (std::abs(dz) <= len * tanMaxPitch_)
    {
        // low altitude path
        //std::cout<<"low"<<std::endl;
        return {path, rho_, dz};
    }
    else if (std::abs(dz) > (len + twopi * rho_) * tanMaxPitch_)
    {
        // high altitude path
        //std::cout<<"high"<<std::endl;
        unsigned int k = std::floor((std::abs(dz)/tanMaxPitch_ - len) / (twopi * rho_));
        auto radius = rho_;
        auto radiusFun = [&, this](double r) {
            return (dubinsSpace_.dubins(state1, state2, r).length() + twopi * k) * r * tanMaxPitch_ - std::abs(dz);
        };
        std::uintmax_t iter = MAX_ITER;
        auto result = boost::math::tools::bracket_and_solve_root(radiusFun, radius, 2., true, TOLERANCE, iter);
        radius = .5 * (result.first + result.second);
        assert(std::abs(radiusFun(radius)) < 1e-5);
        path = dubinsSpace_.dubins(state1, state2, radius);        
        return {path, radius, dz, k};
    }

    // medium altitude path
    {
        //std::cout<<"medium"<<std::endl;
        auto zi = dubinsSpace_.allocState()->as<DubinsStateSpace::StateType>();
        auto phi = 0.;
        auto phiFun = [&, this](double phi) {
            turn(state1, rho_, phi, zi);
            return (phi + dubinsSpace_.dubins(zi, state2).length()) * rho_ * tanMaxPitch_ - std::abs(dz);
        };
        // for (int i=-10; i<=10; ++i)
        //     std::cout<< "i=" << i << " phi=" << (double)i * boost::math::constants::pi<double>() *.2 << " phifun="<<phiFun((double)i * boost::math::constants::pi<double>() *.2)<<std::endl;
        std::uintmax_t iter = MAX_ITER;
        auto result = boost::math::tools::toms748_solve(phiFun, -twopi, twopi, TOLERANCE, iter);
        phi = .5 * (result.first + result.second);
        assert(std::abs(phiFun(phi)) < 1e-5);
        turn(state1, rho_, phi, zi);
        path = dubinsSpace_.dubins(zi, state2, rho_);
        dubinsSpace_.freeState(zi);
        return {path, rho_, dz, phi};
    }
}

void OwenStateSpace::turn(const State* from, double turnRadius, double angle, State* state) const
{
    auto s0 = from->as<DubinsStateSpace::StateType>();
    auto s1 = state->as<DubinsStateSpace::StateType>();
    double theta = s0->getYaw(), phi = theta + angle, r = (angle > 0 ? turnRadius : -turnRadius);
    s1->setXY(s0->getX() + r * (std::sin(phi) - std::sin(theta)),
              s0->getY() + r * (-std::cos(phi) + std::cos(theta)));
    s1->setYaw(phi);
}

double OwenStateSpace::distance(const State *state1, const State *state2) const
{
    return owenPath(state1, state2).length();
}

unsigned int OwenStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return StateSpace::validSegmentCount(state1, state2);
}

void OwenStateSpace::interpolate(const State *from, const State *to, double t, State *state) const
{
    auto path = owenPath(from, to);
    interpolate(from, to, t, path, state);
}

void OwenStateSpace::interpolate(const State *from, const State *to, double t, OwenPath &path,
                                 State *state) const
{
    if (t >= 1.)
    {
        if (to != state)
            copyState(state, to);
        return;
    }
    if (t <= 0.)
    {
        if (from != state)
            copyState(state, from);
        return;
    }

    auto f = from->as<StateType>();
    auto s = state->as<StateType>();
    (*s)[2] = (*f)[2] + t * path.deltaZ_;
    if (path.phi_ == 0.)
    {
        if (path.numTurns_ == 0)
        {
            // low altitude path
            dubinsSpace_.interpolate(from, path.path_, t, state, path.turnRadius_);
        }
        else
        {
            // high altitude path
            auto lengthSpiral = twopi * path.turnRadius_ * path.numTurns_;
            auto lengthPath = path.turnRadius_ * path.path_.length();
            auto length = lengthSpiral + lengthPath, dist = t * length;
            if (dist > lengthSpiral)
                dubinsSpace_.interpolate(from, path.path_, (dist - lengthSpiral) / lengthPath, state, path.turnRadius_);
            else
                turn(from, path.turnRadius_, dist / path.turnRadius_, state);
        }
    }
    else
    {
        // medium altitude path
        auto lengthTurn = std::abs(path.phi_) * path.turnRadius_;
        auto lengthPath = path.turnRadius_ * path.path_.length();
        auto length = lengthTurn + lengthPath, dist = t * length;
        if (dist > lengthTurn)
        {
            State* s = (state == to) ? dubinsSpace_.allocState() : state;
            turn(from, path.turnRadius_, path.phi_, s);
            dubinsSpace_.interpolate(s, path.path_, (dist - lengthTurn) / lengthPath, state, path.turnRadius_);
            if (state == to) dubinsSpace_.freeState(s);
        }
        else
        {
            turn(from, path.turnRadius_, dist / path.turnRadius_, state);
        }
    }

    getSubspace(1)->enforceBounds(state->as<CompoundStateSpace::StateType>()->as<SO2StateSpace::StateType>(1));
}

double OwenStateSpace::OwenPath::length() const
{
    double hlen = turnRadius_ * (path_.length() + twopi * numTurns_ + phi_); 
    return std::sqrt(hlen * hlen + deltaZ_ * deltaZ_);
}

namespace ompl::base
{
    std::ostream &operator<<(std::ostream &os, const OwenStateSpace::OwenPath &path)
    {
        static const DubinsStateSpace dubinsStateSpace;

        os << "OwenPath[ length = " << path.length() << ", turnRadius=" << path.turnRadius_ << ", deltaZ=" << path.deltaZ_
           << ", phi=" << path.phi_ << ", numTurns=" << path.numTurns_ << ", path=" << path.path_;
        os << " ]";
        return os;
    }
}  // namespace ompl::base

void ompl::base::OwenMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<OwenStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::OwenMotionValidator::checkMotion(const State *s1, const State *s2,
                                                  std::pair<State *, double> &lastValid) const
{
    auto path = stateSpace_->owenPath(s1, s2);

    /* assume motion starts in a valid configuration so s1 is valid */
    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, path, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, path, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, path, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool ompl::base::OwenMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;
    auto path = stateSpace_->owenPath(s1, s2);

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, path, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
