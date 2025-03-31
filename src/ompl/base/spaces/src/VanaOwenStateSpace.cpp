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

#include "ompl/base/spaces/VanaOwenStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/tools/toms748_solve.hpp>

using namespace ompl::base;

namespace
{
    constexpr double onepi = boost::math::constants::pi<double>();
    constexpr double twopi = 2. * boost::math::constants::pi<double>();

    // tolerance for boost root finding algorithm
    boost::math::tools::eps_tolerance<double> TOLERANCE(10);

    // max # iterations for doubling turning radius for initial solution path
    constexpr unsigned int MAX_ITER = 64;

    void turn(const State *from, double turnRadius, double angle, State *state)
    {
        auto s0 = from->as<DubinsStateSpace::StateType>();
        auto s1 = state->as<DubinsStateSpace::StateType>();
        double theta = s0->getYaw(), phi = theta + angle, r = (angle > 0 ? turnRadius : -turnRadius);
        s1->setXY(s0->getX() + r * (std::sin(phi) - std::sin(theta)),
                  s0->getY() + r * (-std::cos(phi) + std::cos(theta)));
        s1->setYaw(phi);
    }

}  // namespace

VanaOwenStateSpace::VanaOwenStateSpace(double turningRadius, double maxPitch)
  : VanaOwenStateSpace(turningRadius, {-maxPitch, maxPitch})
{
}
VanaOwenStateSpace::VanaOwenStateSpace(double turningRadius, std::pair<double, double> pitchRange)
  : rho_(turningRadius), minPitch_(pitchRange.first), maxPitch_(pitchRange.second), dubinsSpace_(turningRadius)
{
    setName("VanaOwen" + getName());
    type_ = STATE_SPACE_VANA_OWEN;
    auto space = std::make_shared<RealVectorStateSpace>(4);
    ompl::base::RealVectorBounds pitchBounds(4);
    pitchBounds.setLow(3, minPitch_);
    pitchBounds.setHigh(3, maxPitch_);
    space->setBounds(pitchBounds);
    addSubspace(space, 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 0.5);
    lock();
}

State *VanaOwenStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void VanaOwenStateSpace::registerProjections()
{
    class VanaOwenDefaultProjection : public ProjectionEvaluator
    {
    public:
        VanaOwenDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 3;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(3);
            bounds_ = space_->as<VanaOwenStateSpace>()->getBounds();
            bounds_.resize(3);
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection = Eigen::Map<const Eigen::VectorXd>(
                state->as<StateType>()->as<RealVectorStateSpace::StateType>(0)->values, 3);
        }
    };

    registerDefaultProjection(std::make_shared<VanaOwenDefaultProjection>(this));
}

DubinsStateSpace::StateType *VanaOwenStateSpace::get2DPose(double x, double y, double yaw) const
{
    auto state = dubinsSpace_.allocState()->as<DubinsStateSpace::StateType>();
    state->setXY(x, y);
    state->setYaw(yaw);
    return state;
}

bool VanaOwenStateSpace::isValid(DubinsStateSpace::DubinsPath const &path, StateType const *state) const
{
    // in three cases the result is invalid:
    // 1. path of type CCC (i.e., RLR or LRL)
    // 2. pitch smaller than minPitch_
    // 3. pitch greater than maxPitch_
    if ((path.type_->at(1) != DubinsStateSpace::DUBINS_STRAIGHT) ||
        (path.type_->at(0) == DubinsStateSpace::DUBINS_RIGHT && state->pitch() - path.length_[0] < minPitch_) ||
        (path.type_->at(0) == DubinsStateSpace::DUBINS_LEFT && state->pitch() + path.length_[0] > maxPitch_))
    {
        return false;
    }

    return true;
}

bool VanaOwenStateSpace::decoupled(const StateType *from, const StateType *to, double radius, PathType &result,
                                   std::array<DubinsStateSpace::StateType *, 3> &scratch) const
{
    result.horizontalRadius_ = radius;
    result.verticalRadius_ = 1. / std::sqrt(1. / (rho_ * rho_) - 1. / (radius * radius));
    result.deltaZ_ = (*to)[2] - (*from)[2];
    // note that we are exploiting properties of the memory layout of state types
    result.pathXY_ = dubinsSpace_.dubins(from, to, radius);
    result.phi_ = 0.;
    result.numTurns_ = 0;

    // can't change altitude if vertical turning radius is infinite, but that's ok if we don't need a vertical turn
    if (!std::isfinite(result.verticalRadius_))
    {
        if (std::abs(result.deltaZ_) < 1e-8 && std::abs(to->pitch() - from->pitch()) < 1e-8)
        {
            result.pathSZ_.type_ = &DubinsStateSpace::dubinsPathType[0]; // LSL type
            result.pathSZ_.length_[0] = result.pathSZ_.length_[2] = 0.;
            result.pathSZ_.length_[1] = result.deltaZ_;
            return true;
        }
        else
            return false;
    }

    double len = radius * result.pathXY_.length();
    auto startSZ = result.startSZ_, endSZ = scratch[0];

    startSZ->setXY(0., (*from)[2]);
    startSZ->setYaw(from->pitch());
    endSZ->setXY(len, (*to)[2]);
    endSZ->setYaw(to->pitch());
    result.pathSZ_ = dubinsSpace_.dubins(startSZ, endSZ, result.verticalRadius_);
    if (isValid(result.pathSZ_, from))
    {
        // low altitude path
        return true;
    }

    double pitch = (result.deltaZ_ < 0) ? minPitch_ : maxPitch_, tanPitch = tan(pitch);
    auto s1b = scratch[1], s2b = scratch[2];
    double turnS, turnZ;
    auto update = [&, this](double r, bool skipInit = false)
    {
        if (!skipInit)
        {
            result.horizontalRadius_ = r;
            result.verticalRadius_ = 1. / std::sqrt(1. / (rho_ * rho_) - 1. / (r * r));
            result.pathXY_ = dubinsSpace_.dubins(from, to, result.horizontalRadius_);
        }
        endSZ->setX((result.pathXY_.length() + twopi * result.numTurns_) * r);
        turn(startSZ, result.verticalRadius_, pitch - from->pitch(), s1b);
        assert(std::abs(s1b->getYaw() - pitch) < 1e-8);
        dubinsSpace_.copyState(s2b, endSZ);
        s2b->setYaw(s2b->getYaw() + onepi);
        turn(s2b, result.verticalRadius_, pitch - to->pitch(), s2b);
        assert(std::abs(s2b->getYaw() - pitch - onepi) < 1e-8);
        turnS = (s1b->getX() - startSZ->getX()) + (endSZ->getX() - s2b->getX());
        turnZ = (s1b->getY() - startSZ->getY()) + (endSZ->getY() - s2b->getY());
    };
    update(radius, true);

    double highAltitudeS = len + twopi * radius;
    double highAltitudeZ = turnZ + (highAltitudeS - turnS) * tanPitch;
    if (std::abs(result.deltaZ_) > std::abs(highAltitudeZ))
    {
        // high altitude path
        result.numTurns_ = std::floor((std::abs((result.deltaZ_ - turnZ) / tanPitch) - len + turnS) / (twopi * radius));
        auto radiusFun = [&, this](double r)
        {
            update(r);
            result.pathSZ_ = dubinsSpace_.dubins(startSZ, endSZ, result.verticalRadius_);
            return ((result.pathXY_.length() + twopi * result.numTurns_) * r - turnS) * tanPitch + turnZ -
                   result.deltaZ_;
        };
        std::uintmax_t iter = MAX_ITER;
        try
        {
            auto root = boost::math::tools::toms748_solve(radiusFun, radius, 3. * radius, TOLERANCE, iter);
            auto rval = radiusFun(.5 * (root.first + root.second));
            if (iter >= MAX_ITER)
                OMPL_WARN("Maximum number of iterations exceeded for high altitude Vana-Owen path");
            return std::abs(rval) < 1e-4;
        }
        catch (std::domain_error& e)
        {
            return false;
        }
    }
    else
    {
        // medium altitude path
        auto zi = scratch[1];
        auto phiFun = [&, this](double phi)
        {
            if (std::abs(phi)>twopi)
                throw std::domain_error("phi too large");
            turn(from, radius, phi, zi);
            return (std::abs(phi) + dubinsSpace_.dubins(zi, to).length()) * radius * std::abs(tanPitch) - std::abs(result.deltaZ_);
        };

        try
        {
            std::uintmax_t iter = MAX_ITER;
            result.phi_ = 0.1;
            auto root = boost::math::tools::bracket_and_solve_root(phiFun, result.phi_, 2., true, TOLERANCE, iter);
            result.phi_ = .5 * (root.first + root.second);
            if (std::abs(phiFun(result.phi_)) > .01)
                throw std::domain_error("fail");
        }
        catch (...)
        {
            try {
                std::uintmax_t iter = MAX_ITER;
                result.phi_ = -.1;
                auto root = boost::math::tools::bracket_and_solve_root(phiFun, result.phi_, 2., true, TOLERANCE, iter);
                result.phi_ = .5 * (root.first + root.second);
            }
            catch (...)
            {
                //OMPL_ERROR("this shouldn't be happening!");
                return false;
            }
        }
        result.pathXY_ = dubinsSpace_.dubins(zi, to, radius);
        endSZ->setX((result.pathXY_.length() + result.phi_) * radius);
        result.pathSZ_ = dubinsSpace_.dubins(startSZ, endSZ, result.verticalRadius_);
        return std::abs(phiFun(result.phi_)) < .01 && isValid(result.pathSZ_, from);
    }
    return true;
}

std::optional<VanaOwenStateSpace::PathType> VanaOwenStateSpace::getPath(const State *state1, const State *state2) const
{
    auto from = state1->as<StateType>(), to = state2->as<StateType>();
    double radiusMultiplier = 1.5;
    std::array<DubinsStateSpace::StateType *, 3> scratch;
    PathType path;

    unsigned int iter = 0;
    std::generate(scratch.begin(), scratch.end(),
                  [this]() { return dubinsSpace_.allocState()->as<DubinsStateSpace::StateType>(); });

    while (!decoupled(from, to, rho_ * radiusMultiplier, path, scratch) && iter++ < MAX_ITER)
    {
        radiusMultiplier *= 2.;
    }
    if (iter >= MAX_ITER)
    {
        OMPL_ERROR("Maximum number of iterations exceeded in VanaOwenStateSpace::PathType");
        for (auto s : scratch)
            dubinsSpace_.freeState(s);
        return {};
    }

    // Local optimalization between horizontal and vertical radii
    double step = .1, radiusMultiplier2, minLength = path.length(), length;
    PathType path2;
    while (std::abs(step) > tolerance_)
    {
        radiusMultiplier2 = std::max(1., radiusMultiplier + step);
        if (decoupled(from, to, rho_ * radiusMultiplier2, path2, scratch) && (length = path2.length()) < minLength)
        {
            radiusMultiplier = radiusMultiplier2;
            path = path2;
            minLength = length;
            step *= 2.;
        }
        else
            step *= -.1;
    }

    for (auto s : scratch)
        dubinsSpace_.freeState(s);
    return path;
}

double VanaOwenStateSpace::distance(const State *state1, const State *state2) const
{
    if (auto path = getPath(state1, state2))
        return path->length();
    return getMaximumExtent();
}

unsigned int VanaOwenStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return StateSpace::validSegmentCount(state1, state2);
}

void VanaOwenStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    if (auto path = getPath(from, to))
        interpolate(from, to, t, *path, state);
    else
        if (from != state)
            copyState(state, from);
}

void VanaOwenStateSpace::interpolate(const State *from, const State *to, const double t, PathType &path,
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

    auto intermediate = (from == state) ? dubinsSpace_.allocState() : state;
    auto s = state->as<StateType>();
    auto i = intermediate->as<DubinsStateSpace::StateType>();

    // This is exploiting internal properties of compound state spaces like DubinsStateSpace
    // and VanaOwenStateSpace
    dubinsSpace_.interpolate(path.startSZ_, path.pathSZ_, t, intermediate, path.verticalRadius_);
    (*s)[2] = i->getY();
    s->pitch() = i->getYaw();

    if (path.phi_ == 0.)
    {
        if (path.numTurns_ == 0)
        {
            // low altitude path
            dubinsSpace_.interpolate(from, path.pathXY_, t, state, path.horizontalRadius_);
        }
        else
        {
            // high altitude path
            auto lengthSpiral = twopi * path.horizontalRadius_ * path.numTurns_;
            auto lengthPath = path.horizontalRadius_ * path.pathXY_.length();
            auto length = lengthSpiral + lengthPath, dist = t * length;
            if (dist > lengthSpiral)
                dubinsSpace_.interpolate(from, path.pathXY_, (dist - lengthSpiral) / lengthPath, state,
                                         path.horizontalRadius_);
            else
                turn(from, path.horizontalRadius_, dist / path.horizontalRadius_, state);
        }
    }
    else
    {
        // medium altitude path
        auto lengthTurn = std::abs(path.phi_) * path.horizontalRadius_;
        auto lengthPath = path.horizontalRadius_ * path.pathXY_.length();
        auto length = lengthTurn + lengthPath, dist = t * length;
        if (dist > lengthTurn)
        {
            State *s = (state == to) ? dubinsSpace_.allocState() : state;
            turn(from, path.horizontalRadius_, path.phi_, s);
            dubinsSpace_.interpolate(s, path.pathXY_, (dist - lengthTurn) / lengthPath, state, path.horizontalRadius_);
            if (state == to)
                dubinsSpace_.freeState(s);
        }
        else
        {
            auto angle = dist / path.horizontalRadius_;
            if (path.phi_ < 0)
                angle = -angle;
            turn(from, path.horizontalRadius_, angle, state);
        }
    }

    if (from == state)
        dubinsSpace_.freeState(intermediate);
    getSubspace(1)->enforceBounds(state->as<CompoundStateSpace::StateType>()->as<SO2StateSpace::StateType>(1));
}

VanaOwenStateSpace::PathType::~PathType()
{
    static const DubinsStateSpace dubinsStateSpace_;
    dubinsStateSpace_.freeState(startSZ_);
}

VanaOwenStateSpace::PathType::PathType()
{
    static const DubinsStateSpace dubinsStateSpace_;
    startSZ_ = dubinsStateSpace_.allocState()->as<DubinsStateSpace::StateType>();
}

VanaOwenStateSpace::PathType::PathType(const VanaOwenStateSpace::PathType &path)
  : pathXY_(path.pathXY_)
  , pathSZ_(path.pathSZ_)
  , horizontalRadius_(path.horizontalRadius_)
  , verticalRadius_(path.verticalRadius_)
  , deltaZ_(path.deltaZ_)
  , phi_(path.phi_)
  , numTurns_(path.numTurns_)
{
    static const DubinsStateSpace dubinsStateSpace_;
    startSZ_ = dubinsStateSpace_.allocState()->as<DubinsStateSpace::StateType>();
    dubinsStateSpace_.copyState(startSZ_, path.startSZ_);
}

VanaOwenStateSpace::PathType &VanaOwenStateSpace::PathType::operator=(const VanaOwenStateSpace::PathType &path)
{
    static const DubinsStateSpace dubinsStateSpace_;
    pathXY_ = path.pathXY_;
    pathSZ_ = path.pathSZ_;
    horizontalRadius_ = path.horizontalRadius_;
    verticalRadius_ = path.verticalRadius_;
    deltaZ_ = path.deltaZ_;
    phi_ = path.phi_;
    numTurns_ = path.numTurns_;
    dubinsStateSpace_.copyState(startSZ_, path.startSZ_);
    return *this;
}

double VanaOwenStateSpace::PathType::length() const
{
    return verticalRadius_ * pathSZ_.length();
}

VanaOwenStateSpace::PathCategory VanaOwenStateSpace::PathType::category() const
{
    if (phi_ == 0.)
    {
        if (numTurns_ == 0)
            return PathCategory::LOW_ALTITUDE;
        else
            return PathCategory::HIGH_ALTITUDE;
    }
    else
    {
        if (numTurns_ == 0)
            return PathCategory::MEDIUM_ALTITUDE;
        else
            return PathCategory::UNKNOWN;
    }
}

namespace ompl::base
{
    std::ostream &operator<<(std::ostream &os, const VanaOwenStateSpace::PathType &path)
    {
        static const DubinsStateSpace dubinsStateSpace;

        os << "VanaOwenPath[ category = " << (char)path.category() << "\n\tlength = " << path.length()
           << "\n\tXY = " << path.pathXY_ << "\n\tSZ = " << path.pathSZ_ << "\n\trh = " << path.horizontalRadius_
           << "\n\trv = " << path.verticalRadius_ << "\n\tdeltaZ = " << path.deltaZ_ << "\n\tphi = " << path.phi_
           << "\n\tnumTurns = " << path.numTurns_ << "\n\tstartSZ = ";
        dubinsStateSpace.printState(path.startSZ_, os);
        os << "]";
        return os;
    }
}  // namespace ompl::base
