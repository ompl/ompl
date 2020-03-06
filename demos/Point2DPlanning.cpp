/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
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

/* Author: Ioan Sucan */

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRMstar.h>

#include <ompl/util/PPM.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Plane2DEnvironment
{
public:
    Plane2DEnvironment(const char *ppm_file, bool use_deterministic_sampling = false)
    {
        bool ok = false;
        useDeterministicSampling_ = use_deterministic_sampling;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch (ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            auto space(std::make_shared<ob::RealVectorStateSpace>());
            space->addDimension(0.0, ppm_.getWidth());
            space->addDimension(0.0, ppm_.getHeight());
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;
            ss_ = std::make_shared<og::SimpleSetup>(space);

            // set state validity checking for this space
            ss_->setStateValidityChecker([this](const ob::State *state) { return isStateValid(state); });
            space->setup();
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());

            // set the deterministic sampler
            // 2D space, no need to specify bases specifically
            if (useDeterministicSampling_)
            {
                // PRMstar can use the deterministic sampling
                ss_->setPlanner(std::make_shared<og::PRMstar>(ss_->getSpaceInformation()));
                space->setStateSamplerAllocator(std::bind(&Plane2DEnvironment::allocateHaltonStateSamplerRealVector,
                                                          this, std::placeholders::_1, 2,
                                                          std::vector<unsigned int>{2, 3}));
            }
        }
    }

    bool plan(unsigned int start_row, unsigned int start_col, unsigned int goal_row, unsigned int goal_col)
    {
        if (!ss_)
            return false;
        ob::ScopedState<> start(ss_->getStateSpace());
        start[0] = start_row;
        start[1] = start_col;
        ob::ScopedState<> goal(ss_->getStateSpace());
        goal[0] = goal_row;
        goal[1] = goal_col;
        ss_->setStartAndGoalStates(start, goal);
        // generate a few solutions; all will be added to the goal;
        for (int i = 0; i < 10; ++i)
        {
            if (ss_->getPlanner())
                ss_->getPlanner()->clear();
            ss_->solve();
        }
        const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
        OMPL_INFORM("Found %d solutions", (int)ns);
        if (ss_->haveSolutionPath())
        {
            if (!useDeterministicSampling_)
                ss_->simplifySolution();

            og::PathGeometric &p = ss_->getSolutionPath();
            if (!useDeterministicSampling_)
            {
                ss_->getPathSimplifier()->simplifyMax(p);
                ss_->getPathSimplifier()->smoothBSpline(p);
            }

            return true;
        }

        return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0; i < p.getStateCount(); ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
            const int h =
                std::min(maxHeight_, (int)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
    }

private:
    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    ob::StateSamplerPtr allocateHaltonStateSamplerRealVector(const ompl::base::StateSpace *space, unsigned int dim,
                                                             std::vector<unsigned int> bases = {})
    {
        // specify which deterministic sequence to use, here: HaltonSequence
        // optionally we can specify the bases used for generation (otherwise first dim prime numbers are used)
        if (bases.size() != 0)
            return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
                space, std::make_shared<ompl::base::HaltonSequence>(bases.size(), bases));
        else
            return std::make_shared<ompl::base::RealVectorDeterministicStateSampler>(
                space, std::make_shared<ompl::base::HaltonSequence>(dim));
    }

    og::SimpleSetupPtr ss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
    bool useDeterministicSampling_;
};

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    boost::filesystem::path path(TEST_RESOURCES_DIR);
    bool useDeterministicSampling = true;
    Plane2DEnvironment env((path / "ppm/floor.ppm").string().c_str(), useDeterministicSampling);

    if (env.plan(0, 0, 777, 1265))
    {
        env.recordSolution();
        env.save("result_demo.ppm");
    }

    return 0;
}
