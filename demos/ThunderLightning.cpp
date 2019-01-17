/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
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

/* Author: Mark Moll, Dave Coleman, Ioan Sucan */

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/thunder/Thunder.h>
#include <ompl/tools/lightning/Lightning.h>
#include <ompl/util/PPM.h>

#include <ompl/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

class Plane2DEnvironment
{
public:

    Plane2DEnvironment(const char *ppm_file, bool useThunder = true)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
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
            if (useThunder)
            {
                expPlanner_ = std::make_shared<ot::Thunder>(space);
                expPlanner_->setFilePath("thunder.db");
            }
            else
            {
                expPlanner_ = std::make_shared<ot::Lightning>(space);
                expPlanner_->setFilePath("lightning.db");
            }
            // set state validity checking for this space
            expPlanner_->setStateValidityChecker([this](const ob::State *state)
                { return isStateValid(state); });
            space->setup();
            expPlanner_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / space->getMaximumExtent());
            vss_ = expPlanner_->getSpaceInformation()->allocValidStateSampler();

            // DTC
            //experience_setup_->setPlanner(std::make_shared<og::RRTConnect>(si_));
            // Set the repair planner
            // experience_setup_->setRepairPlanner(std::make_shared<og::RRTConnect>(si_));
        }
    }

    ~Plane2DEnvironment()
    {
        expPlanner_->save();
    }

    bool plan()
    {
        std::cout << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;
        std::cout << "-------------------------------------------------------" << std::endl;

        if (!expPlanner_)
        {
            OMPL_ERROR("Simple setup not loaded");
            return false;
        }
        expPlanner_->clear();

        ob::ScopedState<> start(expPlanner_->getStateSpace());
        vss_->sample(start.get());
        ob::ScopedState<> goal(expPlanner_->getStateSpace());
        vss_->sample(goal.get());
        expPlanner_->setStartAndGoalStates(start, goal);

        bool solved = expPlanner_->solve(10.);
        if (solved)
            OMPL_INFORM("Found solution in %g seconds",
                expPlanner_->getLastPlanComputationTime());
        else
            OMPL_INFORM("No solution found");

        expPlanner_->doPostProcessing();

        return false;
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[0], maxWidth_);
        const int h = std::min((int)state->as<ob::RealVectorStateSpace::StateType>()->values[1], maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    ot::ExperienceSetupPtr expPlanner_;
    ob::ValidStateSamplerPtr vss_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;
};

int main(int argc, char **)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    boost::filesystem::path path(TEST_RESOURCES_DIR);
    Plane2DEnvironment env((path / "ppm" / "floor.ppm").string().c_str(), argc==1);

    for (unsigned int i = 0; i < 100; ++i)
        env.plan();

    return 0;
}
