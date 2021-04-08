/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include <ompl/base/SpaceInformation.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/range/irange.hpp>
#include <boost/range/algorithm_ext/push_back.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>

using namespace ompl::base;

const double edgeWidth = 0.1;  // original STRIDE paper had edgewidth = 0.1

std::vector<int> getHypercubeAdmissibleProjection(int dim)
{
    std::vector<int> discrete;
    boost::push_back(discrete, boost::irange(2, dim + 1));
    return discrete;
}

// Only states near some edges of a hypercube are valid. The valid edges form a
// narrow passage from (0,...,0) to (1,...,1). A state s is valid if there exists
// a k s.t. (a) 0<=s[k]<=1, (b) for all i<k s[i]<=edgeWidth, and (c) for all i>k
// s[i]>=1-edgewidth.
class HyperCubeValidityChecker : public StateValidityChecker
{
public:
    HyperCubeValidityChecker(const SpaceInformationPtr &si, int dimension)
      : StateValidityChecker(si), dimension_(dimension)
    {
        si->setStateValidityCheckingResolution(0.001);
    }

    bool isValid(const State *state) const override
    {
        const auto *s = state->as<RealVectorStateSpace::StateType>();
        bool foundMaxDim = false;

        for (int i = dimension_ - 1; i >= 0; i--)
            if (!foundMaxDim)
            {
                if ((*s)[i] > edgeWidth)
                    foundMaxDim = true;
            }
            else if ((*s)[i] < (1. - edgeWidth))
                return false;
        return true;
    }

protected:
    int dimension_;
};

template <typename T>
PlannerPtr GetMultiLevelPlanner(std::vector<int> sequenceLinks, SpaceInformationPtr si, std::string name = "Planner")
{
    std::vector<SpaceInformationPtr> si_vec;

    for (unsigned int k = 0; k < sequenceLinks.size() - 1; k++)
    {
        int links = sequenceLinks.at(k);

        auto spaceK(std::make_shared<ompl::base::RealVectorStateSpace>(links));
        ompl::base::RealVectorBounds bounds(links);
        bounds.setLow(0.);
        bounds.setHigh(1.);
        spaceK->setBounds(bounds);

        auto siK = std::make_shared<SpaceInformation>(spaceK);
        siK->setStateValidityChecker(std::make_shared<HyperCubeValidityChecker>(siK, links));

        spaceK->setup();
        si_vec.push_back(siK);
    }
    si_vec.push_back(si);

    auto planner = std::make_shared<T>(si_vec, name);

    return planner;
}
