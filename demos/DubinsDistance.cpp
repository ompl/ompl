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

/* Author: Mark Moll */

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;

int main(int argc, char* argv[])
{
    const unsigned int num_pts = 200;
    ob::StateSpacePtr space(new ob::DubinsStateSpace);
    ob::ScopedState<> from(space), to(space), s(space);
    std::vector<double> reals;

    from[0] = from[1] = from[2] = 0.;

    // if 3 command line arguments are given, it will interpret them as
    // an SE(2) state, and print interpolated points along the Dubins path
    // from (0,0,0) to the input state.
    if (argc == 4)
    {
        to[0] = atof(argv[1]);
        to[1] = atof(argv[2]);
        to[2] = atof(argv[3]);

        std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";
        for (unsigned int i=0; i<=num_pts; ++i)
        {
            space->interpolate(from(), to(), (double)i/num_pts, s());
            reals = s.reals();
            std::cout << reals[0] << ' ' << reals[1] << ' ' << reals[2] << std::endl;
        }
    }
    else
    // Otherwise, print the Dubins distance for (x,y,theta) for all points
    // in a 3D grid in SE(2) over (-5,5] x (-5, 5] x (-pi,pi].
        for (unsigned int i=0; i<num_pts; ++i)
            for (unsigned int j=0; j<num_pts; ++j)
                for (unsigned int k=0; k<num_pts; ++k)
                {
                    to[0] = 5. * (1. - 2. * (double)j/num_pts);
                    to[1] = 5. * (1. - 2. * (double)i/num_pts);
                    to[2] = boost::math::constants::pi<double>() * (1. - 2. * (double)k/num_pts);
                    std::cout << space->distance(from(), to()) << '\n';
                }
}