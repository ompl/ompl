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

#ifndef PY_BINDINGS_OMPL_PY_GEOMETRIC_
#define PY_BINDINGS_OMPL_PY_GEOMETRIC_

#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/quotientspace/QRRT.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"
#include <deque>
#include <map>
#include <boost/graph/adjacency_list.hpp>
#include "py_std_function.hpp"


namespace ompl
{
    namespace geometric
    {
        inline int dummyFn() { return 1; }
        inline int dummyConnectionStrategy()
        {
            NearestNeighborsLinear<PRM::Vertex> nn;
            std::shared_ptr<NearestNeighbors<PRM::Vertex> > nnPtr(&nn);
            return sizeof(KStrategy<PRM::Vertex>(1, nnPtr)) + sizeof(KStarStrategy<PRM::Vertex>(dummyFn, nnPtr, 1)) + sizeof(nn);
        }
        inline int dummySTLContainerSize()
        {
            return sizeof(std::deque<ompl::base::State*>) +
                sizeof(std::map<boost::adjacency_list<>::vertex_descriptor, ompl::base::State*>) +
                sizeof(std::vector<const ompl::base::State*>) +
                sizeof(std::vector< std::shared_ptr<ompl::geometric::BITstar::Vertex> >) +
                sizeof(std::vector< std::shared_ptr<ompl::base::SpaceInformation> >);
        }
        inline int dummyQRRTsize()
        {
            return sizeof(QRRT);
        }
    }
}

#endif
