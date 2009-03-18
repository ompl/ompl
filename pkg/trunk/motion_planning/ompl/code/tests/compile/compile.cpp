/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/** \author Ioan Sucan */

#include <gtest/gtest.h>

#include "ompl/base/util/random_utils.h"
#include "ompl/base/util/time.h"
#include "ompl/base/util/output.h"

#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include "ompl/base/Control.h"
#include "ompl/base/Path.h"
#include "ompl/base/Goal.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateForwardPropagator.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/StateDistanceEvaluator.h"

#include "ompl/datastructures/Hash.h"
#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/datastructures/GridN.h"
#include "ompl/datastructures/GridB.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"

#include "ompl/extension/samplingbased/State.h"
#include "ompl/extension/samplingbased/Control.h"
#include "ompl/extension/samplingbased/Goal.h"
#include "ompl/extension/samplingbased/Planner.h"
#include "ompl/extension/samplingbased/SpaceInformation.h"
#include "ompl/extension/samplingbased/StateAllocator.h"
#include "ompl/extension/samplingbased/ProjectionEvaluator.h"
#include "ompl/extension/samplingbased/StateDistanceEvaluator.h"

#include "ompl/extension/samplingbased/kinematic/PathKinematic.h"
#include "ompl/extension/samplingbased/kinematic/PathSmootherKinematic.h"
#include "ompl/extension/samplingbased/kinematic/SpaceInformationKinematic.h"

#include "ompl/extension/samplingbased/kinematic/extension/ik/HCIK.h"
#include "ompl/extension/samplingbased/kinematic/extension/ik/GAIK.h"
#include "ompl/extension/samplingbased/kinematic/extension/ik/IKPlanner.h"

#include "ompl/extension/samplingbased/kinematic/extension/rrt/RRT.h"
#include "ompl/extension/samplingbased/kinematic/extension/rrt/LazyRRT.h"
#include "ompl/extension/samplingbased/kinematic/extension/rrt/IKRRT.h"
#include "ompl/extension/samplingbased/kinematic/extension/rrt/IKLRRT.h"

#include "ompl/extension/samplingbased/kinematic/extension/est/EST.h"
#include "ompl/extension/samplingbased/kinematic/extension/est/IKEST.h"

#include "ompl/extension/samplingbased/kinematic/extension/kpiece/KPIECE1.h"
#include "ompl/extension/samplingbased/kinematic/extension/kpiece/LBKPIECE1.h"
#include "ompl/extension/samplingbased/kinematic/extension/kpiece/IKKPIECE1.h"
#include "ompl/extension/samplingbased/kinematic/extension/kpiece/IKLBKPIECE1.h"

#include "ompl/extension/samplingbased/kinematic/extension/sbl/SBL.h"
#include "ompl/extension/samplingbased/kinematic/extension/sbl/IKSBL.h"

#include "ompl/extension/samplingbased/kinodynamic/SpaceInformationControls.h"
#include "ompl/extension/samplingbased/kinodynamic/SpaceInformationControlsIntegrator.h"
#include "ompl/extension/samplingbased/kinodynamic/SpaceInformationControlsPhysics.h"

/* Just make sure everything compiles */
TEST(Compile, All)
{    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
