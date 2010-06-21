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


#include "ompl/base/General.h"
#include "ompl/base/State.h"
#include "ompl/base/Path.h"
#include "ompl/base/Goal.h"
#include "ompl/base/GoalRegion.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/base/GoalState.h"
#include "ompl/base/GoalStates.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/ProblemDefinition.h"
#include "ompl/base/StateAllocator.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/OrthogonalProjectionEvaluator.h"
#include "ompl/base/LinearProjectionEvaluator.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/StateDistanceEvaluator.h"
#include "ompl/base/L2SquareStateDistanceEvaluator.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/UniformStateSampler.h"

#include "ompl/util/RandomNumbers.h"
#include "ompl/util/Console.h"
#include "ompl/util/Time.h"

#include "ompl/datastructures/BinaryHeap.h"
#include "ompl/datastructures/Grid.h"
#include "ompl/datastructures/GridN.h"
#include "ompl/datastructures/GridB.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/datastructures/NearestNeighborsSqrtApprox.h"

#include "ompl/kinematic/PathKinematic.h"
#include "ompl/kinematic/PathSimplifierKinematic.h"
#include "ompl/kinematic/SpaceInformationKinematic.h"
#include "ompl/kinematic/StateInterpolatorKinematic.h"
#include "ompl/kinematic/LinearStateInterpolatorKinematic.h"

#include "ompl/kinematic/planners/ik/HCIK.h"
#include "ompl/kinematic/planners/ik/GAIK.h"
#include "ompl/kinematic/planners/ik/IKPlanner.h"

#include "ompl/kinematic/planners/rrt/RRT.h"
#include "ompl/kinematic/planners/rrt/LazyRRT.h"
#include "ompl/kinematic/planners/rrt/RRTConnect.h"
#include "ompl/kinematic/planners/rrt/IKRRT.h"
#include "ompl/kinematic/planners/rrt/IKLRRT.h"
#include "ompl/kinematic/planners/rrt/pRRT.h"

#include "ompl/kinematic/planners/est/EST.h"
#include "ompl/kinematic/planners/est/IKEST.h"

#include "ompl/kinematic/planners/kpiece/KPIECE1.h"
#include "ompl/kinematic/planners/kpiece/LBKPIECE1.h"
#include "ompl/kinematic/planners/kpiece/IKKPIECE1.h"
#include "ompl/kinematic/planners/kpiece/IKLBKPIECE1.h"

#include "ompl/kinematic/planners/sbl/SBL.h"
#include "ompl/kinematic/planners/sbl/IKSBL.h"
#include "ompl/kinematic/planners/sbl/pSBL.h"

#include "ompl/kinematic/SimpleSetup.h"


#include "ompl/dynamic/Control.h"
#include "ompl/dynamic/ControlAllocator.h"
#include "ompl/dynamic/ControlSampler.h"
#include "ompl/dynamic/UniformControlSampler.h"
#include "ompl/dynamic/StateForwardPropagator.h"
#include "ompl/dynamic/PathDynamic.h"
#include "ompl/dynamic/SpaceInformationControls.h"
#include "ompl/dynamic/SpaceInformationControlsIntegrator.h"
#include "ompl/dynamic/SpaceInformationControlsPhysics.h"

#include "ompl/dynamic/planners/rrt/RRT.h"
#include "ompl/dynamic/planners/kpiece/KPIECE1.h"

/* Just make sure everything compiles */
TEST(Compile, All)
{    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
