
/* Author: Bryce Willey */

#define BOOST_TEST_MODULE "JointCost"
#include <boost/test/unit_test.hpp>
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/planners/trajopt/TrajOpt.h"
#include "ompl/base/objectives/JointDistanceObjective.h"
#include "ompl/base/objectives/ConvexifiableOptimization.h"
#include "ompl/base/objectives/ObstacleConstraint.h"
#include "ompl/base/objectives/CollisionEvaluator.h"

using namespace ompl;

std::vector<double> straightLine(int waypoints, int dof, double *start, double *end)
{
    std::vector<double> x;
    for (int i = 0; i < waypoints; i++)
    {
        for (int j = 0; j < dof; j++) 
        {
            x.push_back(start[j] + (end[j] - start[j]) * i / (waypoints - 1));       
        }
    }
    return x;
}

std::vector<double> random(int waypoints, int dof)
{
    std::vector<double> x;
    for (int i = 0; i < waypoints; i++)
    {
        for (int j = 0; j < dof; j++)
        {
            x.push_back(((double)rand()/RAND_MAX) * 5.0);
        }
    }
    return x;
}

void test(int WAYPOINTS, int DOF, std::vector<double> x)
{
    base::StateSpacePtr ss = std::make_shared<base::RealVectorStateSpace>(DOF);
    base::SpaceInformationPtr si = std::make_shared<base::SpaceInformation>(ss);

    base::JointDistanceObjective jointObj(si);
    auto optProb = std::make_shared<geometric::OmplOptProb>(WAYPOINTS, si);
    sco::CostPtr cost = jointObj.toCost(optProb);
    double res = cost->value(x);
    std::cout << res << std::endl;
}

BOOST_AUTO_TEST_CASE(joint_cost)
{
    double start[] = {0, 0, 0, 0, 0, 0};
    double end[] = {1.0, 2.0, 4.0, 3.0, 1.0, 2.0};
    auto x = straightLine(10, 6, start, end);
    test(10, 6, x);

    double realEnd[] = {-1.0, -0.5, 0, 2, 2.0, 3.0};
    auto y = straightLine(10, 6, end, realEnd);
    x.insert(x.end(), y.begin(), y.end());
    test(20, 6, x);

    srand(10);
    for (int i = 0; i < 10; i++)
    {
        x = random(10, 6);
        test(10, 6, x);
    }
}