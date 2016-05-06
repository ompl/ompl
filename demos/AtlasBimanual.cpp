/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Caleb Voss */

#include "AtlasCommon.h"

#include "ompl/base/spaces/RealVectorStateProjections.h"

struct real3
{
    double x;
    double y;
    double z;
};

void scale (double s, const real3 &v, real3 &out) {
    out.x = s * v.x;
    out.y = s * v.y;
    out.z = s * v.z;
}

void add (const real3 &a, const real3 &b, real3 &out) {
    out.x = a.x + b.x;
    out.y = a.y + b.y;
    out.z = a.z + b.z;
}

void cross (const real3 &a, const real3 &b, real3 &out) {
    out.x = a.y*b.z - a.z*b.y;
    out.y = a.z*b.x - a.x*b.z;
    out.z = a.x*b.y - a.y*b.x;
}

double dot (const real3 &a, const real3 &b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

struct quat
{
    double w;
    real3 v;
};

double sqnorm (const quat &q) {
    return q.w*q.w + q.v.x*q.v.x + q.v.y*q.v.y + q.v.z*q.v.z;
}

void scale (double s, const quat &q, quat &out) {
    scale(s, q.v, out.v);
    out.w = s * q.w;
}

void normalize (const quat &q, quat &out) {
    scale(1.0/std::sqrt(sqnorm(q)), q, out);
}

void add (const quat &a, const quat &b, quat &out) {
    out.w = a.w + b.w;
    add(a.v, b.v, out.v);
}

void multiply (const quat &a, const quat &b, quat &out) {
    real3 u = a.v;
    real3 v = b.v;
    double d = dot(u, v);
    real3 c;
    cross(u, v, c);
    scale(b.w, u, u);
    scale(a.w, v, v);
    add(u, v, out.v);
    add(out.v, c, out.v);
    out.w = a.w * b.w - d;
}

void conjugate (const quat &q, quat &out) {
    scale(-1, q.v, out.v);
    out.w = q.w;
}

void invert (const quat &q, quat &out) {
    conjugate(q, out);
    scale(1.0/sqnorm(out), out, out);
}

void rotate (const real3 &v, const quat &r, real3 &out) {
    quat rinv;
    invert(r, rinv);
    quat tmp = { 0, v };
    multiply(tmp, rinv, tmp);
    multiply(r, tmp, tmp);
    out = tmp.v;
}

void axis_angle (const real3 &ax, double th, quat &out) {
    out.w = std::cos(th/2);
    scale(std::sin(th/2), ax, out.v);
    normalize(out, out);
}

const real3 X = {1, 0, 0};
const real3 Y = {0, 1, 0};
const real3 Z = {0, 0, 1};
const real3 O = {0, 0, 0};

/** Bimanual manifold. */
class BimanualManifold : public ompl::base::AtlasStateSpace
{
public:
    
    mutable real3 pencil, forearm, upperarm;
    const int extras;

    BimanualManifold (int e = 1)
        : ompl::base::AtlasStateSpace(14, 14 - e), extras(e)
    {
        // Compute common link transformations.
        scale(0.2, Z, pencil);
        forearm = Z;
        upperarm = Z;
    }

    /* 0 to compute location of left arm's object. 1 for right arm. */
    void compute_object (const Eigen::VectorXd &x, int which_arm, Eigen::VectorXd &out) const
    {
        // Compute joint orientations.
        quat wrist1, wrist2, wrist3, elbow, shoulder1, shoulder2, shoulder3;
        unsigned int i = 7 * which_arm;
        axis_angle(X, x[i+0], wrist1);
        axis_angle(Y, x[i+1], wrist2);
        axis_angle(Z, x[i+2], wrist3);
        axis_angle(X, x[i+3], elbow);
        axis_angle(X, x[i+4], shoulder1);
        axis_angle(Y, x[i+5], shoulder2);
        axis_angle(Z, x[i+6], shoulder3);

        // Compute arm-dependent link transformation.
        real3 base; 
        scale(0.5 * (which_arm ? -1 : 1), X, base);

        // Compute location of object.
        real3 obj = O;
        add(obj, pencil, obj);
        rotate(obj, wrist1, obj);
        rotate(obj, wrist2, obj);
        rotate(obj, wrist3, obj);
        add(obj, forearm, obj);
        rotate(obj, elbow, obj);
        add(obj, upperarm, obj);
        rotate(obj, shoulder1, obj);
        rotate(obj, shoulder2, obj);
        rotate(obj, shoulder3, obj);
        add(obj, base, obj);
        out[0] = obj.x;
        out[1] = obj.y;
        out[2] = obj.z;
    }

    // We will let bigJ be computed numerically instead of writing it out.
    void bigF (const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out) const
    {
        // Compute left and right object positions.
        Eigen::VectorXd left(3), right(3);
        compute_object(x, 0, left);
        compute_object(x, 1, right);

        // Compute left and right wrist positions.
        Eigen::VectorXd wrist_left(3), wrist_right(3);
        real3 pencil_tmp = pencil;
        pencil = O;
        compute_object(x, 0, wrist_left);
        compute_object(x, 1, wrist_right);
        pencil = pencil_tmp;

        // Touches two points on a tray: Fixed distance apart.
        if (extras > 0)
            out[0] = (left - right).norm() - 1;

        // Don't let things slip off sideways: 0 distance in y-direction.
        if (extras > 1)
            out[1] = left[1] - right[1];

        // Level tray: wrist-to-tray has 0 distance in y-direction.
        if (extras > 2)
            out[2] = left[1] - wrist_left[1];

    }

    bool isValid (const ompl::base::State *)
    {
        return true;
    }

};

/** Print usage information. Does not return. */
void usage (const char *const progname)
{
    std::cout << "Usage: " << progname << " <planner> <timelimit> <extras>\n";
    printPlanners();
    std::exit(-1);
}

int main (int argc, char **argv)
{
    if (argc != 4)
        usage(argv[0]);

    int extras = std::atoi(argv[3]);
    if (extras <= 0 || extras > 4) {
        std::cout << "extras must be in [1, 4]\n";
        std::exit(-1);
    }
    ompl::base::AtlasStateSpacePtr atlas(new BimanualManifold(std::atoi(argv[3])));
    BimanualManifold *biman = (BimanualManifold *) atlas.get();
    ompl::base::StateValidityCheckerFn isValid = std::bind(&BimanualManifold::isValid, biman, std::placeholders::_1);
    biman->setBounds(-M_PI, M_PI);
    Eigen::VectorXd x(14); x << 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd y(14); y << 0, -1.2, 0, 2.2, 0, -1, 0,
                                0, -1.2, 0, 2.2, 0, -1, 0;
    Eigen::VectorXd sl(3), sr(3), gl(3), gr(3);
    biman->project(x);
    biman->project(y);
    biman->compute_object(x, 0, sl);
    biman->compute_object(x, 1, sr);
    biman->compute_object(y, 0, gl);
    biman->compute_object(y, 1, gr);
    std::cout << "Start effectors: " << sl.transpose() << "; " << sr.transpose() << "\n";
    std::cout << "Goal effectors: " << gl.transpose() << "; " << gr.transpose() << "\n";
    Eigen::VectorXd f(biman->getAmbientDimension() - biman->getManifoldDimension());
    biman->bigF(x, f);
    if (f.norm() > 0.01)
        std::cout << "Warning: start state off manifold! " << f.transpose() << "\n";
    biman->bigF(y, f);
    if (f.norm() > 0.01)
        std::cout << "Warning: goal state off manifold! " << f.transpose() << "\n";

    // These two planners get special treatment. The atlas will pretend to be RealVectorStateSpace
    // for them.
    bool cons = false;
    if (std::strcmp(argv[1], "ConstrainedRRT") == 0 || std::strcmp(argv[1], "CBiRRT2") == 0)
        cons = true;
    if (cons)
        atlas->stopBeingAnAtlas(true);
    
    // All the 'Constrained' classes are loose wrappers for the normal classes. No effect except on
    // the two special planners.
    ompl::geometric::ConstrainedSimpleSetup ss(atlas);
    ompl::base::ConstrainedSpaceInformationPtr si = ss.getConstrainedSpaceInformation();
    atlas->setSpaceInformation(si);
    ss.setStateValidityChecker(isValid);
    si->setValidStateSamplerAllocator(std::bind(vssa, atlas, std::placeholders::_1));
    ompl::base::ConstraintInformationPtr ci(new ompl::base::ConstraintInformation);
    ompl::base::ConstraintPtr c(new ompl::base::AtlasConstraint(atlas));
    ci->addConstraint(c);
    si->setConstraintInformation(ci);
    
    // Atlas parameters
    atlas->setExploration(0.5);
    atlas->setRho(0.5);
    atlas->setAlpha(M_PI/8);
    atlas->setEpsilon(0.2);
    atlas->setDelta(0.02);
    atlas->setMaxChartsPerExtension(200);
    atlas->setProjectionTolerance(1e-8);
    
    // The atlas needs some place to start sampling from. We will make start and goal charts.
    ompl::base::AtlasChart &startChart = atlas->anchorChart(x);
    ompl::base::AtlasChart &goalChart = atlas->anchorChart(y);
    ompl::base::ScopedState<> start(atlas);
    ompl::base::ScopedState<> goal(atlas);
    start->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(x, &startChart);
    goal->as<ompl::base::AtlasStateSpace::StateType>()->setRealState(y, &goalChart);
    ss.setStartAndGoalStates(start, goal);
    
    // Bounds
    ompl::base::RealVectorBounds bounds(atlas->getAmbientDimension());
    bounds.setLow(-10);
    bounds.setHigh(10);
    atlas->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    
    // Choose the planner.
    ompl::base::PlannerPtr planner(parsePlanner(argv[1], si, atlas->getRho_s()));
    if (!planner)
        usage(argv[0]);
    if (std::strcmp(argv[1], "KPIECE1") == 0) {
        planner->as<ompl::geometric::KPIECE1>()->setProjectionEvaluator(ompl::base::ProjectionEvaluatorPtr(new ompl::base::RealVectorRandomLinearProjectionEvaluator(atlas, atlas->getManifoldDimension())));
    } else if (std::strcmp(argv[1], "EST") == 0) {
        planner->as<ompl::geometric::EST>()->setProjectionEvaluator(ompl::base::ProjectionEvaluatorPtr(new ompl::base::RealVectorRandomLinearProjectionEvaluator(atlas, atlas->getManifoldDimension())));

    } else if (std::strcmp(argv[1], "STRIDE") == 0) {
        planner->as<ompl::geometric::STRIDE>()->setEstimatedDimension(atlas->getManifoldDimension());
    }
    ss.setPlanner(planner);
    ss.setup();
    
    // Set the time limit
    const double runtime_limit = std::atof(argv[2]);
    if (runtime_limit <= 0)
        usage(argv[0]);
    
    // Plan. For 3D problems, we save the chart mesh, planner graph, and solution path in the .ply format.
    // Regardless of dimension, we write the doubles in the path states to a .txt file.
    std::clock_t tstart = std::clock();
    ompl::base::PlannerStatus stat = planner->solve(runtime_limit);
    const double time = ((double)(std::clock()-tstart))/CLOCKS_PER_SEC;
    if (stat)
    {
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        if (x.size() == 3)
        {
            std::ofstream pathFile("path.ply");
            atlas->dumpPath(path, pathFile, cons);
            pathFile.close();
        }
        
        // Extract the full solution path by re-interpolating between the saved states (except for the special planners)
        const std::vector<ompl::base::State *> &waypoints = path.getStates();
        double length = 0;
        if (cons)
        {
            std::ofstream animFile("anim.txt");
            for (std::size_t i = 0; i < waypoints.size(); i++)
            {
                //std::cout << "[" << waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView().transpose() << "]\n";
                animFile << waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>()->constVectorView().transpose() << "\n";
            }
            animFile.close();
            length = path.length();
        }
        else
        {
            std::ofstream animFile("anim.txt");
            for (std::size_t i = 0; i < waypoints.size()-1; i++)
            {
                // Denote that we are switching to the next saved state
                //std::cout << "-----\n";
                ompl::base::AtlasStateSpace::StateType *from, *to;
                from = waypoints[i]->as<ompl::base::AtlasStateSpace::StateType>();
                to = waypoints[i+1]->as<ompl::base::AtlasStateSpace::StateType>();
                
                // Traverse the manifold
                std::vector<ompl::base::AtlasStateSpace::StateType *> stateList;
                atlas->followManifold(from, to, true, &stateList);
                if (atlas->equalStates(stateList.front(), stateList.back()))
                {
                    //std::cout << "[" << stateList.front()->constVectorView().transpose() << "]  " << stateList.front()->getChart()->getID() << "\n";
                    animFile << stateList.front()->constVectorView().transpose() << "\n";
                }
                else
                {
                    // Print the intermediate states
                    for (std::size_t i = 1; i < stateList.size(); i++)
                    {
                        //std::cout << "[" << stateList[i]->constVectorView().transpose() << "]  " << stateList[i]->getChart()->getID() << "\n";
                        animFile << stateList[i]->constVectorView().transpose() << "\n";
                        length += atlas->distance(stateList[i-1], stateList[i]);
                    }
                }
                
                // Delete the intermediate states
                for (std::size_t i = 0; i < stateList.size(); i++)
                    atlas->freeState(stateList[i]);
            }
            animFile.close();
            //std::cout << "-----\n";
        }
        if (stat == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
            std::cout << "Solution is approximate.\n";
        std::cout << "Length: " << length << "\n";
    }
    else
    {
        std::cout << "No solution found.\n";
    }
    std::cout << "Took " << time << " seconds.\n";
    
    ompl::base::PlannerData data(si);
    planner->getPlannerData(data);
    if (data.properties.find("approx goal distance REAL") != data.properties.end())
        std::cout << "Approx goal distance: " << data.properties["approx goal distance REAL"] << "\n";

    if (!cons)
        std::cout << "Atlas created " << atlas->getChartCount() << " charts.\n";
    
    if (x.size() == 3)
    {
        if (!cons)
        {
            std::ofstream atlasFile("atlas.ply");
            atlas->dumpMesh(atlasFile);
            atlasFile.close();
        }
        
        std::ofstream graphFile("graph.ply");
        ompl::base::PlannerData pd(si);
        planner->getPlannerData(pd);
        atlas->dumpGraph(pd.toBoostGraph(), graphFile, cons);
        graphFile.close();
    }
    
    return 0;
}
