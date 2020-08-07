/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
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

#include <ompl/multilevel/datastructures/metrics/ShortestPath.h>
#include <ompl/geometric/PathGeometric.h>

using namespace ompl::multilevel;
using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

BundleSpaceMetricShortestPath::BundleSpaceMetricShortestPath(BundleSpaceGraph *bundleSpaceGraph)
  : BaseT(bundleSpaceGraph)
{
    if (bundleSpaceGraph_->hasBaseSpace())
    {
        const base::SpaceInformationPtr &base = bundleSpaceGraph_->getBase();
        xBaseStart_ = new Configuration(base);
        xBaseDest_ = new Configuration(base);
    }
}
BundleSpaceMetricShortestPath::~BundleSpaceMetricShortestPath()
{
    if (bundleSpaceGraph_->hasBaseSpace())
    {
        delete xBaseStart_;
        delete xBaseDest_;
        for (uint k = 0; k < tmpPath_.size(); k++)
        {
            Configuration *qk = new Configuration(bundleSpaceGraph_->getBundle());
            tmpPath_.push_back(qk);
        }
        for (auto q : tmpPath_)
        {
            delete q;
        }
        tmpPath_.clear();
    }
}

double BundleSpaceMetricShortestPath::distanceBundle(const Configuration *xStart, const Configuration *xDest)
{
    if (bundleSpaceGraph_->getBaseDimension() <= 0)
    {
        return BaseT::distanceBundle(xStart, xDest);
    }
    else
    {
        std::vector<const Configuration *> path = getInterpolationPath(xStart, xDest);
        double d = 0.0;
        for (uint k = 0; k < path.size() - 1; k++)
        {
            d += BaseT::distanceBundle(path.at(k), path.at(k + 1));
        }
        return d;
    }
}

double BundleSpaceMetricShortestPath::distanceFiber(const Configuration *xStart, const Configuration *xDest)
{
    return BaseT::distanceFiber(xStart, xDest);
}

double BundleSpaceMetricShortestPath::distanceBase(const Configuration *xStart, const Configuration *xDest)
{
    OMPL_WARN("Computing geodesic distance on base space. If you want the graph-based distance, call distanceBundle "
              "instead.");
    return BaseT::distanceBase(xStart, xDest);
}

std::vector<const Configuration *> BundleSpaceMetricShortestPath::getInterpolationPath(const Configuration *xStart,
                                                                                       const Configuration *xDest)
{
    BundleSpaceGraph *parent = dynamic_cast<BundleSpaceGraph *>(bundleSpaceGraph_->getParent());

    const base::SpaceInformationPtr &base = bundleSpaceGraph_->getBase();
    const base::SpaceInformationPtr &bundle = bundleSpaceGraph_->getBundle();

    base::State *sStart = xStart->state;
    base::State *sDest = xDest->state;

    //(1) project onto base
    bundleSpaceGraph_->projectBase(sStart, xBaseStart_->state);
    bundleSpaceGraph_->projectBase(sDest, xBaseDest_->state);

    //(2) get nearest graph nodes on base
    const Configuration *xBaseNearestStart = parent->nearest(xBaseStart_);
    const Configuration *xBaseNearestDest = parent->nearest(xBaseDest_);

    //(3) compute path on base between nearest graph nodes
    // std::vector<const Configuration*>
    base::PathPtr pathBasePtr = parent->getPath(xBaseNearestStart->index, xBaseNearestDest->index);

    //(4) use path on base space to connect start to dest
    std::vector<const Configuration *> pathBundle;
    pathBundle.push_back(xStart);

    if (pathBasePtr)
    {
        geometric::PathGeometricPtr gpath = std::static_pointer_cast<geometric::PathGeometric>(pathBasePtr);

        const std::vector<base::State *> pathBase = gpath->getStates();
        if (pathBase.size() > 1)
        {
            // Fill Up temporary path if necessary
            if (pathBase.size() > tmpPath_.size())
            {
                for (uint k = tmpPath_.size(); k < pathBase.size(); k++)
                {
                    Configuration *qk = new Configuration(bundleSpaceGraph_->getBundle());
                    tmpPath_.push_back(qk);
                }
            }
            //(4b) interpolate path on base, then lift up by interpolating along fiber
            double lengthBasePath = 0;
            std::vector<double> lengthsBasePath;
            double lengthFirstSegment = base->distance(xBaseStart_->state, pathBase.at(0));
            lengthsBasePath.push_back(lengthFirstSegment);
            lengthBasePath += lengthFirstSegment;
            for (uint k = 1; k < pathBase.size(); k++)
            {
                double lengthKthSegment = base->distance(pathBase.at(k - 1), pathBase.at(k));
                lengthsBasePath.push_back(lengthKthSegment);
                lengthBasePath += lengthKthSegment;
            }
            double lengthLastSegment = base->distance(pathBase.back(), xBaseDest_->state);
            lengthsBasePath.push_back(lengthLastSegment);
            lengthBasePath += lengthLastSegment;

            if (bundleSpaceGraph_->getFiberDimension() > 0)
            {
                // Case 1: Interpolate on Fiber
                const base::SpaceInformationPtr &fiber = bundleSpaceGraph_->getFiber();
                bundleSpaceGraph_->projectFiber(sStart, xFiberStartTmp_);
                bundleSpaceGraph_->projectFiber(sDest, xFiberDestTmp_);

                double lengthCurrent = 0;
                for (uint k = 0; k < pathBase.size(); k++)
                {
                    base::State *xFiberK = fiber->allocState();
                    lengthCurrent += lengthsBasePath.at(k);

                    double interpLength = lengthCurrent / lengthBasePath;
                    fiber->getStateSpace()->interpolate(xFiberStartTmp_, xFiberDestTmp_, interpLength, xFiberK);

                    // Configuration *xk = new Configuration(bundle);
                    bundleSpaceGraph_->liftState(pathBase.at(k), xFiberK, tmpPath_.at(k)->state);
                    pathBundle.push_back(tmpPath_.at(k));
                    fiber->freeState(xFiberK);
                }
            }
            else
            {
                // Case 2: bundle = base, just copy states
                for (uint k = 0; k < pathBase.size(); k++)
                {
                    // Configuration *xk = new Configuration(bundle);
                    bundle->copyState(tmpPath_.at(k)->state, pathBase.at(k));
                    pathBundle.push_back(tmpPath_.at(k));
                }
            }
        }
    }
    pathBundle.push_back(xDest);
    return pathBundle;
}

void BundleSpaceMetricShortestPath::interpolateBundle(const Configuration *q_from, const Configuration *q_to,
                                                      const double step, Configuration *q_interp)
{
    if (bundleSpaceGraph_->getBaseDimension() <= 0)
    {
        return BaseT::interpolateBundle(q_from, q_to, step, q_interp);
    }
    else
    {
        std::vector<const Configuration *> path = getInterpolationPath(q_from, q_to);
        if (path.size() <= 2)
        {
            return BaseT::interpolateBundle(q_from, q_to, step, q_interp);
        }

        double d_path = 0;
        for (uint k = 0; k < path.size() - 1; k++)
        {
            d_path += BaseT::distanceBundle(path.at(k), path.at(k + 1));
        }

        double d_step = step * d_path;
        double d_last_to_next = 0;

        unsigned int ctr = 0;
        double d = 0;
        while (d < d_step && ctr < path.size() - 1)
        {
            d_last_to_next = BaseT::distanceBundle(path.at(ctr), path.at(ctr + 1));
            d += d_last_to_next;
            ctr++;
        }

        const Configuration *q_last = path.at(ctr - 1);
        const Configuration *q_next = path.at(ctr);

        //|--------------------- d_path -----------------------|
        //|----------------- d_step -------------|
        //                                |-- d_last_to_next --|
        //                                |-step-|
        // q1 ----- q2 ------ ... ----- q_last ---|---------- q_next

        double step = (d_last_to_next - (d - d_step)) / d_last_to_next;

        BaseT::interpolateBundle(q_last, q_next, step, q_interp);
    }
}
