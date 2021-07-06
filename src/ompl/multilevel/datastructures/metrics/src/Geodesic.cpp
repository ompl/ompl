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

#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
#include <ompl/multilevel/datastructures/Projection.h>
using namespace ompl::multilevel;
using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

BundleSpaceMetricGeodesic::BundleSpaceMetricGeodesic(BundleSpaceGraph *bundleSpaceGraph) : BaseT(bundleSpaceGraph)
{
}

double BundleSpaceMetricGeodesic::distanceBundle(const Configuration *xStart, const Configuration *xDest)
{
    return bundleSpaceGraph_->getBundle()->distance(xStart->state, xDest->state);
}

double BundleSpaceMetricGeodesic::distanceFiber(const Configuration * /*xStart*/, const Configuration * /*xDest*/)
{
    throw "Computing distance along fiber requires explicit fiber space.";
    return 0.0;
}

double BundleSpaceMetricGeodesic::distanceBase(const Configuration *xStart, const Configuration *xDest)
{
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        bundleSpaceGraph_->getProjection()->project(xStart->state, xBaseStartTmp_);
        bundleSpaceGraph_->getProjection()->project(xDest->state, xBaseDestTmp_);
        double d = bundleSpaceGraph_->getBase()->distance(xBaseStartTmp_, xBaseDestTmp_);
        return d;
    }
    else
    {
        return 0.0;
    }
}

void BundleSpaceMetricGeodesic::interpolateBundle(const Configuration *q_from, const Configuration *q_to,
                                                  const double step, Configuration *q_interp)
{
    bundleSpaceGraph_->getBundle()->getStateSpace()->interpolate(q_from->state, q_to->state, step, q_interp->state);
}
