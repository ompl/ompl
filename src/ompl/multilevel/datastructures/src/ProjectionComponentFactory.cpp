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

#include <ompl/multilevel/datastructures/ProjectionComponentFactory.h>

// XRN -> X
#include <ompl/multilevel/datastructures/components/XRN_X_SO2.h>
#include <ompl/multilevel/datastructures/components/XRN_X_SO3.h>
#include <ompl/multilevel/datastructures/components/XRN_X_SE2.h>
#include <ompl/multilevel/datastructures/components/XRN_X_SE3.h>

// XRN -> XRM
#include <ompl/multilevel/datastructures/components/XRN_XRM_SO2.h>
#include <ompl/multilevel/datastructures/components/XRN_XRM_SO3.h>
#include <ompl/multilevel/datastructures/components/XRN_XRM_SE2.h>
#include <ompl/multilevel/datastructures/components/XRN_XRM_SE3.h>

#include <ompl/multilevel/datastructures/components/SE3_R3.h>
#include <ompl/multilevel/datastructures/components/SE3RN_R3.h>
#include <ompl/multilevel/datastructures/components/SE2_R2.h>
#include <ompl/multilevel/datastructures/components/SE2RN_R2.h>

#include <ompl/multilevel/datastructures/components/RN_RM.h>
#include <ompl/multilevel/datastructures/components/RNSO2_RN.h>
#include <ompl/multilevel/datastructures/components/SO2N_SO2M.h>

#include <ompl/multilevel/datastructures/components/None.h>
#include <ompl/multilevel/datastructures/components/EmptySet.h>
#include <ompl/multilevel/datastructures/components/Identity.h>
#include <ompl/multilevel/datastructures/components/Relaxation.h>

#include <ompl/util/Exception.h>

using namespace ompl::multilevel;

std::vector<ProjectionComponentPtr>
ProjectionComponentFactory::MakeProjectionComponents(ompl::base::SpaceInformationPtr Bundle)
{
    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    int ProjectionComponents = GetNumberOfComponents(Bundle_space);

    std::vector<ProjectionComponentPtr> components;

    OMPL_DEBUG("Bundle components: %d", ProjectionComponents);

    if (ProjectionComponents > 1)
    {
        base::CompoundStateSpace *Bundle_compound = Bundle_space->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();

        for (int m = 0; m < ProjectionComponents; m++)
        {
            base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
            ProjectionComponentPtr componentM = MakeProjectionComponent(BundleM);
            components.push_back(componentM);
        }
    }
    else
    {
        ProjectionComponentPtr component = MakeProjectionComponent(Bundle_space);
        components.push_back(component);
    }

    return components;
}

std::vector<ProjectionComponentPtr> ProjectionComponentFactory::MakeProjectionComponents(
    ompl::base::SpaceInformationPtr Bundle, ompl::base::SpaceInformationPtr Base)
{
    const base::StateSpacePtr Bundle_space = Bundle->getStateSpace();
    int ProjectionComponents = GetNumberOfComponents(Bundle_space);
    const base::StateSpacePtr Base_space = Base->getStateSpace();
    int baseSpaceComponents = GetNumberOfComponents(Base_space);

    if (baseSpaceComponents != ProjectionComponents)
    {
        Base->printSettings();
        OMPL_ERROR("Base Space has %d, but Bundle Space has %d components.", baseSpaceComponents,
                   ProjectionComponents);
        throw Exception("Different Number Of Components");
    }

    std::vector<ProjectionComponentPtr> components;

    // Check if planning spaces are equivalent, i.e. if (X, \phi) == (Y, \phi)
    bool areValidityCheckersEquivalent = false;
    if (*(Base->getStateValidityChecker().get()) == *(Bundle->getStateValidityChecker().get()))
    {
        areValidityCheckersEquivalent = true;
    }

    if (ProjectionComponents > 1)
    {
        base::CompoundStateSpace *Bundle_compound = Bundle_space->as<base::CompoundStateSpace>();
        base::CompoundStateSpace *Base_compound = Base_space->as<base::CompoundStateSpace>();

        const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
        const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

        for (int m = 0; m < ProjectionComponents; m++)
        {
            base::StateSpacePtr BaseM = Base_decomposed.at(m);
            base::StateSpacePtr BundleM = Bundle_decomposed.at(m);
            ProjectionComponentPtr componentM =
                MakeProjectionComponent(BundleM, BaseM, areValidityCheckersEquivalent);
            components.push_back(componentM);
        }
    }
    else
    {
        ProjectionComponentPtr component =
            MakeProjectionComponent(Bundle_space, Base_space, areValidityCheckersEquivalent);
        components.push_back(component);
    }
    return components;
}

ProjectionComponentPtr ProjectionComponentFactory::MakeProjectionComponent(const ompl::base::StateSpacePtr Bundle)
{
    return MakeProjectionComponent(Bundle, nullptr, false);
}

ProjectionComponentPtr ProjectionComponentFactory::MakeProjectionComponent(
    const ompl::base::StateSpacePtr Bundle, 
    const ompl::base::StateSpacePtr Base, 
    bool areValidityCheckersEquivalent)
{
    ProjectionComponentType type = identifyProjectionComponentType(Bundle, Base);
    if (type == BUNDLE_SPACE_IDENTITY_PROJECTION && !areValidityCheckersEquivalent)
    {
        type = BUNDLE_SPACE_CONSTRAINED_RELAXATION;
    }

    ProjectionComponentPtr component;

    if (type == BUNDLE_SPACE_NO_PROJECTION)
    {
        component = std::make_shared<ProjectionComponentWithFiber_None>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_EMPTY_SET_PROJECTION)
    {
        component = std::make_shared<ProjectionComponentWithFiber_EmptySet>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_IDENTITY_PROJECTION)
    {
        component = std::make_shared<ProjectionComponentWithFiber_Identity>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_CONSTRAINED_RELAXATION)
    {
        component = std::make_shared<ProjectionComponentWithFiber_Relaxation>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_RN_RM)
    {
        component = std::make_shared<ProjectionComponentWithFiber_RN_RM>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_RNSO2_RN)
    {
        component = std::make_shared<ProjectionComponentWithFiber_RNSO2_RN>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE2_R2)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE2_R2>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE2RN_R2)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE2RN_R2>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE2RN_SE2)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE2RN_SE2>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE2RN_SE2RM)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE2RN_SE2RM>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SO2RN_SO2)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SO2RN_SO2>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SO2RN_SO2RM)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SO2RN_SO2RM>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SO3RN_SO3)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SO3RN_SO3>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SO3RN_SO3RM)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SO3RN_SO3RM>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE3_R3)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE3_R3>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE3RN_R3)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE3RN_R3>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE3RN_SE3)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE3RN_SE3>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SE3RN_SE3RM)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SE3RN_SE3RM>(Bundle, Base);
    }
    else if (type == BUNDLE_SPACE_SO2N_SO2M)
    {
        component = std::make_shared<ProjectionComponentWithFiber_SO2N_SO2M>(Bundle, Base);
    }
    else
    {
        OMPL_ERROR("NYI: %d", type);
        throw Exception("BundleSpaceType not yet implemented.");
    }
    component->setType(type);
    auto componentFiber = std::dynamic_pointer_cast<ProjectionComponentWithFiber>(component);
    if(componentFiber != nullptr)
    {
        componentFiber->makeFiberSpace();
    }
    return component;
}

ProjectionComponentType ProjectionComponentFactory::identifyProjectionComponentType(
    const ompl::base::StateSpacePtr Bundle, const ompl::base::StateSpacePtr Base)
{
    if (Base == nullptr)
    {
        return BUNDLE_SPACE_NO_PROJECTION;
    }

    if (isMapping_Identity(Bundle, Base))
    {
        return BUNDLE_SPACE_IDENTITY_PROJECTION;
    }

    if (isMapping_EmptyProjection(Bundle, Base))
    {
        return BUNDLE_SPACE_EMPTY_SET_PROJECTION;
    }

    // RN ->
    if (isMapping_RN_to_RM(Bundle, Base))
    {
        return BUNDLE_SPACE_RN_RM;
    }
    if (isMapping_RNSO2_to_RN(Bundle, Base))
    {
        return BUNDLE_SPACE_RNSO2_RN;
    }

    // SE3 ->
    if (isMapping_SE3_to_R3(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3_R3;
    }
    if (isMapping_SE3RN_to_SE3(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3RN_SE3;
    }
    if (isMapping_SE3RN_to_R3(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3RN_R3;
    }
    if (isMapping_SE3RN_to_SE3RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SE3RN_SE3RM;
    }

    // SE2 ->
    if (isMapping_SE2_to_R2(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2_R2;
    }
    if (isMapping_SE2RN_to_SE2(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2RN_SE2;
    }
    if (isMapping_SE2RN_to_R2(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2RN_R2;
    }
    if (isMapping_SE2RN_to_SE2RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SE2RN_SE2RM;
    }

    // SO2 ->
    if (isMapping_SO2RN_to_SO2(Bundle, Base))
    {
        return BUNDLE_SPACE_SO2RN_SO2;
    }
    if (isMapping_SO2RN_to_SO2RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SO2RN_SO2RM;
    }
    if (isMapping_SO2N_to_SO2M(Bundle, Base))
    {
        return BUNDLE_SPACE_SO2N_SO2M;
    }

    // SO3 ->
    if (isMapping_SO3RN_to_SO3(Bundle, Base))
    {
        return BUNDLE_SPACE_SO3RN_SO3;
    }
    if (isMapping_SO3RN_to_SO3RM(Bundle, Base))
    {
        return BUNDLE_SPACE_SO3RN_SO3RM;
    }

    OMPL_ERROR("Fiber Bundle unknown.");
    return BUNDLE_SPACE_UNKNOWN;
}

bool ProjectionComponentFactory::isMapping_Identity(const ompl::base::StateSpacePtr Bundle,
                                                     const ompl::base::StateSpacePtr Base)
{
    if (Bundle->isCompound())
    {
        if (Base->isCompound())
        {
            base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
            base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();

            if (Bundle_decomposed.size() == Base_decomposed.size())
            {
                for (unsigned int k = 0; k < Bundle_decomposed.size(); k++)
                {
                    if (!isMapping_Identity(Bundle_decomposed.at(k), Base_decomposed.at(k)))
                    {
                        return false;
                    }
                }
            }
            return true;
        }
    }
    else
    {
        if ((Base->getType() == Bundle->getType()) && (Base->getDimension() == Bundle->getDimension()))
        {
            return true;
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_RN_to_RM(const ompl::base::StateSpacePtr Bundle,
                                                     const ompl::base::StateSpacePtr Base)
{
    if (Bundle->isCompound())
        return false;

    if (Bundle->getType() == base::STATE_SPACE_REAL_VECTOR)
    {
        unsigned int n = Bundle->getDimension();
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            unsigned int m = Base->getDimension();
            if (n > m && m > 0)
            {
                return true;
            }
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_SE3_to_R3(const ompl::base::StateSpacePtr Bundle,
                                                      const ompl::base::StateSpacePtr Base)
{
    if (!Bundle->isCompound())
        return false;

    if (Bundle->getType() == base::STATE_SPACE_SE3)
    {
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getDimension() == 3)
            {
                return true;
            }
        }
    }
    return false;
}
bool ProjectionComponentFactory::isMapping_SE3RN_to_R3(const ompl::base::StateSpacePtr Bundle,
                                                        const ompl::base::StateSpacePtr Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Base->getDimension();
                if (m == 3)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_SE2_to_R2(const ompl::base::StateSpacePtr Bundle,
                                                      const ompl::base::StateSpacePtr Base)
{
    if (!Bundle->isCompound())
        return false;

    if (Bundle->getType() == base::STATE_SPACE_SE2)
    {
        if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getDimension() == 2)
            {
                return true;
            }
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_RNSO2_to_RN(const ompl::base::StateSpacePtr Bundle,
                                                        const ompl::base::StateSpacePtr Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_REAL_VECTOR &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_SO2)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int n = Bundle_decomposed.at(0)->getDimension();
                unsigned int m = Base->getDimension();
                if (m == n)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_SE2RN_to_R2(const ompl::base::StateSpacePtr Bundle,
                                                        const ompl::base::StateSpacePtr Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Base->getDimension();
                if (m == 2)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_SE2RN_to_SE2(const ompl::base::StateSpacePtr Bundle,
                                                         const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SE2);
}

bool ProjectionComponentFactory::isMapping_SE3RN_to_SE3(const ompl::base::StateSpacePtr Bundle,
                                                         const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SE3);
}

bool ProjectionComponentFactory::isMapping_SO2RN_to_SO2(const ompl::base::StateSpacePtr Bundle,
                                                         const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SO2);
}

bool ProjectionComponentFactory::isMapping_SO3RN_to_SO3(const ompl::base::StateSpacePtr Bundle,
                                                         const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_X(Bundle, Base, base::STATE_SPACE_SO3);
}

bool ProjectionComponentFactory::isMapping_SE2RN_to_SE2RM(const ompl::base::StateSpacePtr Bundle,
                                                           const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SE2);
}

bool ProjectionComponentFactory::isMapping_SE3RN_to_SE3RM(const ompl::base::StateSpacePtr Bundle,
                                                           const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SE3);
}

bool ProjectionComponentFactory::isMapping_SO2RN_to_SO2RM(const ompl::base::StateSpacePtr Bundle,
                                                           const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SO2);
}

bool ProjectionComponentFactory::isMapping_SO3RN_to_SO3RM(const ompl::base::StateSpacePtr Bundle,
                                                           const ompl::base::StateSpacePtr Base)
{
    return isMapping_XRN_to_XRM(Bundle, Base, base::STATE_SPACE_SO3);
}

bool ProjectionComponentFactory::isMapping_SO2N_to_SO2M(const ompl::base::StateSpacePtr Bundle,
                                                         const ompl::base::StateSpacePtr Base)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();

    for (unsigned int k = 0; k < Bundle_subspaces; k++)
    {
        if (!(Bundle_decomposed.at(k)->getType() == base::STATE_SPACE_SO2))
        {
            return false;
        }
    }
    if (!Base->isCompound())
    {
        if (!(Base->getType() == base::STATE_SPACE_SO2))
        {
            return false;
        }
    }
    else
    {
        base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
        const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
        unsigned int Base_subspaces = Base_decomposed.size();

        for (unsigned int k = 0; k < Base_subspaces; k++)
        {
            if (!(Base_decomposed.at(k)->getType() == base::STATE_SPACE_SO2))
            {
                return false;
            }
        }
    }

    return true;
}

bool ProjectionComponentFactory::isMapping_XRN_to_X(const ompl::base::StateSpacePtr Bundle,
                                                     const ompl::base::StateSpacePtr Base,
                                                     const ompl::base::StateSpaceType type)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    unsigned int Bundle_subspaces = Bundle_decomposed.size();
    if (Bundle_subspaces == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == type &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (Base->getType() == type)
            {
                return true;
            }
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_XRN_to_XRM(const ompl::base::StateSpacePtr Bundle,
                                                       const ompl::base::StateSpacePtr Base,
                                                       const ompl::base::StateSpaceType type)
{
    if (!Bundle->isCompound())
        return false;

    base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
    const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
    if (Bundle_decomposed.size() == 2)
    {
        if (Bundle_decomposed.at(0)->getType() == type &&
            Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            if (!Base->isCompound())
                return false;
            unsigned int n = Bundle_decomposed.at(1)->getDimension();

            base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
            if (Base_decomposed.size() == 2)
            {
                if (Base_decomposed.at(0)->getType() == type &&
                    Base_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                {
                    unsigned int m = Base_decomposed.at(1)->getDimension();
                    if (n > m && m > 0)
                    {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool ProjectionComponentFactory::isMapping_EmptyProjection(const ompl::base::StateSpacePtr,
                                                            const ompl::base::StateSpacePtr Base)
{
    if (Base == nullptr || Base->getDimension() <= 0)
    {
        return true;
    }
    return false;
}
int ProjectionComponentFactory::GetNumberOfComponents(ompl::base::StateSpacePtr space)
{
    int nrComponents = 0;

    if (space->isCompound())
    {
        base::CompoundStateSpace *compound = space->as<base::CompoundStateSpace>();
        nrComponents = compound->getSubspaceCount();
        if (nrComponents == 2)
        {
            int type = space->getType();

            if ((type == base::STATE_SPACE_SE2) || (type == base::STATE_SPACE_SE3) ||
                (type == base::STATE_SPACE_DUBINS))  // || (type == base::STATE_SPACE_DUBINS_AIRPLANE))
            {
                nrComponents = 1;
            }
            else
            {
                const std::vector<base::StateSpacePtr> decomposed = compound->getSubspaces();
                int t0 = decomposed.at(0)->getType();
                int t1 = decomposed.at(1)->getType();
                if ((t0 == base::STATE_SPACE_SO2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SO3 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SE2 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SE3 && t1 == base::STATE_SPACE_REAL_VECTOR) ||
                    (t0 == base::STATE_SPACE_SO2 && t1 == base::STATE_SPACE_SO2))
                {
                    if (decomposed.at(1)->getDimension() > 0)
                    {
                        nrComponents = 1;
                    }
                }
            }
        }
    }
    else
    {
        nrComponents = 1;
    }
    return nrComponents;
}
