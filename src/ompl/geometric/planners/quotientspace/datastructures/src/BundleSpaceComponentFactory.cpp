#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceComponentFactory.h>

#include <ompl/geometric/planners/quotientspace/datastructures/components/SE2_R2.h>
#include <ompl/geometric/planners/quotientspace/datastructures/components/SE3_R3.h>
#include <ompl/geometric/planners/quotientspace/datastructures/components/NoProjection.h>
#include <ompl/geometric/planners/quotientspace/datastructures/components/EmptySetProjection.h>
#include <ompl/geometric/planners/quotientspace/datastructures/components/Identity.h>

#include <ompl/util/Exception.h>

ompl::geometric::BundleSpaceComponentPtr
ompl::geometric::BundleSpaceComponentFactory::MakeBundleSpaceComponent(
    const base::StateSpacePtr Bundle, 
    const base::StateSpacePtr Base)
{
    BundleSpaceComponentType type = identifyBundleSpaceComponentType(Bundle, Base);
    BundleSpaceComponentPtr component;

    if(type == BUNDLE_SPACE_NO_PROJECTION)
    {
      component = std::make_shared<BundleSpaceComponent_NoProjection>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_EMPTY_SET_PROJECTION){
      component = std::make_shared<BundleSpaceComponent_EmptySetProjection>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_IDENTITY_PROJECTION){
      component = std::make_shared<BundleSpaceComponent_Identity>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE2_R2){
      component = std::make_shared<BundleSpaceComponent_SE2_R2>(Bundle, Base);
    }else if(type == BUNDLE_SPACE_SE3_R3){
      component = std::make_shared<BundleSpaceComponent_SE3_R3>(Bundle, Base);
    }else{
      OMPL_ERROR("NYI: %d", type);
      throw Exception("BundleSpaceType not yet implemented.");
    }
    component->initFiberSpace();
    return component;
}


ompl::geometric::BundleSpaceComponentType
ompl::geometric::BundleSpaceComponentFactory::identifyBundleSpaceComponentType(const base::StateSpacePtr Bundle, const base::StateSpacePtr Base)
{
    //
    // We can currently handle 11 types of quotient-space mappings.
    // Emptyset is used for constraint relaxations.
    //
    //   (1)  Bundle Rn     , Base Rm     [0<m<=n]  => Fiber = R(n-m) \union {\emptyset}
    //   (2a) Bundle SE2    , Base R2               => Fiber = SO2
    //   (2b) Bundle SE2    , Base SE2              => Fiber = \emptyset
    //   (3a) Bundle SE3    , Base R3               => Fiber = SO3
    //   (3b) Bundle SE3    , Base SE3              => Fiber = \emptyset
    //
    //   (4)  Bundle SE3xRn , Base SE3              => Fiber = Rn
    //   (5)  Bundle SE3xRn , Base R3               => Fiber = SO3xRn
    //   (6)  Bundle SE3xRn , Base SE3xRm [0<m<=n ] => Fiber = R(n-m) \union {\emptyset}
    //
    //   (7)  Bundle SE2xRn , Base SE2              => Fiber = Rn
    //   (8)  Bundle SE2xRn , Base R2               => Fiber = SO2xRN
    //   (9)  Bundle SE2xRn , Base SE2xRm [0<m<=n ] => Fiber = R(n-m) \union {\emptyset}
    //
    //  (10)  Bundle SO2xRn , Base SO2              => Fiber = Rn
    //  (11)  Bundle SO2xRn , Base SO2xRm [0<m<=n ] => Fiber = R(n-m) \union {\emptyset}
    //  (12)  Multiagent (any combination of (1-11))

    if(Base == nullptr){
      return BUNDLE_SPACE_NO_PROJECTION;
    }

    BundleSpaceComponentType type;

    if (!Bundle->isCompound())
    {
        ///##############################################################################/
        //------------------ non-compound cases:
        ///##############################################################################/
        //
        //------------------ (1) Bundle = Rn, Base = Rm, 0<m<n, Fiber = R(n-m)
        if (Bundle->getType() == base::STATE_SPACE_REAL_VECTOR)
        {
            unsigned int n = Bundle->getDimension();
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                unsigned int m = Base->getDimension();
                if (n > m && m > 0)
                {
                    type = BUNDLE_SPACE_RN_RM;
                }
                else
                {
                    if (n == m && m > 0)
                    {
                        type = BUNDLE_SPACE_IDENTITY_PROJECTION;
                    }
                    else
                    {
                        if(m==0){
                            type = BUNDLE_SPACE_EMPTY_SET_PROJECTION;
                        }else{
                            OMPL_ERROR("Not allowed: dimensionality needs to be monotonically increasing.");
                            OMPL_ERROR("We require n >= m > 0 but have n=%d >= m=%d > 0", n, m);
                            throw ompl::Exception("Invalid dimensionality");
                        }
                    }
                }
            }
            else
            {
                OMPL_ERROR("Bundle is R^%d but Base type %d is not handled.", n, Base->getType());
                throw ompl::Exception("INVALID_STATE_TYPE");
            }
        }
        else
        {
            OMPL_ERROR("Bundle is non-compound state, but its type %d is not handled.", Bundle->getType());
            throw ompl::Exception("INVALID_STATE_TYPE");
        }
    }
    else
    {
        ///##############################################################################/
        //------------------ compound cases:
        ///##############################################################################/
        //
        //------------------ (2) Bundle = SE2, Base = R2, Fiber = SO2
        ///##############################################################################/
        if (Bundle->getType() == base::STATE_SPACE_SE2)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                if (Base->getDimension() == 2)
                {
                    type = BUNDLE_SPACE_SE2_R2;
                }
                else if(Base->getDimension() == 0)
                {
                    type = BUNDLE_SPACE_EMPTY_SET_PROJECTION;
                }
                else
                {

                    OMPL_ERROR("Bundle is SE2 but Base type %d is of dimension %d", Base->getType(), Base->getDimension());
                    throw ompl::Exception("Invalid dimensions.");
                }
            }
            else
            {
                if (Base->getType() == base::STATE_SPACE_SE2)
                {
                    type = BUNDLE_SPACE_IDENTITY_PROJECTION;
                }
                else
                {
                    OMPL_ERROR("Bundle is SE2 but Base type %d is not handled.", Base->getType());
                    throw ompl::Exception("INVALID_STATE_TYPE");
                }
            }
        }
        //------------------ (3) Bundle = SE3, Base = R3, Fiber = SO3
        ///##############################################################################/
        else if (Bundle->getType() == base::STATE_SPACE_SE3)
        {
            if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
            {
                if (Base->getDimension() == 3)
                {
                    type = BUNDLE_SPACE_SE3_R3;
                }
                else if(Base->getDimension() == 0)
                {
                    type = BUNDLE_SPACE_EMPTY_SET_PROJECTION;
                }
                else
                {
                    OMPL_ERROR("Bundle is SE3 but Base type %d is of dimension %d.", Base->getType(), Base->getDimension());
                    throw ompl::Exception("Invalid dimensions.");
                }
            }
            else
            {
                if (Base->getType() == base::STATE_SPACE_SE3)
                {
                    type = BUNDLE_SPACE_IDENTITY_PROJECTION;
                }
                else
                {
                    OMPL_ERROR("Bundle is SE2 but Base type %d is not handled.", Base->getType());
                    throw ompl::Exception("Invalid BundleSpace type");
                }
                OMPL_ERROR("Bundle is SE3 but Base type %d is not handled.", Base->getType());
                throw ompl::Exception("Invalid BundleSpace type");
            }
        }
        ///##############################################################################/
        else
        {
            base::CompoundStateSpace *Bundle_compound = Bundle->as<base::CompoundStateSpace>();
            const std::vector<base::StateSpacePtr> Bundle_decomposed = Bundle_compound->getSubspaces();
            unsigned int Bundle_subspaces = Bundle_decomposed.size();
            if (Bundle_subspaces == 2)
            {
                if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
                    Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                {
                    unsigned int n = Bundle_decomposed.at(1)->getDimension();
                    if (Base->getType() == base::STATE_SPACE_SE3)
                    {
                        //------------------ (4) Bundle = SE3xRn, Base = SE3, Fiber = Rn
                        ///##############################################################################/
                        type = BUNDLE_SPACE_SE3RN_SE3;
                    }
                    else if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        //------------------ (5) Bundle = SE3xRn, Base = R3, Fiber = SO3xRN
                        ///##############################################################################/
                        unsigned int m = Base->getDimension();
                        if (m == 3)
                        {
                            type = BUNDLE_SPACE_SE3RN_R3;
                        }
                        else if(m == 0)
                        {
                            type = BUNDLE_SPACE_EMPTY_SET_PROJECTION;
                        }
                        else
                        {
                            OMPL_ERROR("Not allowed. Base needs to be 3-dimensional but is %d dimensional", m);
                            throw ompl::Exception("Invalid dimensions.");
                        }
                    }
                    else
                    {
                        //------------------ (6) Bundle = SE3xRn, Base = SE3xRm, Fiber = R(n-m)
                        ///##############################################################################/
                        base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
                        const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
                        unsigned int Base_subspaces = Base_decomposed.size();
                        if (Base_subspaces == 2)
                        {
                            if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE3 &&
                                Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                            {
                                unsigned int m = Base_decomposed.at(1)->getDimension();
                                if (m < n && m > 0)
                                {
                                    type = BUNDLE_SPACE_SE3RN_SE3RM;
                                }
                                else
                                {
                                    if (m == n)
                                    {
                                        type = BUNDLE_SPACE_IDENTITY_PROJECTION;
                                    }
                                    else
                                    {
                                        if(m == 0){
                                            type = BUNDLE_SPACE_EMPTY_SET_PROJECTION;
                                        }else{
                                            OMPL_ERROR("We require n >= m > 0, but have n=%d >= m=%d > 0.", n, m);
                                            throw ompl::Exception("Invalid dimensions.");
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            OMPL_ERROR("State compound with %d subspaces not handled.", Base_subspaces);
                            throw ompl::Exception("Invalid BundleSpace type");
                        }
                    }
                }
                else
                {
                    if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
                        Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        unsigned int n = Bundle_decomposed.at(1)->getDimension();
                        if (Base->getType() == base::STATE_SPACE_SE2)
                        {
                            //------------------ (7) Bundle = SE2xRn, Base = SE2, Fiber = Rn
                            ///##############################################################################/
                            type = BUNDLE_SPACE_SE2RN_SE2;
                        }
                        else if (Base->getType() == base::STATE_SPACE_REAL_VECTOR)
                        {
                            //------------------ (8) Bundle = SE2xRn, Base = R2, Fiber = SO2xRN
                            ///##############################################################################/
                            unsigned int m = Base->getDimension();
                            if (m == 2)
                            {
                                type = BUNDLE_SPACE_SE2RN_R2;
                            }
                            else
                            {
                                if(m == 0){
                                    type = BUNDLE_SPACE_EMPTY_SET_PROJECTION;
                                }else{
                                    OMPL_ERROR("Not allowed. Base needs to be 2-dimensional but is %d dimensional", m);
                                    throw ompl::Exception("Invalid dimensions.");
                                }
                            }
                        }
                        else
                        {
                            //------------------ (9) Bundle = SE2xRn, Base = SE2xRm, Fiber = R(n-m)
                            ///##############################################################################/
                            base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
                            const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
                            unsigned int Base_subspaces = Base_decomposed.size();
                            if (Base_subspaces == 2)
                            {
                                if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SE2 &&
                                    Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                                {
                                    unsigned int m = Base_decomposed.at(1)->getDimension();
                                    if (m < n && m > 0)
                                    {
                                        type = BUNDLE_SPACE_SE2RN_SE2RM;
                                    }
                                    else
                                    {
                                        if (m == n)
                                        {
                                            type = BUNDLE_SPACE_IDENTITY_PROJECTION;
                                        }
                                        else
                                        {
                                            OMPL_ERROR("We require n >= m > 0, but have n=%d >= m=%d > 0.", n, m);
                                            throw ompl::Exception("Invalid dimensions.");
                                        }
                                    }
                                }
                                else
                                {
                                }
                            }
                            else
                            {
                                OMPL_ERROR("QO is compound with %d subspaces, but we only handle 2.", Base_subspaces);
                                throw ompl::Exception("Invalid BundleSpace type");
                            }
                        }
                    }
                    else if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SO2 &&
                             Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        if (Base->getType() == base::STATE_SPACE_SO2)
                        {
                            //------------------ (10) Bundle = SO2xRn, Base = SO2, Fiber = Rn
                            ///##############################################################################/
                            type = BUNDLE_SPACE_SO2RN_SO2;
                        }
                        else
                        {
                            //------------------ (11) Bundle = SO2xRn, Base = SO2xRm, Fiber = R(n-m)
                            ///##############################################################################/
                            if (Base->isCompound())
                            {
                                base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
                                const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
                                unsigned int Base_subspaces = Base_decomposed.size();
                                if (Base_subspaces == 2)
                                {
                                    if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_SO2 &&
                                        Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                                    {
                                        unsigned int n = Bundle_decomposed.at(1)->getDimension();
                                        unsigned int m = Base_decomposed.at(1)->getDimension();
                                        if (m < n && m > 0)
                                        {
                                            type = BUNDLE_SPACE_SO2RN_SO2RM;
                                        }
                                        else
                                        {
                                            if (m == n)
                                            {
                                                type = BUNDLE_SPACE_IDENTITY_PROJECTION;
                                            }
                                            else
                                            {
                                                OMPL_ERROR("We require n >= m > 0 but have n=%d >= m=%d > 0.", n, m);
                                                throw ompl::Exception("Invalid dimensions.");
                                            }
                                        }
                                    }
                                    else
                                    {
                                        OMPL_ERROR("Cannot project onto type %d.", Bundle->getType());
                                        throw ompl::Exception("Invalid BundleSpace type.");
                                    }
                                }
                                else
                                {
                                    OMPL_ERROR("Base has %d subspaces. We can handle only 2.", Base_subspaces);
                                    throw ompl::Exception("Invalid BundleSpace type.");
                                }
                            }
                            else
                            {
                                OMPL_ERROR("Cannot project onto type %d.", Base->getType());
                                throw ompl::Exception("Invalid BundleSpace type.");
                            }
                        }
                    }
                    else if (Bundle_decomposed.at(0)->getType() == base::STATE_SPACE_REAL_VECTOR &&
                             Bundle_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR)
                    {
                        type = BUNDLE_SPACE_UNKNOWN;
                    }else{
                      if(Bundle_decomposed.at(0)->isCompound() &&
                           Bundle_decomposed.at(1)->isCompound())
                      {
                        type = BUNDLE_SPACE_UNKNOWN;
                      }else{
                        OMPL_ERROR("State compound %d and %d not recognized.", 
                            Bundle_decomposed.at(0)->getType(), 
                            Bundle_decomposed.at(1)->getType());
                        throw ompl::Exception("Invalid BundleSpace type.");
                      }
                    }
                }
            }
            else
            {
              if(Bundle_subspaces >= 1){
                if (!Base->isCompound())
                {
                    OMPL_ERROR("Bundle is compound, but Base is not.");
                    throw ompl::Exception("Invalid BundleSpace type.");
                }else{
                    base::CompoundStateSpace *Base_compound = Base->as<base::CompoundStateSpace>();
                    const std::vector<base::StateSpacePtr> Base_decomposed = Base_compound->getSubspaces();
                    unsigned int Base_subspaces = Base_decomposed.size();
                    if(Bundle_subspaces != Base_subspaces){
                      OMPL_ERROR("Bundle has %d subspaces, but Base has %d.", Bundle_subspaces, Base_subspaces);
                      throw ompl::Exception("Invalid BundleSpace type.");
                    }
                    type = BUNDLE_SPACE_UNKNOWN;
                }

              }else{
                OMPL_ERROR("Bundle has %d subspaces.", Bundle_subspaces);
                throw ompl::Exception("Invalid BundleSpace type.");
              }
            }
        }
    }
    return type;
}
