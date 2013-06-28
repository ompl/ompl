/* MorseControlSpace.h */

#ifndef OMPL_EXTENSION_MORSE_CONTROL_SPACE_
#define OMPL_EXTENSION_MORSE_CONTROL_SPACE_

#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/extensions/morse/MorseStateSpace.h"

namespace ompl
{

    namespace control
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::MorseEnvironment */
        OMPL_CLASS_FORWARD(MorseControlSpace);
        /// @endcond
        
        /** \brief Representation of controls applied in MORSE
            environments. This is an array of double values. */
        class MorseControlSpace : public RealVectorControlSpace
        {
        public:

            /** \brief Construct a representation of controls passed
                to MORSE. If \e stateSpace does not cast to an
                MorseStateSpace, an exception is thrown. */
            MorseControlSpace(const base::StateSpacePtr &stateSpace);

            virtual ~MorseControlSpace(void)
            {
            }

            /*Control* allocControl(void) const
            {
                ControlType *control = new ControlType();
                control->values = new double[dimension_];
                return control;
            }*/

            /** \brief Get the MORSE environment this state space corresponds to */
            const base::MorseEnvironmentPtr& getEnvironment(void) const
            {
                return stateSpace_->as<base::MorseStateSpace>()->getEnvironment();
            }

        };
    }

}


#endif
