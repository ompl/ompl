/* MorseProjecetion.h */

#ifndef OMPL_EXTENSION_MORSE_PROJECTION_
#define OMPL_EXTENSION_MORSE_PROJECTION_

#include "ompl/extensions/morse/MorseStateSpace.h"

namespace ompl
{
    namespace base
    {
        
        /** \brief This class implements a generic projection for the MorseStateSpace,
            namely, the subspace representing the x and y positions of every rigid body */
        class MorseProjection : public ProjectionEvaluator
        {
        public:
            /** \brief Construct a projection evaluator for a specific state space */
            MorseProjection(const StateSpacePtr &space);
            
            /** \brief Perform configuration steps, if needed */
            void setup(void);
            
            /** \brief Return the dimension of the projection defined by this evaluator */
            virtual unsigned int getDimension(void) const;
            
            /** \brief Set the default cell dimensions for this
                projection. The default implementation of this
                function sets the size to 1.0 for all dimensions.
                setup() calls this function if no cell
                dimensions have been previously set. */
            virtual void defaultCellSizes(void);
            
            /** \brief Compute the projection as an array of double values */
            virtual void project(const State *state, EuclideanProjection &projection) const;
            
        protected:
            /** \brief The state space this projection operates on */
            MorseStateSpace *space_;
        };
    }
}

#endif
