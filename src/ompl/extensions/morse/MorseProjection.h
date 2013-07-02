/* MorseProjecetion.h */

#ifndef OMPL_EXTENSION_MORSE_PROJECTION_
#define OMPL_EXTENSION_MORSE_PROJECTION_

#include "ompl/extensions/morse/MorseStateSpace.h"

namespace ompl
{
    namespace base
    {

        class MorseProjection : public ProjectionEvaluator
        {
        public:
            MorseProjection(const StateSpacePtr &space);
            
            void setup(void);
            
            virtual unsigned int getDimension(void) const;
            
            virtual void defaultCellSizes(void);
            
            virtual void project(const State *state, EuclideanProjection &projection) const;
            
        protected:
            MorseStateSpace *space_;
        };
    }
}


#endif
