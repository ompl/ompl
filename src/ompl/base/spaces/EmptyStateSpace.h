#pragma once
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        class EmptyStateSpace : public RealVectorStateSpace
        {
        public:
            EmptyStateSpace()
              : RealVectorStateSpace(0)
            {
                setName("EmptySpace");
            }
            ~EmptyStateSpace() override = default;

            double getMeasure() const
            {
              return 0;
            }

            void setup() override
            {
              return;
            }

            unsigned int getDimension() const override
            {
              return 0;
            }


            double getMaximumExtent() const override
            {
              return 0;
            }

        };
    }
}


