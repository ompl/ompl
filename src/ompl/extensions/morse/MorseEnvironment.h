/* MorseEnvironment.h */

#ifndef OMPL_EXTENSION_MORSE_ENVIRONMENT_
#define OMPL_EXTENSION_MORSE_ENVIRONMENT_

#include "ompl/config.h"
#if OMPL_EXTENSION_MORSE == 0
#  error MORSE extension not built
#endif

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"

#include "boost/thread.hpp"

#include <limits>
#include <vector>

namespace ompl
{
    namespace base
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::MorseEnvironment */
        OMPL_CLASS_FORWARD(MorseEnvironment);
        /// @endcond

        /** \class ompl::base::MorseEnvironmentPtr
            \brief A boost shared pointer wrapper for ompl::base::MorseEnvironment */
        
        /** \brief This class contains the MORSE constructs OMPL needs to know about when planning. */
        class MorseEnvironment
        {
        public:
            
            const unsigned int controlDim_;
            
            const std::vector<double> controlBounds_;
            
            const unsigned int rigidBodies_;
            
            std::vector<double> positionBounds_, linvelBounds_, angvelBounds_;
            
            /** \brief The simulation step size */
            double stepSize_;

            /** \brief The minimum number of times a control is applies in sequence */
            unsigned int minControlSteps_;

            /** \brief The maximum number of times a control is applies in sequence */
            unsigned int maxControlSteps_;
            
            /** \brief Lock to use when performing simulations in the world */
            mutable boost::mutex mutex_;

            MorseEnvironment(const unsigned int controlDim, const std::vector<double> &controlBounds,
                const unsigned int rigidBodies, const std::vector<double> &positionBounds,
                const std::vector<double> &linvelBounds, const std::vector<double> &angvelBounds,
                const double stepSize, const unsigned int minControlSteps, const unsigned int maxControlSteps)
                 : controlDim_(controlDim), controlBounds_(controlBounds), rigidBodies_(rigidBodies),
                   positionBounds_(positionBounds), linvelBounds_(linvelBounds), angvelBounds_(angvelBounds),
                   stepSize_(stepSize), minControlSteps_(minControlSteps), maxControlSteps_(maxControlSteps)
            {
                for (unsigned int i = 0; i < positionBounds_.size(); i++)
                {
                    if (positionBounds_[i]==std::numeric_limits<double>::infinity())
                        positionBounds_[i] = std::numeric_limits<double>::max();
                    else if (positionBounds_[i]==-std::numeric_limits<double>::infinity())
                        positionBounds_[i] = std::numeric_limits<double>::min();
                }
                for (unsigned int i = 0; i < linvelBounds_.size(); i++)
                {
                    if (linvelBounds_[i]==std::numeric_limits<double>::infinity())
                        linvelBounds_[i] = std::numeric_limits<double>::max()/2;
                    else if (linvelBounds_[i]==-std::numeric_limits<double>::infinity())
                        linvelBounds_[i] = -std::numeric_limits<double>::max()/2;
                }
                for (unsigned int i = 0; i < angvelBounds_.size(); i++)
                {
                    if (angvelBounds_[i]==std::numeric_limits<double>::infinity())
                        angvelBounds_[i] = std::numeric_limits<double>::max()/2;
                    else if (angvelBounds_[i]==-std::numeric_limits<double>::infinity())
                        angvelBounds_[i] = -std::numeric_limits<double>::max()/2;
                }
            }

            ~MorseEnvironment(void)
            {
            }

            /** \brief Get the control bounds -- the bounding box in which to sample controls */
            void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;

            // These IPC functions are left to be implemented in Python
            virtual void readState(State *state) = 0;
            virtual void writeState(const State *state) = 0;
            virtual void applyControl(const std::vector<double> &control) = 0;
            virtual void worldStep(const double dur) = 0;
        };
    }
}

#endif
