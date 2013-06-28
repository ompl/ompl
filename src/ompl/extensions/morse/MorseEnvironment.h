/* MorseEnvironment.h */

#ifndef OMPL_EXTENSION_MORSE_ENVIRONMENT_
#define OMPL_EXTENSION_MORSE_ENVIRONMENT_

#include "ompl/config.h"
#if OMPL_EXTENSION_MORSE == 0
#  error MORSE extension not built
#endif

#include "ompl/util/ClassForward.h"

#include <vector>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

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
            
            const unsigned int rigidBodies_;
            
            const unsigned int controlDim_;
            
            const std::vector<double> controlBounds_, positionBounds_, linvelBounds_, angvelBounds_;
            
            std::vector<double> positions, linVelocities, angVelocities;   // in groups of 3
            std::vector<double> quaternions;   // in groups of 4
            
            /** \brief The simulation step size */
            double stepSize_;

            /** \brief The maximum number of times a control is applies in sequence */
            unsigned int maxControlSteps_;

            /** \brief The minimum number of times a control is applies in sequence */
            unsigned int minControlSteps_;

            /** \brief Lock to use when performing simulations in the world */
            mutable boost::mutex mutex_;

            MorseEnvironment(const unsigned int rigidBodies, const unsigned int controlDim,
                const std::vector<double> &controlBounds, const std::vector<double> &positionBounds,
                const std::vector<double> &linvelBounds, const std::vector<double> &angvelBounds)
                 : rigidBodies_(rigidBodies), controlDim_(controlDim), controlBounds_(controlBounds),
                   positionBounds_(positionBounds), linvelBounds_(linvelBounds), angvelBounds_(angvelBounds),
                   positions(3*rigidBodies), linVelocities(3*rigidBodies), angVelocities(3*rigidBodies), quaternions(4*rigidBodies),
                   stepSize_(1.0/60), maxControlSteps_(100), minControlSteps_(5)
            {
            }

            ~MorseEnvironment(void)
            {
            }

            /** \brief Get the control bounds -- the bounding box in which to sample controls */
            void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;

            virtual void prepareStateRead(void) = 0;
            double *getPosition(const unsigned int obj) const;
            double *getLinearVelocity(const unsigned int obj) const;
            double *getAngularVelocity(const unsigned int obj) const;
            double *getQuaternion(const unsigned int obj) const;

            virtual void finalizeStateWrite(void) = 0;
            void setPosition(const unsigned int obj, const double pos[3]);
            void setLinearVelocity(const unsigned int obj, const double lin[3]);
            void setAngularVelocity(const unsigned int obj, const double ang[3]);
            void setQuaternion(const unsigned int obj, const double rot[4]);
            
            virtual void applyControl(const std::vector<double> &control) = 0;
            
            virtual void worldStep(const double dur) = 0;
        };
    }
}

#endif
