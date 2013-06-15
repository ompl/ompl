/* MorseEnvironment.h */

#ifndef OMPL_EXTENSION_MORSE_ENVIRONMENT_
#define OMPL_EXTENSION_MORSE_ENVIRONMENT_

#include "ompl/config.h"
#if OMPL_EXTENSION_MORSE == 0
#  error MORSE extension not built
#endif

#include "ompl/util/ClassForward.h"

#include <vector>
#include <string>
#include <map>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

namespace ompl
{
    namespace control
    {

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::control::MorseEnvironment */
        OMPL_CLASS_FORWARD(MorseEnvironment);
        /// @endcond

        /** \class ompl::control::MorseEnvironmentPtr
            \brief A boost shared pointer wrapper for ompl::control::MorseEnvironment */

        /** \brief This class contains the MORSE constructs OMPL needs to know about when planning. */
        class MorseEnvironment
        {
        public:
            
            // typedefs for functions that need to be supplied by the Python program
            typedef boost::function<void(void)> getWorldInfoFn;
            typedef boost::function<void(double[24], unsigned int)> getBoundBoxFn;
            typedef boost::function<void(void)> prepareStateReadFn;
            typedef boost::function<void(void)> finalizeStateWriteFn;
            typedef boost::function<void(const double*)> applyControlFn;
            typedef boost::function<void(const double)> worldStepFn;
            typedef boost::function<void(void)> endSimulationFn;

            /** \brief The number of bodies that need to be considered
                part of the state when planning. This is not
                necessarily all the bodies in the environment.*/
            unsigned int rigidBodies_;
            
            /** \brief The number of bodies that may engage in collision
                while planning even if not included in \e rigidBodies_*/
            unsigned int allBodies_;
            
            /** \brief The simulation step size */
            double stepSize_;

            /** \brief The maximum number of times a control is applies in sequence */
            unsigned int maxControlSteps_;

            /** \brief The minimum number of times a control is applies in sequence */
            unsigned int minControlSteps_;

            /** \brief Lock to use when performing simulations in the world */
            mutable boost::mutex mutex_;
            
            // IPC functions
            getWorldInfoFn getWorldInfo;
            getBoundBoxFn getBoundBox;
            prepareStateReadFn prepareStateRead;
            finalizeStateWriteFn finalizeStateWrite;
            applyControlFn applyControl;
            worldStepFn worldStep;
            endSimulationFn endSimulation;

            MorseEnvironment(getWorldInfoFn f1, getBoundBoxFn f2, prepareStateReadFn f3,
                finalizeStateWriteFn f4, applyControlFn f5, worldStepFn f6, endSimulationFn f7)
                 : stepSize_(0.05), maxControlSteps_(100), minControlSteps_(5),
                   getWorldInfo(f1), getBoundBox(f2), prepareStateRead(f3), finalizeStateWrite(f4),
                   applyControl(f5), worldStep(f6), endSimulation(f7)
            {
                getWorldInfo(); // initializes rigidBodies_, allBodies_, conDim_, boundsLow_, and boundsHigh_
                positions = new double[3*rigidBodies_];
                linVelocities = new double[3*rigidBodies_];
                angVelocities = new double[3*rigidBodies_];
                quaternions = new double[4*rigidBodies_];
            }

            ~MorseEnvironment(void)
            {
                delete [] positions;
                delete [] linVelocities;
                delete [] angVelocities;
                delete [] quaternions;
                endSimulation();
            }

            /** \brief Number of parameters (double values) needed to specify a control input */
            unsigned int getControlDimension(void) const
            {
                return conDim_;
            }

            /** \brief Get the control bounds -- the bounding box in which to sample controls */
            void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;

            void getPosition(double result[3], unsigned int obj) const;
            void getLinearVelocity(double result[3], unsigned int obj) const;
            void getAngularVelocity(double result[3], unsigned int obj) const;
            void getQuaternion(double result[4], unsigned int obj) const;

            void setPosition(unsigned int obj, double pos[3]);
            void setLinearVelocity(unsigned int obj, double lin[3]);
            void setAngularVelocity(unsigned int obj, double ang[3]);
            void setQuaternion(unsigned int obj, double rot[4]);
            
        private:
            unsigned int conDim_;
            std::vector<double> boundsLow_, boundsHigh_;
            double *positions, *linVelocities, *angVelocities;   // in groups of 3
            double *quaternions;   // in groups of 4
        };
    }
}

#endif
