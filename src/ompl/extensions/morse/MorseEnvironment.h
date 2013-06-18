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
            
            struct WorldInfo
            {
                unsigned int rigidBodies_, allBodies_, conDim_;
                std::vector<double> conLow_, conHigh_, boundBoxes_;
            };
            
            // typedefs for functions that need to be supplied by the Python program
            typedef boost::function<WorldInfo(void)> getWorldInfoFn;
            typedef boost::function<void(void)> prepareStateReadFn;
            typedef boost::function<void(void)> finalizeStateWriteFn;
            typedef boost::function<void(const std::vector<double>&)> applyControlFn;
            typedef boost::function<void(const double)> worldStepFn;
            typedef boost::function<void(void)> endSimulationFn;

            ///** \brief The number of bodies that need to be considered
            //    part of the state when planning. This is not
            //    necessarily all the bodies in the environment.*/
            //unsigned int rigidBodies_;
            //
            ///** \brief The number of bodies that may engage in collision
            //    while planning even if not included in \e rigidBodies_*/
            //unsigned int allBodies_;
            
            WorldInfo wi_;
            
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
            prepareStateReadFn prepareStateRead;
            finalizeStateWriteFn finalizeStateWrite;
            applyControlFn applyControl;
            worldStepFn worldStep;
            endSimulationFn endSimulation;

            MorseEnvironment(const getWorldInfoFn &f1, const prepareStateReadFn &f2,
                const finalizeStateWriteFn &f3, const applyControlFn &f4, const worldStepFn &f5, const endSimulationFn &f6)
                 : stepSize_(0.05), maxControlSteps_(100), minControlSteps_(5),
                   getWorldInfo(f1), prepareStateRead(f2), finalizeStateWrite(f3),
                   applyControl(f4), worldStep(f5), endSimulation(f6)
            {
                wi_ = getWorldInfo();
                positions = new double[3*wi_.rigidBodies_];
                linVelocities = new double[3*wi_.rigidBodies_];
                angVelocities = new double[3*wi_.rigidBodies_];
                quaternions = new double[4*wi_.rigidBodies_];
            }

            ~MorseEnvironment(void)
            {
                delete [] positions;
                delete [] linVelocities;
                delete [] angVelocities;
                delete [] quaternions;
                endSimulation();
            }

            ///** \brief Number of parameters (double values) needed to specify a control input */
            //unsigned int getControlDimension(void) const
            //{
            //    return conDim_;
            //}

            /** \brief Get the control bounds -- the bounding box in which to sample controls */
            void getControlBounds(std::vector<double> &lower, std::vector<double> &upper) const;
            
            void getBoundBox(double result[24], const unsigned int obj) const;

            void getPosition(double result[3], const unsigned int obj) const;
            void getLinearVelocity(double result[3], const unsigned int obj) const;
            void getAngularVelocity(double result[3], const unsigned int obj) const;
            void getQuaternion(double result[4], const unsigned int obj) const;

            void setPosition(const unsigned int obj, const double pos[3]);
            void setLinearVelocity(const unsigned int obj, const double lin[3]);
            void setAngularVelocity(const unsigned int obj, const double ang[3]);
            void setQuaternion(const unsigned int obj, const double rot[4]);
            
        private:
            //unsigned int conDim_;
            //std::vector<double> boundsLow_, boundsHigh_;
            double *positions, *linVelocities, *angVelocities;   // in groups of 3
            double *quaternions;   // in groups of 4
        };
    }
}

#endif
