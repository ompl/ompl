#ifndef GRIDDECOMPOSITION_H
#define GRIDDECOMPOSITION_H

#include <cstdlib>
#include <valarray>
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/State.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/datastructures/Grid.h"

namespace ompl
{
    namespace control
    {
        class GridDecomposition : public Decomposition
        {
        public:
            /* In some areas of this class, we are assuming a 2-dimensional grid. This will change. */
            GridDecomposition(const int len, const int dim, const base::RealVectorBounds& b);

            virtual ~GridDecomposition()
            {
            }

            virtual double getRegionVolume(const int rid) const
            {
                return cellVolume;
            }

            /* This implementation requires time linear with the number of regions.
             * We can do constant time if we know the dimension offline (oopsmp-syclop has cases for 2 and 3),
             * but can we beat linear time with arbitrary dimension? */
            virtual void getNeighbors(const int rid, std::vector<int>& neighbors);

            virtual int locateRegion(const base::State* s);

        protected:
            virtual bool areNeighbors(int r, int s);

        private:
            /* Convert a region ID to a grid coordinate, which is a vector of length equivalent
             * to the dimension of the grid. */
            void regionToCoord(int rid, std::vector<int>& coord);

            const int length;
            const int dimension;
            double cellVolume;
        };
    }
}
#endif
