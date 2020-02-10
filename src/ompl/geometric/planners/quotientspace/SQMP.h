#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_SQMP_
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_SQMP_

#include <ompl/geometric/planners/quotientspace/algorithms/MultiQuotientDynamic.h>
#include <ompl/geometric/planners/quotientspace/algorithms/SQMPImpl.h>
/*



cd ~/MotionPlanningExplorerGUI/libs/ompl/build/
make -j4
sudo make install

cd ~/MotionPlanningExplorerGUI/build/
make -j4 planner_gui
./planner_gui  ../data/experiments/ICRA2020/02D_manipulator.xml



./planner_gui  ../data/experiments/contacts/46D_atlas_mountain.xml 


./planner_gui  ../data/experiments/ICRA2020/07D_planar_manipulator.xml
*/
namespace ompl
{
    namespace geometric
    {
       typedef ompl::geometric::MultiQuotientDynamic<ompl::geometric::SQMPImpl> SQMP;
    }
}
#endif