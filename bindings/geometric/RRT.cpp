#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/Planner.h"
#include "ompl/geometric/planners/rrt/RRT.h"


namespace nb = nanobind;

void initRRT(nb::module_& m) {  
    nb::class_<ompl::base::Planner>(m, "Planner");   
    nb::class_<ompl::geometric::RRT, ompl::base::Planner>(m, "_RRT")
        .def(nb::init<const ompl::base::SpaceInformationPtr&>());
    
    m.def("RRT", 
          [](const ompl::base::SpaceInformationPtr& si) {
              return std::make_shared<ompl::geometric::RRT>(si);
          }, 
          nb::rv_policy::reference);
}