option(OMPL_BUILD_DEMOS "Build OMPL demos" ON)

if (OMPL_BUILD_DEMOS)

  if(OMPL_IN_ROS)
    
    macro(add_ompl_demo demo_name)
      rosbuild_add_executable(${ARGV})
      target_link_libraries(${demo_name} ompl)
      rosbuild_link_boost(${demo_name} filesystem system thread date_time)
    endmacro(add_ompl_demo)
    
  else(OMPL_IN_ROS)
    
    macro(add_ompl_demo demo_name)
      add_executable(${ARGV})
      target_link_libraries(${demo_name} 
	ompl
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY}
	${Boost_DATE_TIME_LIBRARY})
    endmacro(add_ompl_demo)
    
  endif(OMPL_IN_ROS)
  
  
  add_ompl_demo(demo_RigidBodyPlanning RigidBodyPlanning.cpp)
  add_ompl_demo(demo_RigidBodyPlanningWithControls RigidBodyPlanningWithControls.cpp)
  add_ompl_demo(demo_RigidBodyPlanningWithIntegrationAndControls RigidBodyPlanningWithIntegrationAndControls.cpp)
  
  if (OMPL_EXTENSION_ODE)
    add_ompl_demo(demo_ODERigidBodyPlanning ODERigidBodyPlanning.cpp)
  endif()

endif(OMPL_BUILD_DEMOS)
