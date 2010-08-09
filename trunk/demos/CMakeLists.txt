

if(ROS)
  
  macro(add_ompl_demo demo_name)
    rosbuild_add_executable(${ARGV})
    target_link_libraries(${demo_name} ompl)
    rosbuild_link_boost(${demo_name} filesystem system thread date_time)
  endmacro(add_ompl_demo)

else(ROS)
 
  macro(add_ompl_demo demo_name)
    add_executable(${ARGV})
    target_link_libraries(${demo_name} 
      ompl
      ${Boost_FILESYSTEM_LIBRARY}
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_THREAD_LIBRARY}
      ${Boost_DATE_TIME_LIBRARY})
  endmacro(add_ompl_demo)
  
endif(ROS)


add_ompl_demo(demo_RigidBodyPlanning RigidBodyPlanning.cpp)
add_ompl_demo(demo_RigidBodyPlanningWithControls RigidBodyPlanningWithControls.cpp)
