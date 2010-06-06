macro(glob_headers _var _dir)
	file(GLOB_RECURSE ${_var} "${_dir}/*.h")
	set(${_var} ${${_var}} PARENT_SCOPE)
endmacro(glob_headers)

file(GLOB_RECURSE OMPL_SOURCE_CODE *.cpp)
# \todo hack alert: need to figure why base/planner.h is needed for 
# dynamic and kinematic python modules
file(GLOB OMPL_BASE_PLANNER_H base/planner.h)
set(OMPL_BASE_PLANNER_H ${OMPL_BASE_PLANNER_H} PARENT_SCOPE)

glob_headers(OMPL_BASE_HEADERS base)
glob_headers(OMPL_DYNAMIC_HEADERS dynamic)
glob_headers(OMPL_KINEMATIC_HEADERS kinematic)
glob_headers(OMPL_UTIL_HEADERS util)

#glob_headers(OMPL_DATASTRUCTURES_HEADERS datastructures)
# need to remove this header for Py++
#list(REMOVE_ITEM OMPL_DATASTRUCTURES_HEADERS
#	"${CMAKE_CURRENT_SOURCE_DIR}/datastructures/GridAux.h")
#set(OMPL_DATASTRUCTURES_HEADERS ${OMPL_DATASTRUCTURES_HEADERS} PARENT_SCOPE)

if(ROS)  
	rosbuild_add_library(ompl ${OMPL_SOURCE_CODE})
	rosbuild_add_compile_flags(ompl -Wextra)
	rosbuild_link_boost(ompl thread date_time)
else(ROS)
	add_library(ompl SHARED ${OMPL_SOURCE_CODE})
	target_link_libraries(ompl
		${Boost_THREAD_LIBRARY}
		${Boost_DATE_TIME_LIBRARY}
	)
endif(ROS)
