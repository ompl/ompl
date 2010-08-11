file(GLOB_RECURSE OMPL_SOURCE_CODE *.cpp)

file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" VERSION_DIR)
configure_file("${VERSION_DIR}/version.h.in" "${VERSION_DIR}/version.h")

if(ROS)  
	rosbuild_add_library(ompl ${OMPL_SOURCE_CODE})
	rosbuild_add_compile_flags(ompl -Wextra)
	rosbuild_link_boost(ompl thread date_time ${PROF_LIBRARY})
else(ROS)

        if (PROF_LIBRARY)
	  if (CMAKE_COMPILER_IS_GNUCXX)
	    add_definitions(-fPIC)
	  endif()
	  add_library(ompl STATIC ${OMPL_SOURCE_CODE})
	else (PROF_LIBRARY)
	  add_library(ompl SHARED ${OMPL_SOURCE_CODE})
	endif (PROF_LIBRARY)

	target_link_libraries(ompl
		${Boost_THREAD_LIBRARY}
		${Boost_DATE_TIME_LIBRARY}
		${PROF_LIBRARY}
	)
endif(ROS)

install(TARGETS ompl LIBRARY DESTINATION lib/ ARCHIVE DESTINATION lib/)
