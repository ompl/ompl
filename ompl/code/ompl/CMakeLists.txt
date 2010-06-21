file(GLOB_RECURSE OMPL_SOURCE_CODE *.cpp)

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
      
        install(DIRECTORY ../ompl DESTINATION include/ FILES_MATCHING PATTERN "*.h" REGEX "\\.svn" EXCLUDE REGEX "/src" EXCLUDE)
	install(TARGETS ompl LIBRARY DESTINATION lib/)
endif(ROS)
