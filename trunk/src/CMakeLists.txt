add_subdirectory(ompl)
install(DIRECTORY ompl DESTINATION include/ 
	FILES_MATCHING PATTERN "*.h")
