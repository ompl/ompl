add_subdirectory(ompl)
install(DIRECTORY ompl DESTINATION include/ 
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".DS_Store" EXCLUDE
	PATTERN ".svn" EXCLUDE
	REGEX "/src$" EXCLUDE)
