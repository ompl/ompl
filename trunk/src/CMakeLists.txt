add_subdirectory(ompl)
install(DIRECTORY ompl DESTINATION include/ 
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".DS_Store" EXCLUDE
	REGEX "\\.svn" EXCLUDE
	REGEX "/src$" EXCLUDE)
