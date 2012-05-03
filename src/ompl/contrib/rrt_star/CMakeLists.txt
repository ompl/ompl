# The default should be "ON" for contributions. Since this sample
# contribution doesn't do anything useful it's disabled by default.
option(OMPL_CONTRIB_RRTSTAR_CONTRIB "Enable RRTstar contrib" ON)

if(OMPL_CONTRIB_RRTSTAR_CONTRIB)
    file(GLOB_RECURSE OMPL_CONTRIB_RRTSTAR_CONTRIB_SOURCE_CODE src/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_CONTRIB_RRTSTAR_CONTRIB_SOURCE_CODE} PARENT_SCOPE)

    add_definitions(-DBOOST_TEST_DYN_LINK)
    add_ompl_test(test_rrt_star tests/test_rrt_star.cpp)
endif()
