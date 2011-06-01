option(OMPL_CONTRIB_SAMPLECONTRIB "Enable SampleContrib" ON)

if(OMPL_CONTRIB_SAMPLECONTRIB)
    file(GLOB_RECURSE OMPL_SAMPLECONTRIB_SOURCE_CODE src/*.cpp)
    set(OMPL_SOURCE_CODE ${OMPL_SOURCE_CODE} ${OMPL_SAMPLECONTRIB_SOURCE_CODE} PARENT_SCOPE)

    # doesn't work yet; need to rearrange cmake test code
    # add_ompl_test(test_samplecontrib tests/test_sample_contrib.cpp)
endif()