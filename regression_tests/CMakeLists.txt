add_executable(regression_test RegressionTest.cpp)
target_link_libraries(regression_test
    ompl
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    ${Boost_DATE_TIME_LIBRARY})
