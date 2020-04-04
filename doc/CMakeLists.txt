if(DOXYGEN_FOUND AND NOT CMAKE_VERSION VERSION_LESS 3.9)
    if(UNIX)
        execute_process(COMMAND date "+%b %d, %Y"
            OUTPUT_VARIABLE OMPL_RELEASE_DATE
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        set(OMPL_RELEASE_DATE "${OMPL_RELEASE_DATE}" PARENT_SCOPE)
    endif()
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/download.md.in"
        "${CMAKE_BINARY_DIR}/doc/ompl_download.md")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/mainpage.md.in"
        "${CMAKE_BINARY_DIR}/doc/mainpage.md")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/ompl.svg" OMPLSVG)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/cforest.svg" CFORESTSVG)
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/images/cforest_sampler.svg" CFOREST_SAMPLERSVG)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/api_overview.md.in"
        "${CMAKE_BINARY_DIR}/doc/api_overview.md")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/CForest.md.in"
        "${CMAKE_BINARY_DIR}/doc/CForest.md")

    include(DoxygenSettings)
    set(DOXYGEN_PROJECT_NAME "ompl")
    set(DOXYGEN_EXCLUDE
        "${CMAKE_BINARY_DIR}/doc/omplapp_download.md")
    set(DOXYGEN_EXAMPLE_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/../demos"
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown/code")
    set(DOXYGEN_HTML_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/header.html")
    set(DOXYGEN_HTML_FOOTER "${CMAKE_CURRENT_SOURCE_DIR}/footer.html")
    set(DOXYGEN_HTML_OUTPUT "${CMAKE_BINARY_DIR}/ompl_doc")
    set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_HTML_OUTPUT}/ompl.tag")
    doxygen_add_docs(ompl_doc
        "${CMAKE_CURRENT_SOURCE_DIR}/markdown"
        "${CMAKE_CURRENT_SOURCE_DIR}/../demos"
        "${CMAKE_CURRENT_SOURCE_DIR}/../src/ompl"
        "${CMAKE_BINARY_DIR}/doc"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
    # hide OMPL.app-related content
    set(OMPL_CSS ".omplapp{display:none;}")
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ompl.css" "${DOXYGEN_HTML_OUTPUT}/ompl.css")
    add_custom_command(TARGET ompl_doc POST_BUILD
        COMMAND cp -r images ieee-ram-2012-ompl.pdf "${CMAKE_BINARY_DIR}/install-ompl-ubuntu.sh" "${DOXYGEN_HTML_OUTPUT}"
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

install(FILES man/ompl_benchmark_statistics.1 man/plannerarena.1
    DESTINATION ${CMAKE_INSTALL_MANDIR}/man1 COMPONENT ompl)
