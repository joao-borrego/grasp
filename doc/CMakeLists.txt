find_package(Doxygen)

if (DOXYGEN_FOUND)
  configure_file(${CMAKE_SOURCE_DIR}/doc/grasp.in
    ${CMAKE_BINARY_DIR}/grasp.dox @ONLY)

  add_custom_target(doc

    # Generate the API documentation
    ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/grasp.dox
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/logo.png
            ${CMAKE_BINARY_DIR}/doxygen/html
    COMMAND cp ${CMAKE_SOURCE_DIR}/doc/search.js
            ${CMAKE_BINARY_DIR}/doxygen/html/search

    COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif()

