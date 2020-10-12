# build_and_install_xacro_files(<file> [<file> ...])
#
# Builds urdf from given xacro files and installs them into devel
# and install space. The urdf will be generated in the binary build
# dir of cmake and then installed into urdf/. For example, an xacro
# file named urdf/robot.xacro will be installed to urdf/robot.urdf.
function(build_and_install_xacro_files)
    foreach(input ${ARGN})
        if(${input} MATCHES "(.*)[.]xacro$")
            # Match path without xacro extension
            set(path ${CMAKE_MATCH_1})
            # Get the name of the xacro file without directories
            get_filename_component(name ${path} NAME)
            message(STATUS "${CMAKE_BINARY_DIR}")

            xacro_add_files(${path}.xacro
                TARGET ${name}_xacro
                OUTPUT ${CMAKE_BINARY_DIR}/${name}.urdf
            )
            message(STATUS "installing ${CMAKE_BINARY_DIR}/${name}.urdf in share/${PROJECT_NAME}")
            install(FILES ${CMAKE_BINARY_DIR}/${name}.urdf
                DESTINATION share/${PROJECT_NAME}/urdf
            )

        endif()
    endforeach()
endfunction()
