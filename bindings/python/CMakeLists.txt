cmake_minimum_required(VERSION 3.5)

find_package(pybind11 QUIET)
message(STATUS "**********************************Check for pybind11")
if(${pybind11_FOUND})

    # get python install path
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "if True:
            from distutils import sysconfig as sc
            print(sc.get_python_lib(prefix='', plat_specific=True))"
        OUTPUT_VARIABLE PYTHON_SITE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "Python install dir: ${PYTHON_SITE}")

    message(STATUS "+++++++++++++++++++++++++++++++Compiling python bindings")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(tasks tasks.cpp)
    target_link_libraries(tasks PRIVATE cartesio_acceleration_support)
    install(TARGETS tasks
        DESTINATION ${PYTHON_SITE}/cartesio_acceleration_support)

    file(WRITE ${CMAKE_BINARY_DIR}/__init__.py "")
    install(FILES  ${CMAKE_BINARY_DIR}/__init__.py DESTINATION ${PYTHON_SITE}/cartesio_acceleration_support)

else()
    message(STATUS "-----------------------------------pybind not found")
endif()
