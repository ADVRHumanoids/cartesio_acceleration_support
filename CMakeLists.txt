cmake_minimum_required(VERSION 3.0)
project(cartesio_acceleration_support)

add_compile_options(-std=c++11)

find_package(cartesian_interface REQUIRED)
find_package(catkin REQUIRED COMPONENTS urdf)

catkin_package()

include_directories(src/
    ${catkin_INCLUDE_DIRS})

# cartesio_acceleration_support library
add_library(cartesio_acceleration_support SHARED
    src/Cartesian.cpp
    src/DynamicFeasibility.cpp
    src/Force.cpp
    src/ForceLimits.cpp
    src/FrictionCone.cpp
    src/Interaction.cpp
    src/Postural.cpp
    src/JointLimits.cpp
    src/VelocityLimits.cpp)

target_link_libraries(cartesio_acceleration_support
    ${catkin_LIBRARIES}
    ${cartesian_interface_LIBRARIES})

install(TARGETS cartesio_acceleration_support
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )


# force_optimization_node executable
add_executable(force_optimization_node src/ForceOptimizationNode.cpp)

target_link_libraries(force_optimization_node
    ${cartesian_interface_LIBRARIES}
    ${catkin_LIBRARIES}
    )

install(TARGETS force_optimization_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

add_subdirectory(bindings/python/)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)
