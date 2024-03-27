cmake_minimum_required(VERSION 3.0)
project(cartesio_acceleration_support)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS 
                    roscpp
                    urdf
                    geometry_msgs
                    message_generation)
find_package(cartesian_interface REQUIRED)
find_package(matlogger2 REQUIRED)


include_directories(
    src
    include
    ${catkin_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS})
    
add_message_files(
    FILES
    SetForceLimits.msg)
    
generate_messages(
    DEPENDENCIES
    geometry_msgs)
    
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp message_runtime geometry_msgs
    )

# cartesio_acceleration_support library
add_library(cartesio_acceleration_support SHARED
    src/Cartesian.cpp
    src/DynamicFeasibility.cpp
    src/Force.cpp
    src/ForceLimits.cpp
    src/FrictionCone.cpp
    src/CoP.cpp
    src/CoM.cpp
    src/Interaction.cpp
    #src/PosturalInteraction.cpp
    src/Postural.cpp
    src/JointLimits.cpp
    src/TorqueLimits.cpp
    src/VelocityLimits.cpp
#    src/ForceOptimization.cpp
    src/NormalTorque.cpp)

add_dependencies(cartesio_acceleration_support ${PROJECT_NAME}_generate_messages)

target_link_libraries(cartesio_acceleration_support
    PUBLIC
    ${catkin_LIBRARIES}
    ${cartesian_interface_LIBRARIES}
    matlogger2::matlogger2)

install(TARGETS cartesio_acceleration_support
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# force_optimization_node executable
#add_executable(force_optimization_node src/ForceOptimizationNode.cpp)
#target_link_libraries(force_optimization_node ${catkin_LIBRARIES}
#    yaml-cpp cartesio_acceleration_support)
#install(TARGETS force_optimization_node
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

add_subdirectory(bindings/python/)

install(DIRECTORY examples/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples)

# cartesian impedance solver
#add_library(CartesianInterfaceSolverImpedance SHARED
#    src/cartesian_impedance_solver.cpp)

#target_link_libraries(CartesianInterfaceSolverImpedance
#    PRIVATE
#    matlogger2::matlogger2
#    ${cartesian_interface_LIBRARIES})

#install(TARGETS CartesianInterfaceSolverImpedance
#    DESTINATION lib)
