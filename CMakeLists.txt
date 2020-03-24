project(cartesio_acceleration_support)

add_compile_options(-std=c++11)

find_package(cartesian_interface REQUIRED)
find_package(fmt REQUIRED)

add_library(cartesio_acceleration_support SHARED
    src/Cartesian.cpp
    src/Interaction.cpp
    src/Postural.cpp
    src/JointLimits.cpp
    src/VelocityLimits.cpp)
target_include_directories(cartesio_acceleration_support PUBLIC ${cartesian_interface_INCLUDE_DIRS})
target_link_libraries(cartesio_acceleration_support  ${cartesian_interface_LIBRARIES} fmt::fmt-header-only)
install(TARGETS cartesio_acceleration_support
        DESTINATION lib
        )
