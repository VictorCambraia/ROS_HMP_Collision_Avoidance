cmake_minimum_required(VERSION 3.1)
include(${CMAKE_CURRENT_SOURCE_DIR}/../dqrobotics_dependencies.cmake)

find_package(Threads REQUIRED)

project(cpp_main_codes)


# add_executable(controller_cube_side
#     controller_cube_side.cpp
#     )


# TARGET_LINK_LIBRARIES(controller_cube_side
#     pthread
#     dqrobotics
#     dqrobotics-interface-vrep
#     qpOASES
# )

# add_executable(controller_test
#     controller_test.cpp
#     )


# TARGET_LINK_LIBRARIES(controller_test
#     pthread
#     dqrobotics
#     dqrobotics-interface-vrep
#     qpOASES
# )

add_executable(controller_human_avoidance
    controller_human_avoidance.cpp
    JacobianHMP.cpp
    )


TARGET_LINK_LIBRARIES(controller_human_avoidance
    pthread
    dqrobotics
    dqrobotics-interface-vrep
    qpOASES
)

TARGET_INCLUDE_DIRECTORIES(controller_human_avoidance
PRIVATE
"${CMAKE_CURRENT_SOURCE_DIR}"
)

# target_include_directories(Engine PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")