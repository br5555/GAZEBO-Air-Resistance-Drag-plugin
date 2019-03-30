cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (CMAKE_CXX_STANDARD 11)

find_package(gazebo REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}                    
                    ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARAY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(gazebo_air_resistance_plugin SHARED gazebo_air_resistance_plugin.cc )
target_link_libraries(gazebo_air_resistance_plugin ${GAZEBO_LIBRARIES} )

add_library(gazebo_air_resistance_plugin_collision SHARED gazebo_air_resistance_plugin_collision.cpp )
target_link_libraries(gazebo_air_resistance_plugin_collision ${GAZEBO_LIBRARIES} )