cmake_minimum_required(VERSION 3.16)
project(CoppeliaSimTutorial)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(Python3 3.9 EXACT COMPONENTS Interpreter Development)
find_package(AndreiUtils REQUIRED COMPONENTS eigen python)

option(WITH_PCL_OPTIMIZATION_FLAGS "Use optimization flags used by PCL to compile a compatible interface to EIGEN types" ON)
set(PCL_OPTIMIZATION_FLAGS)
if (${WITH_PCL_OPTIMIZATION_FLAGS})
    find_package(PCL REQUIRED COMPONENTS common)
    get_target_property(OUTPUT pcl_common INTERFACE_COMPILE_OPTIONS)
    set(PCL_OPTIMIZATION_FLAGS ${OUTPUT})
    message(STATUS "PCL optimization flags = ${PCL_OPTIMIZATION_FLAGS}")
endif ()

add_executable(CoppeliaSimTutorial main.cpp)
target_link_libraries(CoppeliaSimTutorial ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep pthread)
target_compile_options(CoppeliaSimTutorial PUBLIC ${PCL_OPTIMIZATION_FLAGS})

add_executable(TrajectoryPlotting trajectoryPlotting.cpp)
target_link_libraries(TrajectoryPlotting ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep pthread)
target_compile_options(TrajectoryPlotting PUBLIC ${PCL_OPTIMIZATION_FLAGS})

add_executable(TestAndreiUtils main.cpp)
target_link_libraries(TestAndreiUtils ${AndreiUtils_LIBRARY})