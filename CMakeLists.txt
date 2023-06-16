cmake_minimum_required(VERSION 3.16)
project(CoppeliaSimTutorial)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

option(WITH_PCL_OPTIMIZATION_FLAGS "Use optimization flags used by PCL to compile a compatible interface to EIGEN types" ON)
set(PCL_OPTIMIZATION_FLAGS)
if (${WITH_PCL_OPTIMIZATION_FLAGS})
    find_package(PCL REQUIRED COMPONENTS common)
    get_target_property(OUTPUT pcl_common INTERFACE_COMPILE_OPTIONS)
    set(PCL_OPTIMIZATION_FLAGS ${OUTPUT})
    message(STATUS "PCL optimization flags = ${PCL_OPTIMIZATION_FLAGS}")
endif ()

find_package(VisualPerception REQUIRED COMPONENTS realsense darknet openpose markerDetectionLib pcl pcl_realsense)
message("${VisualPerception_LIBRARY}")
add_executable(SurfaceDataProcessing surfaceDataProcessing.cpp)
target_link_libraries(SurfaceDataProcessing ${VisualPerception_LIBRARY})
target_compile_options(SurfaceDataProcessing PUBLIC ${PCL_OPTIMIZATION_FLAGS})


find_package(Eigen3 REQUIRED)
find_package(Python3 3.10 EXACT COMPONENTS Interpreter Development)
find_package(AndreiUtils REQUIRED COMPONENTS eigen python json)
# find_package(PCL 1.2 REQUIRED COMPONENTS common io)

add_executable(CoppeliaSimTutorial main.cpp surfaceProximity.cpp)
target_link_libraries(CoppeliaSimTutorial ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep pthread)
target_compile_options(CoppeliaSimTutorial PUBLIC ${PCL_OPTIMIZATION_FLAGS})

add_executable(TrajectoryPlotting trajectoryPlotting.cpp)
target_link_libraries(TrajectoryPlotting ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep)
target_compile_options(TrajectoryPlotting PUBLIC ${PCL_OPTIMIZATION_FLAGS})

add_executable(Test test.cpp)
target_link_libraries(Test ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep pthread)
target_compile_options(Test PUBLIC ${PCL_OPTIMIZATION_FLAGS})

add_executable(TestAndreiUtils main.cpp)
message("${AndreiUtils_LIBRARY}")
target_link_libraries(TestAndreiUtils ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep pthread)
target_compile_options(TestAndreiUtils PUBLIC ${PCL_OPTIMIZATION_FLAGS})


add_executable(VoxelExtraction voxelExtraction.cpp)
message("${PCL_LIBRARIES}")
target_link_libraries(VoxelExtraction ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep ${PCL_LIBRARIES})
target_include_directories(VoxelExtraction
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}
        )
target_link_directories(VoxelExtraction PUBLIC ${PCL_LIBRARY_DIRS})
target_compile_options(VoxelExtraction PUBLIC ${PCL_OPTIMIZATION_FLAGS})

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
set(TARGET SurfaceExtraction)
add_executable(${TARGET} surfaceExtractor.cpp)
target_include_directories(${TARGET} PUBLIC ${PCL_INCLUDE_DIRS})
target_link_directories(${TARGET} PUBLIC ${PCL_LIBRARY_DIRS})
target_compile_definitions(${TARGET} PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(${TARGET} ${AndreiUtils_LIBRARY} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY} ${OPENGL_gl_LIBRARY} ${GLUT_LIBRARIES} glfw GLU glut GL pthread)

add_executable(surfaceProximity surfaceProximity.cpp)
target_link_libraries(surfaceProximity ${AndreiUtils_LIBRARY} Eigen3::Eigen dqrobotics dqrobotics-interface-vrep ${PCL_LIBRARIES})
target_include_directories(surfaceProximity
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${PCL_INCLUDE_DIRS}
        )
target_link_directories(surfaceProximity PUBLIC ${PCL_LIBRARY_DIRS})
target_compile_options(surfaceProximity PUBLIC ${PCL_OPTIMIZATION_FLAGS})