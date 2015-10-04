cmake_minimum_required(VERSION 3.2)
project(KinectToPCL)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.7 REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
    ${PCL_LIBRARY_DIRS}
)
ADD_DEFINITIONS(
    ${PCL_DEFINITIONS}
)

set(LIBRARIES
    ${OPENCV_LIBS}
    ${OPENCV_LIBRARIES}
)
set(SOURCE_FILES main.cpp LineFilter.cpp LineFilter.h)
add_executable(KinectToPCL ${SOURCE_FILES}
            PointCloudBuilder.cpp
            PointCloudBuilder.h
            common.h
            KinectParameters.h
            KinectParameters.h
            KinectParameters.cpp)

target_link_libraries(KinectToPCL
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
