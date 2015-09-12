cmake_minimum_required(VERSION 3.2)
project(KinectToPCL)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(PCL 1.3 REQUIRED)
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
set(SOURCE_FILES main.cpp)
add_executable(KinectToPCL ${SOURCE_FILES}
            PointCloudBuilder.cpp
            PointCloudBuilder.h
            common.h
            KinectParameters.h
            KinectParameters.h
            KinectParameters.cpp
            IPointCloudDivider.h
            ProjectionDivider.cpp
            ProjectionDivider.h)

target_link_libraries(KinectToPCL
${OpenCV_LIBS}
${PCL_LIBRARIES}
)
