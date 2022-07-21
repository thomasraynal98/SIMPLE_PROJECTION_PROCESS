CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(SIMPLE_PROJECTION_PROCESS VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)

# set good gcc version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -L/usr/local/lib")

# found package.
FIND_PACKAGE(OpenCV 4.2 REQUIRED)
# FIND_PACKAGE(OpenXLSX REQUIRED)
# find_package(realsense2 REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# MODULE DE GESTION DU SYSTEME YAML ET REDIS + DEBUG
ADD_EXECUTABLE(MAIN 
src/main.cpp)

TARGET_LINK_LIBRARIES(MAIN
    # pthread
    # hiredis
    # redis++
    # sioclient
    # realsense2
    # serial
    # Eigen3::Eigen
    # OpenXLSX::OpenXLSX
    ${OpenCV_LIBS}
    )