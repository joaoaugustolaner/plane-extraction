cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(TC VERSION 1.0.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Specify OpenCV Directory
# set(OpenCV_INCLUDE_DIRS "/usr/local/Cellar/opencv/4.10.0_6/lib/cmake/opencv4")
# set(OpenCV_DIR "/usr/local/Cellar/opencv/4.10.0_6/lib/cmake/opencv4")


find_package(OpenCV 4.10 REQUIRED)
# Include the directories with header files
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# Source files 
add_executable(app src/Main.cpp src/Stitcher.cpp src/CloudPointProcessor.cpp)


target_link_libraries(TC ${OpenCV_LIBS})

#Debuggind
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

