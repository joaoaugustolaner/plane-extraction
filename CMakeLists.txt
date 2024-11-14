cmake_minimum_required(VERSION 3.10)


cmake_policy(SET CMP0167 NEW)
cmake_policy(SET CMP0144 NEW)


# Set the project name and version
project(TC VERSION 1.0.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV 4.6 REQUIRED)
find_package(PCL 1.13 REQUIRED COMPONENTS common io visualization)
find_package(JsonCpp REQUIRED)

# Include the directories with header files
include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

# Source files 
add_executable(app src/Main.cpp src/Stitcher.cpp src/CloudPointProcessor.cpp src/ClickHandler.cpp)

if(APPLE)
    # Explicitly set OpenMP flags for macOS
    set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/Cellar/libomp/19.1.3/include")
    set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/usr/local/Cellar/libomp/19.1.3/include")
    set(OpenMP_C_LIB_NAMES "omp")
    set(OpenMP_CXX_LIB_NAMES "omp")
    set(OpenMP_omp_LIBRARY "/usr/local/Cellar/libomp/19.1.3/lib/libomp.dylib")

    # Link OpenMP library
    target_link_libraries(app PUBLIC ${OpenMP_omp_LIBRARY})

    # Set compiler and linker flags to ensure proper linking with libomp
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L/usr/local/Cellar/libomp/19.1.3/lib -lomp")
else()
    find_package(OpenMP REQUIRED)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(app PUBLIC OpenMP::OpenMP_CXX)
    endif()
endif()

target_link_libraries(app PUBLIC JsonCpp::JsonCpp)
target_link_libraries(app PUBLIC ${OpenCV_LIBS} ${PCL_LIBRARIES})

#Debuggin
#message(STATUS "OpenCV library status:")
#message(STATUS "    version: ${OpenCV_VERSION}")
#message(STATUS "    libraries: ${OpenCV_LIBS}")
#message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

