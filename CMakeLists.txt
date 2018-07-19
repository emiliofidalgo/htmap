cmake_minimum_required(VERSION 2.8.3)

project(hamap)

# Finding headers and launch files for QtCreator
file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR} *.h *.hpp)
file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR} *.cxx *.cpp *.c *.py)
file(GLOB_RECURSE LNCH ${CMAKE_CURRENT_SOURCE_DIR} *.launch)

# Catkin dependencies
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs image_transport cv_bridge geometry_msgs obindex)

# System dependencies
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Defining the package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES keypoint_description_hamap hamap
    CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs image_transport cv_bridge geometry_msgs
    DEPENDS OpenCV Boost OpenMP
)

###########
## Build ##
###########

# Including directories.
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Custom target to include headers and launch files in QtCreator
add_custom_target(dtarget_hamap SOURCES ${HDRS} ${SRCS} ${LNCH})

# Description Library
add_library(keypoint_description_hamap
    src/imgdesc/KeypointDetector.cpp
    src/imgdesc/KeypointDescriptor.cpp
    src/imgdesc/GlobalDescriptor.cpp
    src/imgdesc/ldb.cpp)
target_link_libraries(keypoint_description_hamap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Hamap Library
set(HAMAP_SRCS
    src/bayes/BayesFilter.cpp    
    src/lc/LoopCloser.cpp
    src/map/HighLevelMap.cpp
    src/map/Location.cpp
    src/util/Params.cpp
    src/util/Statistics.cpp
    src/util/Util.cpp
    src/Hamap.cpp
)
add_library(hamap ${HAMAP_SRCS})
target_link_libraries(hamap keypoint_description_hamap ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

# Hamap Node
add_executable(hamap_node src/nodes/Hamap_node.cpp)
target_link_libraries(hamap_node hamap)
