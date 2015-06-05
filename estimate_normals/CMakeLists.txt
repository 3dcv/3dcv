cmake_minimum_required(VERSION 2.8)
project(estimate_normals)
# ---[ Configurations types
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Possible configurations" FORCE)
if (DEFINED CMAKE_BUILD_TYPE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
endif()
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -o0")
#SET(CMAKE_CXX_LINK_FLAGS "-lboost_program_options")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options date_time)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io visualization)

if(OpenCV_FOUND)
  include_directories(${OpenCV_INCLUDE_DIR})
endif(OpenCV_FOUND)

if(Boost_found)
  include_directories(${Boost_INCLUDE_DIR})
endif(Boost_found)

include_directories(${EIGEN3_INCLUDE_DIRS})

if(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
endif(PCL_FOUND)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(Estimate_Normals src/Estimate_Normals_Main.cpp src/Normal_Estimator.cpp)

target_link_libraries(Estimate_Normals boost_program_options ${OpenCV_LIBS} ${Boost_LIBS} ${PCL_LIBRARIES})
