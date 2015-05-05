cmake_minimum_required(VERSION 2.8)
project(calibrate)
#list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)

if(OpenCV_found)
  include_directories(${OpenCV_INCLUDE_DIR})
endif(OpenCV_found)

if(Boost_found)
  include_directories(${Boost_INCLUDE_DIR})
endif(Boost_found)

add_executable(calibrate src/calibrate.cpp)
add_executable(excalibrate src/excalibrate.cpp)
add_executable(tester src/tester.cpp)

target_link_libraries(calibrate ${OpenCV_LIBS} ${Boost_LIBS})
target_link_libraries(excalibrate ${OpenCV_LIBS} ${Boost_LIBS})
target_link_libraries(tester ${OpenCV_LIBS} ${Boost_LIBS})
