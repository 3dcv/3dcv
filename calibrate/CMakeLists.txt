cmake_minimum_required(VERSION 2.8)

project(calibrate)

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

target_link_libraries(calibrate ${OpenCV_LIBS} ${Boost_LIBS})
target_link_libraries(excalibrate ${OpenCV_LIBS} ${Boost_LIBS})
