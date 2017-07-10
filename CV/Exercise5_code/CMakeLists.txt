cmake_minimum_required(VERSION 2.8)
project( CV_Ex5 )
#SET(CMAKE_PREFIX_PATH /pathtoopencv)
#SET(OpenCV_DIR /pathtoopencv)

SET(CMAKE_BUILD_TYPE DEBUG)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB SRC . *.cpp)
add_executable (main ${SRC})
target_link_libraries( main ${OpenCV_LIBS} )
