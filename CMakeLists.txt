cmake_minimum_required(VERSION 2.8)
project(helloproj)
find_package(PythonLibs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
    ${PYTHON_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR} 
    ./include
    ./include/math
    ./include/visualize)
add_compile_options(-std=c++11)

SET(LIB_DEPENDENCE 
    ${PYTHON_LIBRARIES})

add_executable(draft_demo src/draft_demo.cpp)
target_link_libraries(draft_demo ${LIB_DEPENDENCE})

