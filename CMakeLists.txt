cmake_minimum_required(VERSION 3.0.0)
project(tabu VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.74.0) 

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
    add_executable(tabu)
    add_subdirectory(src)
endif()

include(CTest)
enable_testing()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
