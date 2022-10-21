cmake_minimum_required(VERSION 2.8.12)
project (Epos2TestModule)

find_package (Eigen3 3.4 REQUIRED)
find_package(Curses REQUIRED)
find_package(OpenCV REQUIRED)

set (CMAKE_SHARED_LINKER_FLAGS "-pthread")

add_compile_definitions(NON_MATLAB_PARSING=1)
add_compile_definitions(MAX_EXT_API_CONNECTIONS=255)
add_compile_definitions(DO_NOT_USE_SHARED_MEMORY=1)

add_executable(cdpr_sim ${CMAKE_SOURCE_DIR}/src/cdpr_sim.cpp)

target_link_libraries(cdpr_sim PUBLIC Eigen3::Eigen)
target_include_directories(cdpr_sim PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cdpr_sim PUBLIC ${OpenCV_LIBS})


