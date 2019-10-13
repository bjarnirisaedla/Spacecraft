cmake_minimum_required(VERSION 2.8)
 
PROJECT(CRTBP)

SET(CMAKE_BUILD_TYPE Release)
 
# set path to your VTK directory
SET(VTK_DIR ~/Documents/semester_project/original_directory/VTK/VTK-8.1.2/build/)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

 
include_directories(${CMAKE_SOURCE_DIR})

add_executable(N_body_system 
N_body_system.cpp 
mysystem.hpp 
vtkhelper.hpp 
modes.hpp 
denseMouseInteractor.hpp 
gridMouseInteractor.hpp
trajectoryMouseInteractor.hpp
backwardsMouseInteractor.hpp
ftleMouseInteractor.hpp
isolines.hpp
)
 
target_link_libraries(N_body_system ${VTK_LIBRARIES} ${EIGEN3_LIBRARIES})
