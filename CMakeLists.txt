cmake_minimum_required(VERSION 3.16)
project(TITA_MJ)

set(CMAKE_CXX_STANDARD 17)


find_package(glfw3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

set(MUJOCO_DIR "$ENV{HOME}/mujoco" CACHE PATH "MuJoCo base directory")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
set(MUJOCO_BUILD_DIR "${MUJOCO_DIR}/build")
set(MUJOCO_BIN_DIR "${MUJOCO_DIR}/bin")
set(MUJOCO_LIB_PATH "${MUJOCO_BUILD_DIR}")

include_directories(${MUJOCO_INCLUDE_DIR})
link_directories(${MUJOCO_LIB_PATH})

# Try to find libmujoco in the expected build directory first, fall back to
# standard locations if necessary
find_library(MUJOCO_LIBRARY
    NAMES mujoco
    PATHS ${MUJOCO_LIB_PATH} ${MUJOCO_BIN_DIR}
    NO_DEFAULT_PATH
)
if(NOT MUJOCO_LIBRARY)
    find_library(MUJOCO_LIBRARY NAMES mujoco)
endif()


# Manually set INCLUDE_DIRS and LIBRARIES since not handled in blasfeo and hpipm
set(blasfeo_INCLUDE_DIRS /home/ubuntu/.local/blasfeo/include)
set(hpipm_INCLUDE_DIRS /home/ubuntu/.local/hpipm/include)
set(blasfeo_LIBRARIES /home/ubuntu/.local/blasfeo/lib/libblasfeo.a)
set(hpipm_LIBRARIES /home/ubuntu/.local/hpipm/lib/libhpipm.a)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  $ENV{HOME}/.local/include
)

add_library(mpc MODULE
    src/MPC.cpp
    src/pybind_mpc.cpp
)

#target_link_libraries(wholebody PRIVATE
#    pybind11::module
#    pinocchio::pinocchio
#    Eigen3::Eigen
#    $ENV{HOME}/.local/lib/libblasfeo.a
#    $ENV{HOME}/.local/lib/libhpipm.a
#)


#target_link_libraries(mpc PRIVATE pybind11::module pinocchio::pinocchio Eigen3::Eigen)
# Non serve prefisso "lib" per modulo Python
#set_target_properties(mpc PROPERTIES PREFIX "")


add_library(wbc MODULE
    src/WholeBodyController.cpp
    src/utils.cpp 
    src/pybind_wbc.cpp
)

#target_link_libraries(wbc PRIVATE pybind11::module pinocchio::pinocchio Eigen3::Eigen)
# Non serve prefisso "lib" per modulo Python
#set_target_properties(wbc PROPERTIES PREFIX "")


# MPC
target_link_libraries(mpc PRIVATE
    pybind11::module
    pinocchio::pinocchio
    Eigen3::Eigen
    $ENV{HOME}/.local/lib/libhpipm.a
    $ENV{HOME}/.local/lib/libblasfeo.a
)
set_target_properties(mpc PROPERTIES 
    PREFIX ""
    #LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)

# WBC
target_link_libraries(wbc PRIVATE
    pybind11::module
    pinocchio::pinocchio
    Eigen3::Eigen
    $ENV{HOME}/.local/lib/libhpipm.a
    $ENV{HOME}/.local/lib/libblasfeo.a
)

set_target_properties(wbc PROPERTIES 
    PREFIX ""
    #LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)


install(TARGETS mpc wbc
        LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/compiled)
