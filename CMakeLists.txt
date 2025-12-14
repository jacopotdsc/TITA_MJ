cmake_minimum_required(VERSION 3.16)
project(TITA_MJ)

set(CMAKE_CXX_STANDARD 17)


find_package(glfw3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

set(MUJOCO_DIR "/home/ubuntu/mujoco")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/build") 
set(MUJOCO_LIB_DIR "${MUJOCO_DIR}/bin") 
set(MUJOCO_LIB_PATH "$ENV{HOME}/mujoco/build/lib")

include_directories(${MUJOCO_INCLUDE_DIR})
link_directories(${MUJOCO_LIB_DIR})

include_directories($ENV{HOME}/mujoco/include)
link_directories($ENV{HOME}/mujoco/build/lib)


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
set_target_properties(mpc PROPERTIES PREFIX "")

# WBC
target_link_libraries(wbc PRIVATE
    pybind11::module
    pinocchio::pinocchio
    Eigen3::Eigen
    $ENV{HOME}/.local/lib/libhpipm.a
    $ENV{HOME}/.local/lib/libblasfeo.a
    ${MUJOCO_LIB_PATH}/libmujoco.so
)
set_target_properties(wbc PROPERTIES PREFIX "" 
    INSTALL_RPATH "$ENV{HOME}/mujoco/build/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

