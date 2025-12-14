cmake_minimum_required(VERSION 3.16)
project(TITA_MJ)

set(CMAKE_CXX_STANDARD 17)

#set(MUJOCO_LIBRARY "/home/ubuntu/Desktop/repo_rl/TITA_MJ/.pixi/build/work/tita_mj-f6yTh76OkK4-*/host_placehold*/lib/libmujoco.so")
#set(MUJOCO_INCLUDE_DIR "/home/ubuntu/Desktop/repo_rl/TITA_MJ/.pixi/build/work/tita_mj-f6yTh76OkK4-*/host_placehold*/include")
#set(CMAKE_BUILD_RPATH "$ENV{HOME}/.pixi/build/work/./.pixi/build/work/../host_placehold*/work")
#set(CMAKE_INSTALL_RPATH "$ENV{HOME}/.pixi/build/work/./.pixi/build/work/../host_placehold*/work")
#set(CMAKE_SKIP_BUILD_RPATH FALSE)
#set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
#set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(glfw3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(mujoco CONFIG QUIET)
if(TARGET mujoco::mujoco)
    message(STATUS "MuJoCo found: using mujoco::mujoco")
else()
    # prova fallback su percorso Pixi
    set(MUJOCO_PATH "$ENV{HOME}/.pixi/build/work/.../host_placehold*/lib")
    find_library(MUJOCO_LIBRARY NAMES mujoco PATHS ${MUJOCO_PATH})
    if(NOT MUJOCO_LIBRARY)
        message(FATAL_ERROR "MuJoCo library not found in ${MUJOCO_PATH}")
    endif()
    add_library(mujoco::mujoco UNKNOWN IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_LOCATION ${MUJOCO_LIBRARY})
endif()

#set(MUJOCO_LIBRARY "$ENV{HOME}/.pixi/envs/default/lib/libmujoco.so")
#set(MUJOCO_INCLUDE_DIR "$ENV{HOME}/.pixi/envs/default/include")

include_directories(${MUJOCO_INCLUDE_DIR})

if(NOT MUJOCO_LIBRARY)
    message(FATAL_ERROR "Cannot find MuJoCo library in ${MUJOCO_LIB_DIR}")
endif()


# Manually set INCLUDE_DIRS and LIBRARIES since not handled in blasfeo and hpipm
set(blasfeo_INCLUDE_DIRS /home/ubuntu/.local/blasfeo/include)
set(hpipm_INCLUDE_DIRS /home/ubuntu/.local/hpipm/include)
set(blasfeo_LIBRARIES /home/ubuntu/.local/blasfeo/lib/libblasfeo.a)
set(hpipm_LIBRARIES /home/ubuntu/.local/hpipm/lib/libhpipm.a)


#include_directories(${BLASFEO_INCLUDE_DIR} ${HPIPM_INCLUDE_DIR})
#include_directories(${PROJECT_SOURCE_DIR}/include $ENV{HOME}/.local/include)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  $ENV{HOME}/.local/include
)

add_library(mpc MODULE
    src/MPC.cpp
    src/pybind_mpc.cpp
)

add_library(wbc MODULE
    src/JointCommand.cpp
    src/JointState.cpp
    src/MPC.cpp
    src/RobotState.cpp
    src/SE3.cpp
    src/WalkingManager.cpp
    src/WholeBodyController.cpp
    src/utils.cpp 
    src/pybind_wbc.cpp
)

# WBC
target_link_libraries(mpc PRIVATE
    pybind11::module
    pinocchio::pinocchio
    Eigen3::Eigen
    $ENV{HOME}/.local/lib/libhpipm.a
    $ENV{HOME}/.local/lib/libblasfeo.a
)
set_target_properties(mpc PROPERTIES 
    PREFIX "")

# WBC
target_link_libraries(wbc PRIVATE
    mujoco::mujoco
    pybind11::module
    pinocchio::pinocchio
    Eigen3::Eigen
    $ENV{HOME}/.local/lib/libhpipm.a
    $ENV{HOME}/.local/lib/libblasfeo.a
)

set_target_properties(wbc PROPERTIES 
    PREFIX ""
)

install(TARGETS mpc wbc
        LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/compiled)
