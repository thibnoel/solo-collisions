# Copyright 2020, CNRS
# See LICENSE file.

ADD_EXECUTABLE(end2end_cg_demo end2end_codegen_demo.cpp)
TARGET_INCLUDE_DIRECTORIES(end2end_cg_demo PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(end2end_cg_demo pinocchio::pinocchio ${CMAKE_DL_LIBS})

ADD_EXECUTABLE(autocollision-codegen-solo autocollision-codegen-solo.cpp)
TARGET_INCLUDE_DIRECTORIES(autocollision-codegen-solo PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(autocollision-codegen-solo pinocchio::pinocchio ${CMAKE_DL_LIBS})

ADD_EXECUTABLE(autocollision-shoulders-main autocollision-shoulders-main.cpp)
TARGET_INCLUDE_DIRECTORIES(autocollision-shoulders-main PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(autocollision-shoulders-main pinocchio::pinocchio ${CMAKE_DL_LIBS})
