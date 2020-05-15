# Copyright 2020, CNRS
# See LICENSE file.

#ADD_EXECUTABLE(rohans_test-cg rohans_test-cg.cpp)
#TARGET_LINK_LIBRARIES(rohans_test-cg pinocchio::pinocchio ${CMAKE_DL_LIBS})

#ADD_EXECUTABLE(segment_segment_distance segment_segment_distance.cpp)
#TARGET_INCLUDE_DIRECTORIES(segment_segment_distance PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})

ADD_EXECUTABLE(sep_cg_demo separate_codegen_demo.cpp)
TARGET_INCLUDE_DIRECTORIES(sep_cg_demo PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(sep_cg_demo pinocchio::pinocchio ${CMAKE_DL_LIBS})

ADD_EXECUTABLE(end2end_cg_demo end2end_codegen_demo.cpp)
TARGET_INCLUDE_DIRECTORIES(end2end_cg_demo PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(end2end_cg_demo pinocchio::pinocchio ${CMAKE_DL_LIBS})
