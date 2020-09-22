# Copyright 2020, CNRS
# See LICENSE file.

ADD_EXECUTABLE(solo-autocollision-leg-leg-codegen autocollision-leg-leg-codegen.cpp)
TARGET_INCLUDE_DIRECTORIES(solo-autocollision-leg-leg-codegen  PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(solo-autocollision-leg-leg-codegen pinocchio::pinocchio ${CMAKE_DL_LIBS})

ADD_EXECUTABLE(autocollision-shoulders-main autocollision-shoulders-main.cpp)
TARGET_INCLUDE_DIRECTORIES(autocollision-shoulders-main PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(autocollision-shoulders-main pinocchio::pinocchio ${CMAKE_DL_LIBS})

ADD_EXECUTABLE(inference-nn-main inference_neural_net_codegen.cpp)
TARGET_INCLUDE_DIRECTORIES(inference-nn-main PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(inference-nn-main pinocchio::pinocchio ${CMAKE_DL_LIBS})

#find_package(Torch REQUIRED)

#ADD_EXECUTABLE(load_pytorch_model load_pytorch_model.cpp)
#TARGET_LINK_LIBRARIES(load_pytorch_model "${TORCH_LIBRARIES}")
#set_property(TARGET load_pytorch_model PROPERTY CXX_STANDARD 14)
