cmake_minimum_required(VERSION 3.5)

project(ITKv4ImageRegistration)
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

add_library(PrintTransform PrintTransform.cxx)

add_executable(Skew2D Skew2D.cxx )
target_link_libraries(Skew2D ${ITK_LIBRARIES})
target_link_libraries(Skew2D PrintTransform)

add_executable(Coregistration Coregistration.cxx )
target_link_libraries(Coregistration ${ITK_LIBRARIES})
target_link_libraries(Coregistration PrintTransform)

add_subdirectory(test)
