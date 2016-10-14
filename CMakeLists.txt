project(ITKv4ImageRegistration)
find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

#add_library(SkewTransform SkewTransform.h)

add_executable(Skew2D Skew2D.cxx )
target_link_libraries(Skew2D ${ITK_LIBRARIES})
#target_link_libraries(Skew2D SkewTransform)
