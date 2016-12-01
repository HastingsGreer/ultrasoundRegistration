enable_testing()

configure_file(GroundTruthCat.jpg GroundTruthCat.jpg COPYONLY)
configure_file(SkewedeCat2.jpg SkewedeCat2.jpg COPYONLY)
include_directories(..)

add_executable(TestSkew2D TestSkew2D.cxx)
target_link_libraries(TestSkew2D ${ITK_LIBRARIES})

add_test(TestSimpleRegistration TestSkew2D GroundTruthCat.jpg SkewedeCat2.jpg out.png)

add_executable(TestAngleSkew TestAngleSkew.cxx)
target_link_libraries(TestAngleSkew ${ITK_LIBRARIES})

add_test(TestSimpleRegistration TestSkew2D GroundTruthCat.jpg SkewedeCat2.jpg out.png)
