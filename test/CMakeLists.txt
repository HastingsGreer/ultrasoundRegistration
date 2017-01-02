##############################################################################
#
# Library:   MultipassUltrasound
#
# Copyright 2010 Kitware Inc. 28 Corporate Drive,
# Clifton Park, NY, 12065, USA.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
##############################################################################


configure_file(GroundTruthCat.jpg GroundTruthCat.jpg COPYONLY)
configure_file(SkewedeCat2.jpg SkewedeCat2.jpg COPYONLY)
include_directories(..)

add_executable(TestSkew2D TestSkew2D.cxx)
target_link_libraries(TestSkew2D ${ITK_LIBRARIES})

add_test(TestSimpleRegistration TestSkew2D GroundTruthCat.jpg SkewedeCat2.jpg out.png)

add_executable(TestAngleSkew TestAngleSkew.cxx)
target_link_libraries(TestAngleSkew ${ITK_LIBRARIES})

add_test(TestAngleSkew_ TestAngleSkew)
