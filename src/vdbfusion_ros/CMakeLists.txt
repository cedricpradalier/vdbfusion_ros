# MIT License
#
# # Copyright (c) 2022 Saurabh Gupta, Ignacio Vizzo, Cyrill Stachniss, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

add_library(vdbfusion_ros SHARED Transform.cpp VDBVolume_functions.cpp)
target_link_libraries(vdbfusion_ros PUBLIC
  Sophus::Sophus
  VDBFusion::vdbfusion
  igl::core
  happly
  ${catkin_LIBRARIES}
)
target_include_directories(vdbfusion_ros PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node VDBVolume_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC
  vdbfusion_ros
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_bag VDBVolume_bag.cpp)
target_link_libraries(${PROJECT_NAME}_bag PUBLIC
  vdbfusion_ros
)
target_include_directories(${PROJECT_NAME}_bag PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
