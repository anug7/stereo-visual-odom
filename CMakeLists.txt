

cmake_minimum_required( VERSION 2.8 )
project(opencv_exp)
set(CMAKE_EXPORT_COMPILE_COMMANDS Yes)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++11 -O3 -fopenmp -pthread")

set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV 4.5 REQUIRED PATHS /opt/opencv-4.5.5/lib/cmake/opencv4)
find_package(Eigen3 3.3 REQUIRED PATHS /opt/eigen3.3.4/share/eigen3/cmake)
find_package(Sophus REQUIRED PATHS /opt/sophus/share/sophus/cmake)
find_package(Pangolin REQUIRED PATHS /opt/pangolin/lib/cmake/Pangolin)
find_package(g2o 1.0.0 REQUIRED PATHS /opt/g2o/lib/cmake/g2o)


include_directories(include)
add_library(base_slam SHARED
  src/frame.cpp
  src/feature.cpp
  src/camera.cpp
  src/map.cpp
  src/mappoint.cpp
  src/frontend.cpp
  src/viewer.cpp
  src/backend.cpp
  src/dataset.cpp
  src/visual_odom.cpp
  src/config.cpp
  )

add_executable(vo_app
  app/main.cpp
)

target_link_libraries(base_slam
  Eigen3::Eigen
  Sophus::Sophus
  g2o::core
  g2o::stuff
  g2o::csparse_extension
  g2o::solver_csparse
  ${OpenCV_LIBRARIES}
  ${Pangolin_LIBRARIES} GL GLU GLEW glut
  pthread
)

target_link_libraries(vo_app
  Eigen3::Eigen
  Sophus::Sophus
  g2o::core
  g2o::stuff
  g2o::csparse_extension
  g2o::solver_csparse
  ${OpenCV_LIBRARIES}
  ${Pangolin_LIBRARIES} GL GLU GLEW glut
  pthread
  base_slam
)
