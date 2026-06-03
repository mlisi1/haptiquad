cmake_minimum_required(VERSION 2.8.3)
project(haptiquad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPS
  pinocchio
  Eigen3
)

# find dependencies
find_package(catkin REQUIRED)

find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS pinocchio
)



add_library(${PROJECT_NAME} SHARED
  src/momentum_observer.cpp
  src/force_estimator.cpp
)



target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Eigen3::Eigen
)


target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${Eigen3_INCLUDE_DIRS}
)



install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # Add test dependencies here if needed
endif()