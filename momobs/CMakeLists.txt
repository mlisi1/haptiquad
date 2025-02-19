cmake_minimum_required(VERSION 3.8)
project(momobs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(DEPS
  pinocchio
  Eigen3
  pinocchio_spatial_lib
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_library(${PROJECT_NAME} SHARED
  src/residual_estimator.cpp
)



ament_target_dependencies(${PROJECT_NAME} PUBLIC
  ${DEPS}
)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)



install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include

)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS}) 



# target_link_libraries(observer pinocchio_spatial_lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
