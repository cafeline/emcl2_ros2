cmake_minimum_required(VERSION 3.8)
project(emcl2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

ament_auto_find_build_dependencies()

set(EMCL2_CORE_SOURCES
  src/Pose.cpp
  src/OctomapMap.cpp
  src/SensorModel.cpp
  src/Particle.cpp
  src/Mcl.cpp
  src/OdomModel.cpp
  src/emcl2_node.cpp
)

ament_auto_add_library(emcl2_core SHARED
  ${EMCL2_CORE_SOURCES}
)

target_link_libraries(emcl2_core
  Eigen3::Eigen
  ${OCTOMAP_LIBRARIES}
)

target_include_directories(emcl2_core PRIVATE ${OCTOMAP_INCLUDE_DIRS})

ament_target_dependencies(emcl2_core
  rclcpp
)

ament_auto_add_executable(emcl2_node
  src/main.cpp
)

target_link_libraries(emcl2_node
  emcl2_core
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_sensor_model
    test/test_sensor_model.cpp
  )
  target_include_directories(test_sensor_model PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_sensor_model
    emcl2_core
    ${OCTOMAP_LIBRARIES}
  )

  ament_add_gtest(test_particle
    test/test_particle.cpp
  )
  target_include_directories(test_particle PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_particle
    emcl2_core
    ${OCTOMAP_LIBRARIES}
  )

  ament_add_gtest(test_mcl
    test/test_mcl.cpp
  )
  target_include_directories(test_mcl PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_mcl
    emcl2_core
    ${OCTOMAP_LIBRARIES}
  )

  ament_add_gtest(test_octomap_map
    test/test_octomap_map.cpp
  )
  target_include_directories(test_octomap_map PRIVATE ${OCTOMAP_INCLUDE_DIRS})
  target_link_libraries(test_octomap_map
    emcl2_core
    ${OCTOMAP_LIBRARIES}
  )

endif()

install(PROGRAMS
  test/test.bash
  test/test_gui.bash
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  test
)
