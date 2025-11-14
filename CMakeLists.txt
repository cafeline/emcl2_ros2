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
find_package(HDF5 REQUIRED COMPONENTS C HL)

ament_auto_find_build_dependencies()

set(EMCL2_CORE_SOURCES
  src/Pose.cpp
  src/CompressedVoxelMap.cpp
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
  ${HDF5_C_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
)

target_include_directories(emcl2_core PRIVATE ${HDF5_INCLUDE_DIRS})

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

  ament_add_gtest(test_compressed_voxel_map
    test/test_compressed_voxel_map.cpp
  )
  target_include_directories(test_compressed_voxel_map PRIVATE ${HDF5_INCLUDE_DIRS})
  target_link_libraries(test_compressed_voxel_map
    emcl2_core
    ${HDF5_C_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
  )

  ament_add_gtest(test_sensor_model
    test/test_sensor_model.cpp
  )
  target_include_directories(test_sensor_model PRIVATE ${HDF5_INCLUDE_DIRS})
  target_link_libraries(test_sensor_model
    emcl2_core
    ${HDF5_C_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
  )

  ament_add_gtest(test_particle
    test/test_particle.cpp
  )
  target_include_directories(test_particle PRIVATE ${HDF5_INCLUDE_DIRS})
  target_link_libraries(test_particle
    emcl2_core
    ${HDF5_C_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
  )

  ament_add_gtest(test_mcl
    test/test_mcl.cpp
  )
  target_include_directories(test_mcl PRIVATE ${HDF5_INCLUDE_DIRS})
  target_link_libraries(test_mcl
    emcl2_core
    ${HDF5_C_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
  )

endif()

install(PROGRAMS
  test/test.bash
  test/test_gui.bash
  DESTINATION lib/${PROJECT_NAME}
)

set(_share_dirs_for_merged_layout
  config
  maps
  rviz2
  launch
  test
)

foreach(dir ${_share_dirs_for_merged_layout})
  install(DIRECTORY ${dir}
    DESTINATION ../share/${PROJECT_NAME})
endforeach()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  maps
  rviz2
  launch
  test
)
