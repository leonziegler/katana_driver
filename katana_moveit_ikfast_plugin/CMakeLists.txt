cmake_minimum_required(VERSION 2.8.3)
project(katana_moveit_ikfast_plugin)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
)

include_directories(include)

set(KATANA_450_6M90A_IKFAST_LIBRARY_NAME katana_450_6m90a_arm_moveit_ikfast_plugin)
set(KATANA_450_6M180_IKFAST_LIBRARY_NAME katana_450_6m180_arm_moveit_ikfast_plugin)

find_package(LAPACK REQUIRED)

add_library(${KATANA_450_6M90A_IKFAST_LIBRARY_NAME} src/katana_450_6m90a_arm_ikfast_moveit_plugin.cpp)
add_library(${KATANA_450_6M180_IKFAST_LIBRARY_NAME} src/katana_450_6m180_arm_ikfast_moveit_plugin.cpp)
target_link_libraries(${KATANA_450_6M90A_IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})
target_link_libraries(${KATANA_450_6M180_IKFAST_LIBRARY_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS ${KATANA_450_6M90A_IKFAST_LIBRARY_NAME} ${KATANA_450_6M180_IKFAST_LIBRARY_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  katana_450_6m90a_arm_moveit_ikfast_plugin_description.xml
  katana_450_6m180_arm_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
