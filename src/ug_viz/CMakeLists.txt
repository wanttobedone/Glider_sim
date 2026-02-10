cmake_minimum_required(VERSION 3.0.2)
project(ug_viz)

find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()

install(FILES resource/plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

catkin_install_python(PROGRAMS scripts/glider_dashboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
