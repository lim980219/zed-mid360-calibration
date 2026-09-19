cmake_minimum_required(VERSION 3.0.2)
project(zed_mid360_calibration)
find_package(catkin REQUIRED)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/calibrate_bags.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

