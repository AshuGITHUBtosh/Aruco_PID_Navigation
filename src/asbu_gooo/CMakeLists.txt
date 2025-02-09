cmake_minimum_required(VERSION 3.5)
project(asbu_gooo)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/asbu_gooo/launch
)

ament_package()
