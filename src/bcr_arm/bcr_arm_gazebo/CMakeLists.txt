cmake_minimum_required(VERSION 3.8)
project(bcr_arm_gazebo)

find_package(ament_cmake REQUIRED)

# Install directories (Launch, Config, Scripts, Worlds)
install(DIRECTORY launch config scripts worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
