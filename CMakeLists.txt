cmake_minimum_required(VERSION 3.5)
project(ros_networktables_bridge_host)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/list_all.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    ros_networktables_bridge_host/
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
