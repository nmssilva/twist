cmake_minimum_required(VERSION 3.4)
project(twist)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        code_coverage
        message_generation
        roscpp
        rospy
        rostest
        std_msgs
        )

add_message_files(FILES
        target.msg
        )

generate_messages(DEPENDENCIES
        std_msgs
        )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS
        message_runtime
        roscpp
        rospy
        rostest
        std_msgs
        )

# Add compiler flags for coverage instrumentation before defining any targets
append_coverage_compiler_flags()

add_subdirectory(src)

# Header only lib that adds support for toml configuration files
add_subdirectory(external/3rdParty/cpptoml)

add_subdirectory(test)
