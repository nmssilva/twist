project(Business)

Message("-- Generating objects for ${PROJECT_NAME}")

set(HEADERS
        include/business/flow/InitManager.h
        flow/NavigationManager.h
        )

set(SOURCES
        flow/InitManager.cpp
        flow/NavigationManager.cpp
        )

add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
add_library(TWIST::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} INTERFACE
        include)

target_link_libraries(${PROJECT_NAME} PRIVATE
        TWIST::Common
        ${catkin_LIBRARIES})
