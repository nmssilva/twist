project(TestMain)

set(SOURCES main.cpp)

set(HEADERS )

add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
add_library(Tests::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} INTERFACE . )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)
