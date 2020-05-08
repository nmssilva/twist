project(Application)

Message("-- Generating objects for ${PROJECT_NAME}")

set(HEADERS
        Application.h
        )

set(SOURCES
        main.cpp
        Application.cpp
        )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME} PRIVATE
        TWIST::Business
         ${catkin_LIBRARIES}
        )
