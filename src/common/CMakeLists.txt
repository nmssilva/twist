project(Common)
Message("-- Generating objects for ${PROJECT_NAME}")

set(HEADERS
        include/common/Configs.h
        include/common/Mapping.h
        include/common/Logging.h
        )

set(SOURCES Configs.cpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
add_library(TWIST::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE include)

target_link_libraries(${PROJECT_NAME} PRIVATE
        3rdParty::cpptoml
        ${catkin_LIBRARIES}
        )

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_compile_definitions(${PROJECT_NAME} PUBLIC CONFIG_FILE="${CMAKE_SOURCE_DIR}/resources/configuration/config.toml")
