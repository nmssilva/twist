project(Common)
Message("-- Generating objects for ${PROJECT_NAME}")

set(HEADERS
        include/common/Mapping.h
        include/common/Logging.h
)

set(SOURCES
)

include_directories(include)

add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
add_library(TWIST::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        INTERFACE include)

target_link_libraries(${PROJECT_NAME}
)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
