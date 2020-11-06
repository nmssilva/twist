project(cpptoml)
Message("-- Generating objects for ${PROJECT_NAME}")

set(HEADERS include/cpptoml/cpptoml.h)

set(SOURCES)

include_directories(include)

add_library(${PROJECT_NAME} STATIC ${SOURCES} ${HEADERS})
add_library(3rdParty::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE include)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
