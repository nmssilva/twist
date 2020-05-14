project(business-tests)
Message("-- Generating objects for ${PROJECT_NAME}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_unit_test(
        BASENAME InitManagerTest
        LIBS gtest TWIST::Business
        SOURCES InitManagerTest.cpp
)
