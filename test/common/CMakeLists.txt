project(common-tests)
Message("-- Generating objects for ${PROJECT_NAME}")

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_add_gtest(MappingTest MappingTest.cpp)
target_link_libraries(MappingTest
        ${catkin_LIBRARIES}
        Tests::TestMain
)
