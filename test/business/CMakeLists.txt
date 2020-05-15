project(business-tests)
Message("-- Generating objects for ${PROJECT_NAME}")

include_directories(include ${catkin_INCLUDE_DIRS})

add_rostest_gtest(InitManagerTest InitManagerTest.launch InitManagerTest.cpp)
target_link_libraries(InitManagerTest
        ${catkin_LIBRARIES}
        Tests::TestMain
        TWIST::Business
        TWIST::Common)

add_rostest_gtest(NavigationManagerTest NavigationManagerTest.launch NavigationManagerTest.cpp)
target_link_libraries(NavigationManagerTest
        ${catkin_LIBRARIES}
        Tests::TestMain
        TWIST::Business
        TWIST::Common)
