find_package(ament_cmake_gtest)

ament_add_gtest(test_string_utilities
test_string_utilities.cpp
)

target_link_libraries(test_string_utilities
  ${library_name}
)

ament_add_gtest(test_parsing_utilities
  test_parsing_utilities.cpp
)

target_link_libraries(test_parsing_utilities
  ${library_name}
)
