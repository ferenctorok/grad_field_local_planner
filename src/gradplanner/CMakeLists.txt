# adding the gradplanner library as target to build:
add_library(gradplanner
  ${CMAKE_CURRENT_LIST_DIR}/grad_field_base.cpp
  ${CMAKE_CURRENT_LIST_DIR}/pixel.cpp
  ${CMAKE_CURRENT_LIST_DIR}/utils.cpp
  ${CMAKE_CURRENT_LIST_DIR}/field.cpp
)

# Specify libraries to link against the gradplanner target library:
target_link_libraries(gradplanner
  ${catkin_LIBRARIES}
)

