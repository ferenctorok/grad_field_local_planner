# adding the gradplanner library as target to build:

# setting variable for the dir where the headers are:
set(GRADPLANNER_INCLUDE_DIR
  ${INCLUDE_DIR}/gradplanner
)

# set the sources and headers lists:
set(GRADPLANNER_SOURCES
  utils.cpp
  pixel.cpp
  field.cpp
  index.cpp
  grad_field_base.cpp
  repulsive_field.cpp
)
set(GRADPLANNER_HEADERS
  ${GRADPLANNER_INCLUDE_DIR}/field_utils.h
  ${GRADPLANNER_INCLUDE_DIR}/utils.h
  ${GRADPLANNER_INCLUDE_DIR}/grad_field_base.h
  ${GRADPLANNER_INCLUDE_DIR}/repulsive_field.h
)

# constructing target library:
add_library(gradplanner
  ${GRADPLANNER_SOURCES}
  ${GRADPLANNER_HEADERS}
)

# Specify libraries to link against the gradplanner target library:
target_link_libraries(gradplanner
  ${catkin_LIBRARIES}
)

