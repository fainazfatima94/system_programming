project(physengine)

# Add library target
add_library(${PROJECT_NAME} INTERFACE )

## Link in libraries
target_link_libraries(${PROJECT_NAME}
  INTERFACE
  gmlib::gmlib
    )

# Headers
target_include_directories( ${PROJECT_NAME}
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include> )

# Add alias gmlib2::${PROJECT_NAME} to ${PROJECT_NAME} such that we can reference this
# in the same way as for other consumer usages (external library)
add_library(dte3607::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
