# Find GTest
find_package(GTest REQUIRED)

# Enable testing
enable_testing()

# Configure threads
find_package(Threads REQUIRED)

# Test sources
set(TESTS
#  core_gm2_blaze_h_tests
#  core_parametricspaceobject_h_tests
  core_parametricsurfaceconstruction_h_tests
#  core_space_h_tests
#  core_space_traits_tests
#  core_projectivespaceobject_traits_tests
#  parametric_parametricobject_traits_tests
#  parametric_classic_constructions_traits_tests
#  parametric_classic_shapes_traits_tests
#  parametric_subspaceobjects_traits_tests
  parametric_parametricobject_evaluationctrl_traits_tests
#  parametric_parametricsubobject_evaluationctrl_traits_tests
  psurf_compile_tests
  )

# Add tests
foreach(TEST ${TESTS})

  add_executable( unittest_${TEST} ${TEST}.cc )

  target_include_directories(unittest_${TEST} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    )

  target_link_libraries(unittest_${TEST}
    PRIVATE
      ${PROJECT_NAME}
      gmlib2::gmlib2
      ${GTEST_LIBRARIES} ${GTEST_MAIN_LIBRARIES}
      Threads::Threads
    )

  # Get GMlib2 target COMPILE properties
  get_target_property(TEST_GMLIB2_INTERFACE_COMPILE_DEFINITIONS gmlib2::gmlib2 INTERFACE_COMPILE_DEFINITIONS)
  get_target_property(TEST_GMLIB2_INTERFACE_COMPILE_FEATURES gmlib2::gmlib2 INTERFACE_COMPILE_FEATURES)
  get_target_property(TEST_GMLIB2_INTERFACE_COMPILE_OPTIONS gmlib2::gmlib2 INTERFACE_COMPILE_OPTIONS)

  # Set GMlib2 INTERFACE target COMPILE property dependency on GMlib2 PUBLIC profile
  target_compile_definitions(unittest_${TEST} PUBLIC ${TEST_GMLIB2_INTERFACE_COMPILE_DEFINITIONS})
  target_compile_features(unittest_${TEST}    PUBLIC ${TEST_GMLIB2_INTERFACE_COMPILE_FEATURES})
  target_compile_options(unittest_${TEST}     PUBLIC ${TEST_GMLIB2_INTERFACE_COMPILE_OPTIONS})

  # Turn off platform-spesific extensions
  set_target_properties(unittest_${TEST} PROPERTIES CXX_EXTENSIONS OFF)

  # Add additional compile options
  target_compile_options(unittest_${TEST}
    PUBLIC $<$<CXX_COMPILER_ID:Clang>:
      -Wno-used-but-marked-unused
      >
    PUBLIC $<$<CXX_COMPILER_ID:GNU>:
      -Wno-unused-value
      >
      )

  gtest_add_tests( unittest_${TEST} "" AUTO )
endforeach(TEST)

message("GMlib2 unittests SETUP done!")
