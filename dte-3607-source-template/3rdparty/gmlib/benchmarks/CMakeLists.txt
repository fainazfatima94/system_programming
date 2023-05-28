# Find GTest
find_package(benchmark REQUIRED)

# Enable testing
enable_testing()

# Configure threads
find_package(Threads REQUIRED)

# Test sources
set(BENCHMARKS
  tpsurface_evaluation_benchmarks)

# Add tests
foreach(BENCH ${BENCHMARKS})

  add_executable( bench_${BENCH} ${BENCH}.cc )

  target_include_directories(bench_${BENCH} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    )

  target_link_libraries(bench_${BENCH}
    PRIVATE ${PROJECT_NAME}
      benchmark::benchmark
      Threads::Threads
      )

  # Get GMlib2 target COMPILE properties
  get_target_property(BENCH_GMLIB2_INTERFACE_COMPILE_DEFINITIONS gmlib2::gmlib2 INTERFACE_COMPILE_DEFINITIONS)
  get_target_property(BENCH_GMLIB2_INTERFACE_COMPILE_FEATURES gmlib2::gmlib2 INTERFACE_COMPILE_FEATURES)
  get_target_property(BENCH_GMLIB2_INTERFACE_COMPILE_OPTIONS gmlib2::gmlib2 INTERFACE_COMPILE_OPTIONS)

  # Set GMlib2 INTERFACE target COMPILE property dependency on GMlib2 PUBLIC profile
  target_compile_definitions(bench_${BENCH} PUBLIC ${BENCH_GMLIB2_INTERFACE_COMPILE_DEFINITIONS})
  target_compile_features(bench_${BENCH}    PUBLIC ${BENCH_GMLIB2_INTERFACE_COMPILE_FEATURES})
  target_compile_options(bench_${BENCH}     PUBLIC ${BENCH_GMLIB2_INTERFACE_COMPILE_OPTIONS})

  # Turn off platform-spesific extensions
  set_target_properties(bench_${BENCH} PROPERTIES CXX_EXTENSIONS OFF)

  # Add additional compile options
#    target_compile_options(bench_${BENCH}
#      PUBLIC $<$<CXX_COMPILER_ID:Clang>:
#        -Wno-some-compile-feature
#        >
#        )

  add_test(NAME bench_${BENCH} COMMAND bench_${BENCH})
endforeach(BENCH)

message("GMlib2 benchmarks SETUP done!")
