include(../cmake/testing_utils.cmake)

SETUP_BENCHMARK_ENV()

if(benchmark_FOUND)


  # MY benchmarks
  set( BENCHMARKS_DIR benchmarks )
  set( BENCHMARKS
    my_dummy_bechmarks
    )

  ADD_BENCHMARKS(dte3607::physengine ${BENCHMARKS_DIR} BENCHMARKS)

endif(benchmark_FOUND)
