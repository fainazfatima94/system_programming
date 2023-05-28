include(../cmake/testing_utils.cmake)

SETUP_UNITTEST_ENV()

if(GTest_FOUND)

  # Provided unittests
  set( PREDEF_UNITTEST_DIR unittests/predefined )
  set( PREDEF_UNITTESTS
    predef_mech_compute_trajectory
    predef_mech_sp_vs_fpl_detection
    predef_mech_sp_vs_fpl_response
    predef_mech_sp_vs_sp_detection
    predef_mech_sp_vs_sp_response
    predef_solver_dev_level1
    predef_solver_dev_level2
    predef_solver_dev_level3
    predef_solver_dev_level4
    )

  # MY unittests
  set( UNITTEST_DIR unittests )
  set( UNITTESTS
    my_dummy_unittests
    )

  ADD_UNITTESTS(dte3607::physengine ${PREDEF_UNITTEST_DIR} PREDEF_UNITTESTS)
  ADD_UNITTESTS(dte3607::physengine ${UNITTEST_DIR} UNITTESTS)

endif(GTest_FOUND)
