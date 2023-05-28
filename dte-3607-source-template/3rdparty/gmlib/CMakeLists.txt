# Minimum CMake version required
cmake_minimum_required(VERSION 3.15)
# version 3.15 introduces behaviour on policy cmp0092 for MSVC
# version 3.12 provides a force on cxx_std_20 compile features

# Project
project(gmlib VERSION 2.1 LANGUAGES CXX)

# Add library target
add_library(${PROJECT_NAME} INTERFACE )

####################
# Configure Wintendo

# IF on Wintendo platform
if(MSVC AND NOT VCPKG_TOOLCHAIN)
  message(FATAL_ERROR
    "Enable VCPKG toolchain on MSVC platform\n"
    "  Star over; CMAKE_TOOLCHAIN_FILE definition missing!\n"
    "  i.e.: cmake -DCMAKE_TOOLCHAIN_FILE=<path_to>\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake <path_to>\\source\\"
    )
endif()



#################
# Configure blaze
find_package(blaze REQUIRED)
target_link_libraries(${PROJECT_NAME} INTERFACE blaze::blaze)

# (Manual) Blaze hook: configure lapack for blaze on wintendo
if(MSVC AND VCPKG_TOOLCHAIN)
  find_package(lapack REQUIRED)
  target_link_libraries(${PROJECT_NAME} INTERFACE ${LAPACK_LIBRARIES})
endif()


######################
# Configure gmconcepts
add_subdirectory(libs/gmconcepts)
target_link_libraries(${PROJECT_NAME} INTERFACE gmconcepts::gmconcepts )


###########################
# Compiler spesific options

# Turn on c++20 compile features -- minimum CMake Version 3.12 required
target_compile_features(${PROJECT_NAME}
  INTERFACE $<$<CXX_COMPILER_ID:Clang>:cxx_std_20>
  INTERFACE $<$<CXX_COMPILER_ID:GNU>:cxx_std_20>
  INTERFACE $<$<CXX_COMPILER_ID:MSVC>:cxx_std_20>
  )

# Compile definitions
target_compile_definitions( ${PROJECT_NAME}
  INTERFACE $<$<CXX_COMPILER_ID:MSVC>: _USE_MATH_DEFINES>
  )


# Compiler spesific options
target_compile_options(${PROJECT_NAME}
#  INTERFACE $<$<CXX_COMPILER_ID:AppleClang>:
#    -some-compiler-flag # somewhere over the rainbow
#    >
  INTERFACE $<$<CXX_COMPILER_ID:Clang>:
    -Weverything -Werror
    -pedantic
    -Wno-c++98-compat -Wno-c++98-compat-pedantic
    -Wno-documentation
    -Wno-global-constructors
    -Wno-exit-time-destructors
    -Wno-weak-vtables
    -Wno-padded
    -Wno-unused-parameter
    -Wno-language-extension-token
    -Wno-extra-semi-stmt
    -Wno-ctad-maybe-unsupported
    >
  INTERFACE $<$<CXX_COMPILER_ID:GNU>:
    -pedantic -Wall -Werror
    >
  INTERFACE $<$<CXX_COMPILER_ID:MSVC>:
    /wd4127  # BLAZE; conditional expression is constant
    >
    )




############################
# Target sources and friends

# Headers
target_include_directories( ${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<INSTALL_INTERFACE:include>
)





##################################
# Export targets and configuration
export( TARGETS ${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}-targets.cmake"
  )

# Add alias gmlib2::${PROJECT_NAME} to ${PROJECT_NAME} such that we can reference this
# in the same way as for other consumer usages (external library)
add_library(gmlib::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}ConfigVersion.cmake"
  COMPATIBILITY SameMajorVersion
  )

configure_package_config_file(
  "cmake/${PROJECT_NAME}-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "lib/cmake/${PROJECT_NAME}"
  )




#########
# Testing
option(GM_BUILD_TESTS "Build unit tests..." FALSE)
if(GM_BUILD_TESTS)
  add_subdirectory(tests)
endif(GM_BUILD_TESTS)




##############
# Benchmarking
option(GM_BUILD_BENCHMARKS "Build benchmarks..." FALSE)
if(GM_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif(GM_BUILD_BENCHMARKS)



##############
# Dummy target
option(GM_DUMMY_TARGET "Dummy target..." TRUE)
if(GM_DUMMY_TARGET)
  add_custom_target(${PROJECT_NAME}_DUMMY)
endif(GM_DUMMY_TARGET)
