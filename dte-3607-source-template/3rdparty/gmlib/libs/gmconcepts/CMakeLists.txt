# Minimum CMake version required
cmake_minimum_required(VERSION 3.15)
# version 3.15 introduces behaviour on policy cmp0092 for MSVC
# version 3.12 provides a force on cxx_std_20 compile features

# Project
project(gmconcepts VERSION 0.1 LANGUAGES CXX)

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



###########################
# Compiler spesific options

# Turn on c++20 compile features -- minimum CMake Version 3.12 required
target_compile_features(${PROJECT_NAME}
  INTERFACE $<$<CXX_COMPILER_ID:Clang>:cxx_std_17>
  INTERFACE $<$<CXX_COMPILER_ID:GNU>:cxx_std_17>
  INTERFACE $<$<CXX_COMPILER_ID:MSVC>:cxx_std_17>
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
    >
  INTERFACE $<$<CXX_COMPILER_ID:GNU>:
    -pedantic -Wall -Werror
    >
  INTERFACE $<$<CXX_COMPILER_ID:MSVC>:
    /wd4146  # BLAZE; unary minus operator applied to unsigned type
    /wd4267  # BLAZE; 'var': conversion from 'size_t' to 'type'
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

# Add alias gmconcepts::${PROJECT_NAME} to ${PROJECT_NAME} such that we can
# reference this in the same way as for other consumer usages
# (external library)
add_library(gmconcepts::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


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



##############
# Dummy target
option(DUMMY_TARGET "Dummy target..." TRUE)
if(DUMMY_TARGET)
  add_custom_target(${PROJECT_NAME}_DUMMY)
endif(DUMMY_TARGET)
