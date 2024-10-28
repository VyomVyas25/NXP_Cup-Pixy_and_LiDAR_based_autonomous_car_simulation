# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_electrode_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED electrode_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(electrode_FOUND FALSE)
  elseif(NOT electrode_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(electrode_FOUND FALSE)
  endif()
  return()
endif()
set(_electrode_CONFIG_INCLUDED TRUE)

# output package information
if(NOT electrode_FIND_QUIETLY)
  message(STATUS "Found electrode: 0.0.0 (${electrode_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'electrode' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${electrode_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(electrode_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${electrode_DIR}/${_extra}")
endforeach()
