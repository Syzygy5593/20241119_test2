# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imgsubscriber_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imgsubscriber_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imgsubscriber_FOUND FALSE)
  elseif(NOT imgsubscriber_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imgsubscriber_FOUND FALSE)
  endif()
  return()
endif()
set(_imgsubscriber_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imgsubscriber_FIND_QUIETLY)
  message(STATUS "Found imgsubscriber: 0.0.0 (${imgsubscriber_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imgsubscriber' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${imgsubscriber_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imgsubscriber_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imgsubscriber_DIR}/${_extra}")
endforeach()
