# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "CMakeFiles\\ComputtationalGeometryApp_autogen.dir\\AutogenUsed.txt"
  "CMakeFiles\\ComputtationalGeometryApp_autogen.dir\\ParseCache.txt"
  "ComputtationalGeometryApp_autogen"
  )
endif()
