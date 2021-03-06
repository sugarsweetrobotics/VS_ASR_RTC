if ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")

endif  ("${CMAKE_SYSTEM_NAME}" MATCHES "Linux")
if(WIN32)

find_path(CP2110_DIR SLABHIDtoUART.h
  HINTS "C:/Silabs/MCU/CP2110_4_SDK/Library/Windows/")


find_path(CP2110_INCLUDE_DIR SLABHIDtoUART.h
  HINTS "C:/Silabs/MCU/CP2110_4_SDK/Library/Windows/")

find_path(CP2110_LIB_DIR x86
  HINTS ${CP2110_DIR})

find_library(CP2110_LIBRARY SLABHIDtoUART
  HINTS ${CP2110_DIR}/x86)

set(CP2110_LIBRARY_DIRS ${CP2110_LIB_DIR})

endif(WIN32)
