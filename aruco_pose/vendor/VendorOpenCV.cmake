# There should be a better way to add ArUco libraries,
# but I have not found it yet.
#  -- sfalexrog, 2019.09.11
if (NOT OpenCV_LIBRARIES OR NOT OpenCV_INCLUDE_DIRS)
  message(FATAL_ERROR "OpenCV was not found - cannot continue")
endif()

message(STATUS "Adding vendored aruco_pose OpenCV module")
add_library(_opencv_aruco STATIC
  vendor/aruco/src/aruco.cpp
  vendor/aruco/src/charuco.cpp
  vendor/aruco/src/dictionary.cpp
  vendor/aruco/src/zmaxheap.cpp
  )

target_include_directories(_opencv_aruco PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(_opencv_aruco PRIVATE
  ${OpenCV_LIBRARIES}
)
target_compile_definitions(_opencv_aruco PRIVATE
  CV_OVERRIDE=override
)
target_compile_options(_opencv_aruco PRIVATE
  -fpic -fPIC
)

target_include_directories(_opencv_aruco PUBLIC
  vendor/aruco/include
)

set(OpenCV_LIBRARIES "_opencv_aruco;${OpenCV_LIBRARIES}")
