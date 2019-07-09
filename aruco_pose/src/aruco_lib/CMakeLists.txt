# aruco_lib: ArUco detection and pose estimation routines
# Lifted directly from OpenCV, modified to not conflict with it

# I'm specifying static linkage here, since I'm not sure where catkin is going to put
# my shared libraries (and whether they're going to be packaged)
add_library(aruco_lib STATIC
        # ArUco detection and pose estimation
        aruco/apriltag_quad_thresh.cpp
        aruco/aruco.cpp
        aruco/charuco.cpp
        aruco/dictionary.cpp
        aruco/zmaxheap.cpp
        # Camera calibration routines (for SolvePnP)
        # cv_calib_camera/ap3p.cpp
        # cv_calib_camera/calib3d_c_api.cpp
        # cv_calib_camera/calibinit.cpp
        # cv_calib_camera/calibration.cpp
        # cv_calib_camera/calibration_handeye.cpp
        # cv_calib_camera/circlesgrid.cpp
        # cv_calib_camera/compat_ptsetreg.cpp
        # cv_calib_camera/compat_stereo.cpp
        # cv_calib_camera/dls.cpp
        # cv_calib_camera/epnp.cpp
        # cv_calib_camera/fisheye.cpp
        # cv_calib_camera/five-point.cpp
        # cv_calib_camera/fundam.cpp
        # cv_calib_camera/homography_decomp.cpp
        # cv_calib_camera/levmarq.cpp
        # cv_calib_camera/main.cpp
        # cv_calib_camera/p3p.cpp
        # cv_calib_camera/polynom_solver.cpp
        # cv_calib_camera/posit.cpp
        # cv_calib_camera/ptsetreg.cpp
        # cv_calib_camera/quadsubpix.cpp
        # cv_calib_camera/rho.cpp
        # cv_calib_camera/solvepnp.cpp
        # cv_calib_camera/stereobm.cpp
        # cv_calib_camera/stereosgbm.cpp
        # cv_calib_camera/triangulate.cpp
        # cv_calib_camera/upnp.cpp
        )

target_link_libraries(aruco_lib
        PUBLIC
        ${OpenCV_LIBRARIES}
        )

target_include_directories(aruco_lib
        PRIVATE
        aruco
        # cv_calib_camera
        )

target_include_directories(aruco_lib
        PUBLIC
        aruco/include
        # cv_calib_camera/include
        ${OpenCV_INCLUDE_DIRS}
        )

target_compile_definitions(aruco_lib
        PRIVATE
        CV_OVERRIDE=override
        )

target_compile_options(aruco_lib
        PRIVATE
        -fpic -fPIC
        )
