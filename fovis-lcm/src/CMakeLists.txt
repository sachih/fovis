add_executable(fovis-lcm
    main.cpp
    data_capture.cpp
    )
pods_use_pkg_config_packages(fovis-lcm lcmtypes_kinect fovis kinect-utils jpeg-utils)

target_link_libraries(fovis-lcm z)

pods_install_executables(fovis-lcm)
