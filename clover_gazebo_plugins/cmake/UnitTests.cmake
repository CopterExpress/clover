
option(ENABLE_UNIT_TESTS "Build unit tests for the plugins" OFF)

if(ENABLE_UNIT_TESTS)

    enable_testing()

    find_package(GTest REQUIRED)
    find_package(Threads REQUIRED)

endif(ENABLE_UNIT_TESTS)
