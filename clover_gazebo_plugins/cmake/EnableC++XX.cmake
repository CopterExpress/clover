# Sets the C++ compiler
# Available: C++11, C++14 and C++17

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if( (${CMAKE_VERSION} VERSION_GREATER "3.8.2") OR (${CMAKE_VERSION} VERSION_EQUAL "3.8.2") ) 

  if(COMPILER_SUPPORTS_CXX17)
    set (CMAKE_CXX_STANDARD 17)
    message(STATUS "Using C++17 compiler")
  elseif(COMPILER_SUPPORTS_CXX14)
    set (CMAKE_CXX_STANDARD 14)
    message(STATUS "Using C++14 compiler")
  elseif(COMPILER_SUPPORTS_CXX11)
    set (CMAKE_CXX_STANDARD 11)
    message(STATUS "Using C++11 compiler")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set (CMAKE_CXX_STANDARD 11)
    message(STATUS "Using C++11 compiler")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 or above support. Please use a different C++ compiler.")
  endif()

else() 

  if(COMPILER_SUPPORTS_CXX14)
    set (CMAKE_CXX_STANDARD 14)
    message(STATUS "Using C++14 compiler")
  elseif(COMPILER_SUPPORTS_CXX11)
    set (CMAKE_CXX_STANDARD 11)
    message(STATUS "Using C++11 compiler")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set (CMAKE_CXX_STANDARD 11)
    message(STATUS "Using C++11 compiler")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 or above support. Please use a different C++ compiler.")
  endif()

endif()


