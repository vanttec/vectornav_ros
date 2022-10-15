cmake_minimum_required(VERSION 3.5)
project(libvncxx CXX)

set(CMAKE_SUPPRESS_REGENERATION TRUE)
#add_definitions(-DPL150)
#
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

#option(BUILD_TESTS "Build tests." OFF)
#option(BUILD_BENCHMARKS "Build benchmarks." OFF)
#option(PYTHON "Build for Python library." OFF)
#option(BUILD_GRAPHICS "Build in the graphics library." OFF)

#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(SOURCE
        src/attitude.cpp
        src/compositedata.cpp
        src/conversions.cpp
        src/criticalsection.cpp
        src/dllvalidator.cpp
        src/error_detection.cpp
        src/event.cpp
        src/ezasyncdata.cpp
        src/memoryport.cpp
        src/packet.cpp
        src/packetfinder.cpp
        src/port.cpp
        src/position.cpp
        src/searcher.cpp
        src/sensors.cpp
        src/serialport.cpp
        src/thread.cpp
        src/types.cpp
        src/util.cpp
        src/utilities.cpp
        src/vntime.cpp
        include/vn/exceptions.hpp
        include/vn/matrix.hpp
        include/vn/attitude.hpp
        include/vn/boostpython.hpp
        include/vn/dllvalidator.hpp
        include/vn/signal.hpp
        include/vn/error_detection.hpp
        include/vn/position.hpp
        include/vn/registers.hpp
        include/vn/utilities.hpp
        include/vn/memoryport.hpp
        include/vn/nocopy.hpp
        include/vn/compositedata.hpp
        include/vn/criticalsection.hpp
        include/vn/compiler.hpp
        include/vn/sensors.hpp
        include/vn/searcher.hpp
        include/vn/event.hpp
        include/vn/ezasyncdata.hpp
        include/vn/serialport.hpp
        include/vn/export.hpp
        include/vn/vector.hpp
        include/vn/vntime.hpp
        include/vn/packetfinder.hpp
        include/vn/conversions.hpp
        include/vn/types.hpp
        include/vn/int.hpp
        include/vn/thread.hpp
        include/vn/mock.hpp
        include/vn/port.hpp
        include/vn/util.hpp
        include/vn/consts.hpp
        include/vn/packet.hpp)

include_directories(
    include)

add_library(libvncxx ${SOURCE})
#add_subdirectory(examples/ez_async_data)
#add_subdirectory(examples/getting_started)
#add_subdirectory(examples/math)
#add_subdirectory(examples/uart_protocol)

#
#if (BUILD_GRAPHICS)
#
#	include_directories(
#		../libs/glew/include
#		../libs/glm)
#
#
#	#target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
##	set(CMAKE_PREFIX_PATH "../libs/glew")
##	set(CMAKE_LIBRARY_PATH "../libs/glew/lib/Release/Win32/")
#
#	#message("DIR: ${GLEW_INCLUDE_DIR}")
##	find_package(GLEW REQUIRED)
#	#add_library(GLEW_shared INTERFACE IMPORTED)
#	#message("DIR: ${GLEW_INCLUDE_DIR}")
#
#	#message("${GLEW_FOUND}")
#
##	include_directories(${GLEW_INCLUDE_DIRS})
##    link_libraries(${GLEW_LIBRARIES})
#
#
##    add_subdirectory(../libs/glew GLEW_shared)
#
#
#	add_library(proglib-cpp-graphics SHARED
#		${graphics_sources}
#		${graphics_math_sources})
#	target_compile_definitions(proglib-cpp-graphics PUBLIC -D_BUILD_DLL)
#	target_compile_definitions(proglib-cpp-graphics PUBLIC -Dproglib_cpp_graphics_EXPORTS)
#	target_link_libraries(proglib-cpp-graphics PRIVATE OpenGL32)
#	target_link_libraries(proglib-cpp-graphics PRIVATE GLEW_shared)
#
#
#	
#
#
#endif()
