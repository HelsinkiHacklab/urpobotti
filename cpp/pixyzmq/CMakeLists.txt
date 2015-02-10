cmake_minimum_required (VERSION 2.8)
project (pixyzmq CXX)

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake" )

# Add sources here... #
add_executable (pixyzmq pixyzmq.cpp)

# libpixyusb should always come before libboost and libusb #
target_link_libraries (pixyzmq pixyusb)

find_package ( libpixyusb REQUIRED )
find_package ( libusb-1.0 REQUIRED )
find_package ( Boost 1.48 COMPONENTS thread system chrono REQUIRED)
find_package ( libzmq1 REQUIRED )

target_link_libraries (pixyzmq ${Boost_LIBRARIES})
target_link_libraries (pixyzmq ${LIBUSB_1_LIBRARY})
target_link_libraries (pixyzmq ${ZEROMQ_LIBRARY})

file(STRINGS "cmake/VERSION" LIBPIXY_VERSION)
add_definitions(-D__LIBPIXY_VERSION__="${LIBPIXY_VERSION}")

include_directories (src
                     include
                     ../../common
                     ${Boost_INCLUDE_DIR}
                     ${LIBUSB_1_INCLUDE_DIRS}
                     ${ZEROMQ_INCLUDE_DIR}
                     )

