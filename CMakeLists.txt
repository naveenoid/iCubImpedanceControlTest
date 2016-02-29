project(iCubImpedanceControlTest)


CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})

# Include include_directories
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS})

# Add required linker flags
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")


add_executable(iCubImpedanceControlTest main.cpp)

target_link_libraries(iCubImpedanceControlTest ${YARP_LIBRARIES})
install(TARGETS iCubImpedanceControlTest RUNTIME DESTINATION bin)

#add_subdirectory(app)