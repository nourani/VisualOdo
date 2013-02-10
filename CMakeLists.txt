
PROJECT( VisualOdoGroundCam )

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

ADD_DEFINITIONS( -ggdb -O2 )

find_package(OpenCV REQUIRED)
include_directories(
	${OpenCV_INCLUDE_DIR}
)
link_directories(
	${OpenCV_LINK_DIRECTORIES}
)

ADD_EXECUTABLE(VisualOdoGroundCam
	VisualOdoGroundCam.cpp
)
TARGET_LINK_LIBRARIES(VisualOdoGroundCam
	${OpenCV_LIBS}
)

###############################################################################
##  Install
# Binaries and libs
INSTALL(TARGETS VisualOdoGroundCam
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
)
# Header files
INSTALL(FILES 
	DESTINATION include
)
