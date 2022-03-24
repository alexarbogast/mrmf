cmake_minimum_required(VERSION 3.0.2)
project(mrmf_rviz)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
)

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
	find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
	include(${QT_USE_FILE})
	macro(qt_wrap_ui)
		qt4_wrap_ui(${ARGN})
	endmacro()
	macro(qt_wrap_cpp)
		qt4_wrap_cpp(${ARGN})
	endmacro()
else()
	find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
	set(QT_LIBRARIES Qt5::Widgets)
	macro(qt_wrap_ui)
		qt5_wrap_ui(${ARGN})
	endmacro()
	macro(qt_wrap_cpp)
		qt5_wrap_cpp(${ARGN})
	endmacro()
endif()

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
	INCLUDE_DIRS
		${QT_INCLUDE_DIR}
	LIBRARIES
		roscpp
		rviz
	DEPENDS
		QT
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	SYSTEM ${QT_INCLUDE_DIR}
)