FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c)
FILE(GLOB_RECURSE lib_c_sources ${CMAKE_SOURCE_DIR}/lib/*_C/*.c)

set(component_include_dirs
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/config
	${CMAKE_SOURCE_DIR}/lib/ESP32Encoder
	${CMAKE_SOURCE_DIR}/lib/fpid
	${CMAKE_SOURCE_DIR}/lib/odom_helper
	${CMAKE_SOURCE_DIR}/lib/robotito_omni
	${CMAKE_SOURCE_DIR}/lib/robotito_vl53ring
)

idf_component_register(
	SRCS
		${app_sources}
		${lib_c_sources}
	INCLUDE_DIRS
		${component_include_dirs}
)
