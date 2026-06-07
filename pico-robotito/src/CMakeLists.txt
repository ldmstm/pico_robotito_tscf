FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c)
FILE(GLOB_RECURSE lib_c_sources ${CMAKE_SOURCE_DIR}/lib/*_C/*.c)

set(component_include_dirs
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/config
	${CMAKE_SOURCE_DIR}/lib/ESP32Encoder_C
	${CMAKE_SOURCE_DIR}/lib/fpid_C
	${CMAKE_SOURCE_DIR}/lib/odom_helper_C
	${CMAKE_SOURCE_DIR}/lib/robotito_omni_C
)

idf_component_register(
	SRCS
		${app_sources}
		${lib_c_sources}
	INCLUDE_DIRS
		${component_include_dirs}
)
