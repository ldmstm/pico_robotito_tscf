FILE(GLOB_RECURSE app_sources
	${CMAKE_SOURCE_DIR}/src/*.c
)

# list(FILTER app_sources EXCLUDE REGEX ".*/main\\.c$")

idf_component_register(SRCS ${app_sources})
