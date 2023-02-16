cmake_minimum_required(VERSION 3.5)
project(OSICheck)

add_subdirectory( lib/open-simulation-interface )

get_directory_property(OSI_VERSION_MAJOR DIRECTORY lib/open-simulation-interface DEFINITION VERSION_MAJOR)
get_directory_property(OSI_VERSION_MINOR DIRECTORY lib/open-simulation-interface DEFINITION VERSION_MINOR)
get_directory_property(OSI_VERSION_PATCH DIRECTORY lib/open-simulation-interface DEFINITION VERSION_PATCH)
set(OSIVERSION "${OSI_VERSION_MAJOR}.${OSI_VERSION_MINOR}.${OSI_VERSION_PATCH}")

include_directories( lib/fmi2/headers ${CMAKE_CURRENT_BINARY_DIR}/src)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LINK_WITH_SHARED_OSI OFF CACHE BOOL "Link FMU with shared OSI library instead of statically linking")
set(PUBLIC_LOGGING OFF CACHE BOOL "Enable logging via FMI logger")
set(PRIVATE_LOGGING OFF CACHE BOOL "Enable private logging to file")

string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID src/modelDescription.in.xml)
configure_file(src/modelDescription.in.xml src/modelDescription.xml @ONLY)

find_package(Protobuf 2.6.1 REQUIRED)
add_library(OSICheck SHARED src/OSICheck.cpp)
set_target_properties(OSICheck PROPERTIES PREFIX "")
target_compile_definitions(OSICheck PRIVATE "FMU_SHARED_OBJECT")
if(LINK_WITH_SHARED_OSI)
	target_link_libraries(OSICheck open_simulation_interface)
else()
	target_link_libraries(OSICheck open_simulation_interface_pic)
endif()

if(WIN32)
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "win64")
	else()
		set(FMI_BINARIES_PLATFORM "win32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "linux64")
	else()
		set(FMI_BINARIES_PLATFORM "linux32")
	endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(FMI_BINARIES_PLATFORM "darwin64")
	else()
		set(FMI_BINARIES_PLATFORM "darwin32")
	endif()
endif()

add_custom_command(TARGET OSICheck
	POST_BUILD
	COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources"
	COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/src/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/OSICheck.cpp" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_SOURCE_DIR}/src/OSICheck.h" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources/"
	COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:OSICheck> $<$<PLATFORM_ID:Windows>:$<$<CONFIG:Debug>:$<TARGET_PDB_FILE:OSICheck>>> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
	COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "../OSICheck.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/sources" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}")
