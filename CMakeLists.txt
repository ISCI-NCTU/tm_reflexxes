cmake_minimum_required(VERSION 2.8.2)
project(tm_reflexxes)

include_directories(tm_reflexxes/include 
					ReflexxesTypeII/include 
					tm_driver/include
)

file(GLOB rml_SRC
	ReflexxesTypeII/src/*.cpp)

set (tm_driver_SRC 
	tm_driver/src/tm_print.cpp
	tm_driver/src/tm_robot_state_rt.cpp
	tm_driver/src/tm_communication.cpp
	tm_driver/src/tm_driver.cpp)

 set(tm_reflexxes_SRC 
 	tm_reflexxes/src/tm_reflexxes.cpp)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

add_library(RML_LIB          ${rml_SRC}         )
add_library(TM_DRIVER        ${tm_driver_SRC}   )
add_library(TM_REFLEXXES_LIB ${tm_reflexxes_SRC})
target_link_libraries(TM_REFLEXXES_LIB 
	TM_DRIVER 
	RML_LIB)

add_executable(tm_reflexxes  main.cpp  )
add_executable(tm_otg        tm_otg.cpp)

target_link_libraries(tm_reflexxes TM_REFLEXXES_LIB)
target_link_libraries(tm_otg       TM_REFLEXXES_LIB)

install(TARGETS tm_reflexxes RUNTIME DESTINATION bin)
