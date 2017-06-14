cmake_minimum_required(VERSION 2.8.2)
project(tm_reflexxes)

include_directories(tm_reflexxes/include 
					ReflexxesTypeII/include 
					tm_driver/include
)

file(GLOB TM_REFLEXXEX_SRC
	tm_reflexxes/src/*.cpp
	ReflexxesTypeII/src/*.cpp
	tm_driver/src/tm_print.cpp
	tm_driver/src/tm_robot_state_rt.cpp
	tm_driver/src/tm_communication.cpp
	tm_driver/src/tm_driver.cpp
)

set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

add_executable(tm_reflexxes main.cpp ${TM_REFLEXXEX_SRC})
#add_executable(tm_reflexxes_test src/tm_print.cpp src/tm_robot_state_rt.cpp src/tm_communication.cpp src/tm_driver.cpp ${reflexxes_SRC} tm_reflexxes_test.cpp)
add_executable(tm_otg tm_otg.cpp ${TM_REFLEXXEX_SRC})

install(TARGETS tm_reflexxes RUNTIME DESTINATION bin)
