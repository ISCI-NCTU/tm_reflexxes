cmake_minimum_required(VERSION 2.8.2)
project(reflexxes_project)

include_directories(ReflexxesTypeII)

file(GLOB reflexxes_SRC
	"ReflexxesTypeII/*.h"
	"ReflexxesTypeII/src/*.cpp"
)


add_executable(01_RMLPositionSampleApplication src/01_RMLPositionSampleApplication.cpp ${reflexxes_SRC})
add_executable(02_RMLPositionSampleApplication src/02_RMLPositionSampleApplication.cpp ${reflexxes_SRC})
add_executable(03_RMLPositionSampleApplication src/03_RMLPositionSampleApplication.cpp ${reflexxes_SRC})
add_executable(07_RMLPositionSampleApplication src/07_RMLPositionSampleApplication.cpp ${reflexxes_SRC})

add_executable(04_RMLVelocitySampleApplication src/04_RMLVelocitySampleApplication.cpp ${reflexxes_SRC})
add_executable(05_RMLVelocitySampleApplication src/05_RMLVelocitySampleApplication.cpp ${reflexxes_SRC})
add_executable(06_RMLVelocitySampleApplication src/06_RMLVelocitySampleApplication.cpp ${reflexxes_SRC})
add_executable(08_RMLVelocitySampleApplication src/08_RMLVelocitySampleApplication.cpp ${reflexxes_SRC})