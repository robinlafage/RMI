# This file assumes there is a environment variable called "TOOLSDIR", which value 
#    is the absolute path to the ciberRato tools base directory
# Before running build.sh, execute: 
#    export TOOLSDIR="/home/$USER/folders/ciberRatoTools"
#      replace "folders" by the path to ciberRatoTools in your setup

find_package(Qt5 COMPONENTS Widgets Multimedia Network Xml REQUIRED)

add_library(RobSock SHARED IMPORTED) # or STATIC instead of SHARED
set_target_properties(RobSock PROPERTIES
  IMPORTED_LOCATION "$ENV{TOOLSDIR}/build/libRobSock/libRobSock.so"
  INTERFACE_INCLUDE_DIRECTORIES "$ENV{TOOLSDIR}/libRobSock"
)

set(c1_SRC
    #Source
    robC1.c
    robfunc.c
)

add_executable(robC1 ${c1_SRC})

target_include_directories(robC1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robC1 RobSock Qt5::Xml Qt5::Widgets)

set_target_properties(robC1 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")


set(c2_SRC
    #Source
    robC2.c
    robfunc.c
)

add_executable(robC2 ${c2_SRC})

target_include_directories(robC2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robC2 RobSock Qt5::Xml Qt5::Widgets)

set_target_properties(robC2 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")


set(c3_SRC
    #Source
    robC3.c
    robfunc.c
)

add_executable(robC3 ${c3_SRC})

target_include_directories(robC3 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robC3 RobSock Qt5::Xml Qt5::Widgets)

set_target_properties(robC3 PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")

