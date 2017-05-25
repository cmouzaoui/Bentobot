cmake_minimum_required(VERSION 2.8)
project( SerialCom )
add_executable( SC SC.cpp )
target_link_libraries( SC -lwiringPi)
