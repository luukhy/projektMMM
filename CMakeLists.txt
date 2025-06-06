cmake_minimum_required(VERSION 3.14)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/dependencies CACHE PATH "Dependency base")

#DO MATPLOT++ MOZE BYC POTRZEBNY gnuplot WIEC MOZLIWE ZE TRZEBA BĘDZIE ZAINSTALOWAĆ
FetchContent_Declare(
    matplot
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
    GIT_TAG master
)


FetchContent_MakeAvailable(matplot)

if (MSVC)
    add_compile_options(/Zc:__cplusplus)
endif()

if(UNIX AND NOT APPLE)
    set(CMAKE_PREFIX_PATH "/home/piotrek/Qt/6.9.0/gcc_64/lib/cmake")
endif()

if(WIN32)
    set(CMAKE_PREFIX_PATH "D:/QT/6.9.0/msvc2022_64/lib/cmake")
endif()

find_package(Qt6 REQUIRED COMPONENTS Widgets PrintSupport) 

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(MyExecutable 
    main.cpp
    include/mainwindow.h
    src/mainwindow.cpp
    
    include/force.h
    src/force.cpp
    
    include/qcustomplot.h 
    src/qcustomplot.cpp 
)

add_executable(test-executable 
    src/testing-od-piotrka.cpp
    
    include/force.h
    src/force.cpp
)

target_link_libraries(test-executable PRIVATE 
    matplot       
)

target_link_libraries(MyExecutable PRIVATE 
    matplot       
    Qt6::Widgets 
    Qt6::PrintSupport  
)