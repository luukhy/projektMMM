cmake_minimum_required(VERSION 3.14)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

set(FETCHCONTENT_BASE_DIR ${CMAKE_SOURCE_DIR}/dependencies CACHE PATH "Dependency base")

#DO MATPLOT++ MOZE BYC POTRZEBNY gnuplot WIEC MOZLIWE ZE TRZEBA BĘDZIE ZAINSTALOWAĆ
FetchContent_Declare(
    matplot
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
    GIT_TAG master
)

# Manually populate the project
FetchContent_MakeAvailable(matplot)


add_executable(MyExecutable main.cpp)
target_link_libraries(MyExecutable PRIVATE matplot)
