cmake_minimum_required(VERSION 3.16)
project(stm32emu LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Core emulator: no GUI dependencies, shared by the app and the tests.
add_library(stm32core cpu.cpp bus.cpp)
target_include_directories(stm32core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(stm32emu olcGUI.cpp)
target_link_libraries(stm32emu PRIVATE stm32core)

find_package(X11 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(PNG REQUIRED)
find_package(ZLIB REQUIRED)

target_include_directories(stm32emu PRIVATE ${X11_INCLUDE_DIR} ${PNG_INCLUDE_DIRS})
target_link_libraries(stm32emu PRIVATE
  ${X11_LIBRARIES}
  OpenGL::GL
  PNG::PNG
  ZLIB::ZLIB
  pthread
)

# Tests
enable_testing()
add_executable(test_mem tests/test_mem.cpp)
target_link_libraries(test_mem PRIVATE stm32core)
add_test(NAME test_mem COMMAND test_mem)

add_executable(test_reset tests/test_reset.cpp)
target_link_libraries(test_reset PRIVATE stm32core)
add_test(NAME test_reset COMMAND test_reset)
