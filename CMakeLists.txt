project(stm32emu LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(stm32emu cpu.cpp bus.cpp olcGUI.cpp)
target_include_directories(stm32emu PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

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