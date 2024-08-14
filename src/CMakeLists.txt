set(PROGRAM_NAME MCControlUnitree2)
set(CMAKE_INSTALL_RPATH "$ENV{LD_LIBRARY_PATH}")

find_package(mc_rtc REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
#find_package(mc_go2 REQUIRED)

set(HEADERS
  Go2Control.h
  MCControlUnitree2.h
  ControlMode.h
)

set(SOURCES
  main.cpp
  Go2Control.cpp
  MCControlUnitree2.cpp
)

find_package(unitree_sdk2 REQUIRED)

include_directories(
  ${CMAKE_INSTALL_PREFIX}/include/ddscxx
)
link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib/${CMAKE_SYSTEM_PROCESSOR}
)

add_executable(${PROGRAM_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROGRAM_NAME}
  PUBLIC
  mc_rtc::mc_control
  Boost::program_options
  unitree_sdk2
)

install(TARGETS ${PROGRAM_NAME} DESTINATION bin)
