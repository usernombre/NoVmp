file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS *.cpp *.hpp)

set(NOVMP_DEMO_TARGET NoVmpDemo)
add_executable(${NOVMP_DEMO_TARGET} ${SOURCES})


set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_include_directories(${NOVMP_DEMO_TARGET} PRIVATE ../linux-pe/includes)
target_link_libraries(${NOVMP_DEMO_TARGET} PRIVATE VTIL Threads::Threads)