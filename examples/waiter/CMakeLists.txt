include_directories(${PROJECT_SOURCE_DIR})

add_executable(waiter waiter.cc)
target_link_libraries(waiter
                      drake::drake
                      example_base
                      find_resource
                      gflags)

configure_file(waiter.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/waiter/waiter.yaml COPYONLY)
configure_file(waiter_rotate.yaml 
               ${CMAKE_BINARY_DIR}/idto/examples/waiter/waiter_rotate.yaml COPYONLY)
