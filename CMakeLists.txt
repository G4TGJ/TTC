add_library(TTC INTERFACE EXCLUDE_FROM_ALL)

target_sources(TTC INTERFACE
    main.c
)

target_include_directories(TTC INTERFACE . ${CMAKE_SOURCE_DIR})
