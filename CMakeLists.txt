zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PMW3610 src/pmw3610.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
