build_lib_example(
    NAME example-grid
    SOURCE_FILES example-grid.cc
    LIBRARIES_TO_LINK ${libfuturenet-v2x}
)

build_lib_example(
    NAME example-map
    SOURCE_FILES example-map.cc
    LIBRARIES_TO_LINK ${libfuturenet-v2x} ${libflow-monitor}
)
