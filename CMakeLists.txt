check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(source_files
    model/futurenet-v2x.cc
    model/sumo-fcd-parser.cc
    model/vehicle-node-mapper.cc
    model/v2x-mobility-importer.cc
    model/rsu-selection-policy.cc
    helper/futurenet-v2x-helper.cc
    helper/rsu-placement-helper.cc
    helper/v2x-scenario-helper.cc
)

set(header_files
    model/futurenet-v2x.h
    model/vehicle-waypoint.h
    model/v2x-common-types.h
    model/sumo-fcd-parser.h
    model/vehicle-node-mapper.h
    model/v2x-mobility-importer.h
    model/rsu-selection-policy.h
    helper/futurenet-v2x-helper.h
    helper/rsu-placement-helper.h
    helper/v2x-scenario-helper.h
)

set(libraries_to_link
    ${libcore}
    ${libnetwork}
    ${libmobility}
    ${libwifi}
    ${libinternet}
    ${libapplications}
    ${libflow-monitor}
)

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/futurenet-v2x-examples-test-suite.cc
        )
endif()

build_lib(
    LIBNAME futurenet-v2x
    SOURCE_FILES ${source_files}
    HEADER_FILES ${header_files}
    LIBRARIES_TO_LINK ${libraries_to_link}
    TEST_SOURCES test/futurenet-v2x-test-suite.cc
                 ${examples_as_tests_sources}
)
