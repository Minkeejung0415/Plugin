idf_component_register(
    SRCS
        "${CMAKE_SOURCE_DIR}/../../vqf.c"
        "${CMAKE_SOURCE_DIR}/../../sensor_fusion.c"
    INCLUDE_DIRS
        "${CMAKE_SOURCE_DIR}/../.."
    REQUIRES driver
)
