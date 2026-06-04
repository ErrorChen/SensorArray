idf_component_register(
    SRCS
        "fdc2214Cap.c"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_common driver esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wformat=2
    -Wformat-security
)
