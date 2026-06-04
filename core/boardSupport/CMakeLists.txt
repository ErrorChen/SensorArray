idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall
    -Wextra
    -Wformat=2
    -Wformat-security
)
