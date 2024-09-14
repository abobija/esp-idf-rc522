idf_component_register(
    INCLUDE_DIRS
        include
    PRIV_INCLUDE_DIRS
        private_include
    SRCS
        src/rc522.c
        src/rc522_helpers.c
        src/rc522_pcd.c
        src/rc522_picc.c
        src/picc/rc522_mifare.c
    REQUIRES
        esp_event
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    #-Werror # Warning as error
    #-pedantic
    -Wextra
    -Wall
    -Wshadow
    -Wcast-qual
    -Wswitch-default
    -Wswitch-enum
    #-Wconversion
    -Wunreachable-code
    -Wuninitialized
    -Winit-self
    -Wpointer-arith
    -Werror-implicit-function-declaration
)
