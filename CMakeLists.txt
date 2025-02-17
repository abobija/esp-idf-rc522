idf_component_register(
    INCLUDE_DIRS
        include
    PRIV_INCLUDE_DIRS
        internal
    SRCS
        src/rc522.c
        src/rc522_helpers.c
        src/rc522_pcd.c
        src/rc522_picc.c
        src/picc/rc522_mifare.c
        src/picc/rc522_nxp.c
        src/rc522_driver.c
        src/driver/rc522_spi.c
        src/driver/rc522_i2c.c
    REQUIRES
        esp_event
        # esp_driver_spi # introduced in esp-idf 5.3, autoincluded in 'driver' component
        driver # required for i2c, TODO: migrate to the new API
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Werror # Warning as error
    #-pedantic
    -Wextra
    -Wall
    -Wshadow
    -Wcast-qual
    -Wswitch-default
    #-Wswitch-enum
    #-Wconversion
    -Wunreachable-code
    -Wuninitialized
    -Winit-self
    -Wpointer-arith
    -Werror-implicit-function-declaration
)
