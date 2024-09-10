idf_component_register(
    INCLUDE_DIRS include
    SRCS src/rc522.c src/rc522/helpers/spi.c
    REQUIRES esp_event esp_driver_spi
)
