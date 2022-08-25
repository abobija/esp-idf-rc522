idf_component_register(
    SRCS src/rc522.c src/rc522_i2c.c src/rc522_spi.c
    INCLUDE_DIRS include
    REQUIRES esp_event
)
