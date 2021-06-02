set(srcs "pretty_effect.c"
    "spi_master_example_main.c"
    )

# Only ESP32 has enough memory to do jpeg decoding
if(IDF_TARGET STREQUAL "esp32")
    list(APPEND srcs "decode_image.c")
    list(APPEND srcs "IMU_Sensor.c")
    list(APPEND srcs "BMS.c")
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
                    EMBED_FILES image.jpg)

