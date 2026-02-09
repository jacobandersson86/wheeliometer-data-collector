idf_component_register(
    SRCS "sample_collection.cpp"
    INCLUDE_DIRS "."
    REQUIRES M5Unified imu_sampler fs rtc terminal driver
)
