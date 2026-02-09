idf_component_register(
    SRCS "imu_consumer.cpp" "imu_sampler.cpp"
    INCLUDE_DIRS "."
    REQUIRES M5Unified driver terminal
)
