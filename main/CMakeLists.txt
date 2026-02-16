idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES M5GFX M5Unified terminal screens buttons rtc fs wifi webserver sample_collection
)
