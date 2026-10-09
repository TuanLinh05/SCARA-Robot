cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(scara_cartesian)
target_sources(app PRIVATE
    src/main.c
    src/cart_core.c
    src/cart_commands.c
    src/cart_telemetry.c
    src/command_text.c
    src/command_stream.c
    src/jog_core.c
    src/home_worker.c
)
