# examples/CMakeLists.txt — Example programs for IVP
#
# All dependencies (ivp, ivp_render, SDL3, glad) are provided by the
# top-level CMakeLists.txt before add_subdirectory(examples) is called.

# ── Backward-compatible facade for examples ───────────────────────────────────
add_library(ivp_example_render STATIC render.c render.h)
target_link_libraries(ivp_example_render PUBLIC ivp_render)
target_include_directories(ivp_example_render
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)
# Relax warnings for facade
if (NOT MSVC)
    target_compile_options(ivp_example_render PRIVATE
            -Wno-conversion -Wno-double-promotion -Wno-sign-conversion
    )
endif ()

add_library(ivp_example_interaction STATIC interaction.c interaction.h)
target_link_libraries(ivp_example_interaction PUBLIC ivp SDL3::SDL3)
target_include_directories(ivp_example_interaction
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(ivp_example_inspect STATIC inspect.c inspect.h)
target_include_directories(ivp_example_inspect
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

# ── Helper function to add examples ──────────────────────────────────────────
function(ivp_add_example NAME SOURCE)
    add_executable(${NAME} ${SOURCE})
    target_link_libraries(${NAME} PRIVATE ivp ivp_example_render ivp_example_interaction ivp_example_inspect)
    if (NOT MSVC)
        target_link_libraries(${NAME} PRIVATE m)
        # Apply floating-point strictness and relax example-specific warnings
        target_compile_options(${NAME} PRIVATE
                -ffp-contract=off -fno-fast-math -frounding-math
                -Wno-conversion -Wno-double-promotion -Wno-sign-conversion
        )
    endif ()
    # Copy SDL3 shared library next to the executable so it can be found at runtime.
    if (TARGET SDL3::SDL3-shared)
        add_custom_command(TARGET ${NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:SDL3::SDL3-shared>
                $<TARGET_FILE_DIR:${NAME}>
        )
    endif ()
endfunction()

# ── Examples ──────────────────────────────────────────────────────────────────
ivp_add_example(example_freefall example_freefall.c)
ivp_add_example(example_cubes example_cubes.c)
ivp_add_example(example_springs example_springs.c)
ivp_add_example(example_rope example_rope.c)
ivp_add_example(example_buoyancy example_buoyancy.c)
ivp_add_example(example_motor example_motor.c)
ivp_add_example(example_force_actuator example_force_actuator.c)
ivp_add_example(example_forcefield example_forcefield.c)
ivp_add_example(example_stiff_spring example_stiff_spring.c)
ivp_add_example(example_check_distance example_check_distance.c)
ivp_add_example(example_collision_filter example_collision_filter.c)
ivp_add_example(example_motion_controller example_motion_controller.c)
ivp_add_example(example_phantom example_phantom.c)
ivp_add_example(example_vehicle example_vehicle.c)
ivp_add_example(example_3ds_import example_3ds_import.c)
