# examples/CMakeLists.txt — Example programs for IVP
#
# All dependencies (ivp, ivp_render, SDL3) are provided by the
# top-level CMakeLists.txt before add_subdirectory(examples) is called.

add_library(ivp_example_interaction STATIC interaction.c interaction.h)
target_link_libraries(ivp_example_interaction PUBLIC ivp ivp_render SDL3::SDL3)
target_include_directories(ivp_example_interaction
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(ivp_example_inspect STATIC inspect.c inspect.h)
target_include_directories(ivp_example_inspect
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

# ── Runtime files next to the example executables ───────────────────────────
# The render shaders (and the SDL3 shared library, when SDL3 is built from
# source / found as a shared target) are staged ONCE by this target.  Every
# example depends on it.  (A POST_BUILD copy per example raced: with -j N
# several examples copied into the same directory at the same time.)
get_property(IVP_EXAMPLES_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if (IVP_EXAMPLES_MULTI_CONFIG)
    set(IVP_EXAMPLE_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
else ()
    set(IVP_EXAMPLE_BIN_DIR ${CMAKE_CURRENT_BINARY_DIR})
endif ()

set(IVP_EXAMPLE_RUNTIME_COMMANDS
    COMMAND ${CMAKE_COMMAND} -E make_directory ${IVP_EXAMPLE_BIN_DIR}/shaders
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${CMAKE_BINARY_DIR}/shaders ${IVP_EXAMPLE_BIN_DIR}/shaders
)
if (TARGET SDL3::SDL3-shared)
    list(APPEND IVP_EXAMPLE_RUNTIME_COMMANDS
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:SDL3::SDL3-shared> ${IVP_EXAMPLE_BIN_DIR}
    )
endif ()
add_custom_target(ivp_example_runtime ALL
    ${IVP_EXAMPLE_RUNTIME_COMMANDS}
    COMMENT "Staging shaders / SDL3 next to the examples"
    VERBATIM
)
add_dependencies(ivp_example_runtime ivp_render_shaders)

# ── Helper function to add examples ──────────────────────────────────────────
function(ivp_add_example NAME SOURCE)
    add_executable(${NAME} ${SOURCE})
    target_link_libraries(${NAME} PRIVATE ivp_example_interaction ivp_example_inspect)
    add_dependencies(${NAME} ivp_example_runtime)
    if (UNIX AND NOT APPLE AND NOT MSVC)
        target_link_libraries(${NAME} PRIVATE m)
        # Apply floating-point strictness and relax example-specific warnings
        target_compile_options(${NAME} PRIVATE
                -ffp-contract=off -fno-fast-math -frounding-math
                -Wno-conversion -Wno-double-promotion -Wno-sign-conversion
        )
    elseif (NOT MSVC)
        target_compile_options(${NAME} PRIVATE
                -ffp-contract=off -fno-fast-math -frounding-math
                -Wno-conversion -Wno-double-promotion -Wno-sign-conversion
        )
    endif ()
endfunction()

# ── Examples ──────────────────────────────────────────────────────────────────
# All examples are built on demo_app.h: they open a window, or run headless
# with `--headless [frames]` (IVP_EXAMPLE_HEADLESS=N) and finish with a
# self-check (exit code 1 on failure).
ivp_add_example(example_freefall example_freefall.c)
ivp_add_example(example_cubes example_cubes.c)
ivp_add_example(example_shapes example_shapes.c)
ivp_add_example(example_terrain example_terrain.c)
ivp_add_example(example_platform example_platform.c)
ivp_add_example(example_boat example_boat.c)
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

# ── Ports of the reference ivp_examples demos / quickstarts ──────────────────
ivp_add_example(example_bridge example_bridge.c)
ivp_add_example(example_domino example_domino.c)
ivp_add_example(example_onthefly example_onthefly.c)
ivp_add_example(example_raycasting example_raycasting.c)
ivp_add_example(example_snowflake example_snowflake.c)
ivp_add_example(example_universe example_universe.c)
ivp_add_example(example_black_hole example_black_hole.c)
ivp_add_example(example_active_value example_active_value.c)
ivp_add_example(example_callback example_callback.c)
ivp_add_example(example_cut example_cut.c)
ivp_add_example(example_hinge example_hinge.c)
ivp_add_example(example_maxforce example_maxforce.c)
ivp_add_example(example_ring example_ring.c)
ivp_add_example(example_push example_push.c)
