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

# The GLES backend compiles its shaders into the library.
set(IVP_EXAMPLE_RUNTIME_COMMANDS
    COMMAND ${CMAKE_COMMAND} -E make_directory ${IVP_EXAMPLE_BIN_DIR}
)
if (TARGET ivp_render_shaders)
    list(APPEND IVP_EXAMPLE_RUNTIME_COMMANDS
        COMMAND ${CMAKE_COMMAND} -E make_directory ${IVP_EXAMPLE_BIN_DIR}/shaders
        COMMAND ${CMAKE_COMMAND} -E copy_directory
                ${CMAKE_BINARY_DIR}/shaders ${IVP_EXAMPLE_BIN_DIR}/shaders
    )
endif ()
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
if (TARGET ivp_render_shaders)
    add_dependencies(ivp_example_runtime ivp_render_shaders)
endif ()

# ── Browser build (Emscripten) ───────────────────────────────────────────────
# Each example becomes example_<name>.html (+ .js, .wasm) from web/shell.html.
# The renderer waits for the browser's next animation frame inside the
# example's own loop (Asyncify).  Every call on the way there is a direct
# call (main, demo_run, demo_run_viewer, demo_viewer_end_frame,
# ivp_renderer_end_frame), so indirect calls are left uninstrumented.  The
# examples also run headless under node (`node example_<name>.js --headless`).
if (EMSCRIPTEN)
    set(IVP_EXAMPLE_SHELL ${CMAKE_CURRENT_SOURCE_DIR}/web/shell.html)
    set(IVP_EXAMPLE_WEB_LINK_OPTIONS
        --shell-file ${IVP_EXAMPLE_SHELL}
        -sENVIRONMENT=web,node
        -sASYNCIFY_IGNORE_INDIRECT=1
        -sASYNCIFY_STACK_SIZE=65536
    )
endif ()

# ── Helper function to add examples ──────────────────────────────────────────
function(ivp_add_example NAME SOURCE)
    set_property(GLOBAL APPEND PROPERTY IVP_EXAMPLE_TARGETS ${NAME})
    add_executable(${NAME} ${SOURCE})
    target_link_libraries(${NAME} PRIVATE ivp_example_interaction ivp_example_inspect)
    add_dependencies(${NAME} ivp_example_runtime)
    if (NOT MSVC)
        if (UNIX AND NOT APPLE)
            target_link_libraries(${NAME} PRIVATE m)
        endif ()
        # Apply floating-point strictness and relax example-specific warnings
        ivp_apply_strict_fp(${NAME})
        target_compile_options(${NAME} PRIVATE
                -Wno-conversion -Wno-double-promotion -Wno-sign-conversion
        )
    endif ()
    if (EMSCRIPTEN)
        set_target_properties(${NAME} PROPERTIES
            SUFFIX ".html"
            LINK_DEPENDS ${IVP_EXAMPLE_SHELL})
        target_link_options(${NAME} PRIVATE ${IVP_EXAMPLE_WEB_LINK_OPTIONS})
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
ivp_add_example(example_airboat example_airboat.c)
ivp_add_example(example_jetski example_jetski.c)
ivp_add_example(example_wheelcar example_wheelcar.c)
ivp_add_example(example_factory example_factory.c)
ivp_add_example(example_rolling_ball example_rolling_ball.c)
ivp_add_example(example_friction example_friction.c)
ivp_add_example(example_concave example_concave.c)
ivp_add_example(example_psi_listener example_psi_listener.c)
ivp_add_example(example_constraints example_constraints.c)
ivp_add_example(example_actuators example_actuators.c)
ivp_add_example(example_waves example_waves.c)
ivp_add_example(example_hover example_hover.c)
ivp_add_example(example_zones example_zones.c)
ivp_add_example(example_contacts example_contacts.c)
ivp_add_example(example_sensors example_sensors.c)
ivp_add_example(example_workshop example_workshop.c)
ivp_add_example(example_compound example_compound.c)
ivp_add_example(example_offroad example_offroad.c)
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

get_property(IVP_ALL_EXAMPLES GLOBAL PROPERTY IVP_EXAMPLE_TARGETS)
if (EMSCRIPTEN)
    # ── The browser index: the gallery's cards as a page of links ────────────
    # (a page cannot start processes, so there is no gallery executable)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    string(REPLACE ";" "," IVP_ALL_EXAMPLES_LIST "${IVP_ALL_EXAMPLES}")
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/index.html
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/../tools/web/make_index.py
                ${CMAKE_CURRENT_SOURCE_DIR}/example_gallery.c
                ${CMAKE_CURRENT_BINARY_DIR}/index.html
                ${IVP_ALL_EXAMPLES_LIST}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/example_gallery.c
                ${CMAKE_CURRENT_SOURCE_DIR}/../tools/web/make_index.py
        COMMENT "Writing the browser examples' index.html"
        VERBATIM
    )
    add_custom_target(ivp_example_index ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/index.html)
else ()
    # ── The launcher: its headless check lists the executables next to it ────
    ivp_add_example(example_gallery example_gallery.c)
    add_dependencies(example_gallery ${IVP_ALL_EXAMPLES})
endif ()
