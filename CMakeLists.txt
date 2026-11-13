cmake_minimum_required(VERSION 3.20)
project(ivp LANGUAGES C VERSION 0.1.0)

# ── C17 standard ──────────────────────────────────────────────────────────────
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

if (CMAKE_C_COMPILER_ID STREQUAL "MSVC")
    # MSVC exposes the C17 <stdatomic.h> implementation behind this switch.
    add_compile_options($<$<COMPILE_LANGUAGE:C>:/experimental:c11atomics>)
endif ()

# ── Options ───────────────────────────────────────────────────────────────────
option(IVP_REAL_DOUBLE "Use double as ivp_real_t like the reference IVP_DOUBLE (OFF: float)" ON)
include(cmake/IVPInternalState.cmake)
option(IVP_ENABLE_CHECKPOINTS
       "Memory regions with in-place checkpoints and rewinding (ivp_memory_region.h, ivp_rewind.h)"
       OFF)
# Migrate existing caches without silently changing their Qhull backend.
if (DEFINED IVP_QHULL_MODE)
    if (NOT DEFINED IVP_REENTRANT_QHULL)
        string(TOUPPER "${IVP_QHULL_MODE}" _ivp_old_qhull_mode)
        if (_ivp_old_qhull_mode STREQUAL "REENTRANT")
            set(IVP_REENTRANT_QHULL ON CACHE BOOL "Use Qhull 8.0.2 libqhull_r instead of the bundled original")
        else ()
            set(IVP_REENTRANT_QHULL OFF CACHE BOOL "Use Qhull 8.0.2 libqhull_r instead of the bundled original")
        endif ()
    endif ()
    message(DEPRECATION
            "IVP_QHULL_MODE was replaced by IVP_REENTRANT_QHULL (OFF: the reference engine's bundled Qhull, ON: Qhull 8.0.2 libqhull_r); the new option takes precedence when both are set")
    unset(IVP_QHULL_MODE CACHE)
    unset(IVP_QHULL_MODE)
endif ()
option(IVP_REENTRANT_QHULL "Use Qhull 8.0.2 libqhull_r instead of the bundled original" OFF)
option(IVP_ENABLE_ASSERTS "Enable IVP_ASSERT in Release builds" OFF)
option(IVP_CONTAINER_DEBUG "Enable container debug checks (iterator, bounds, canaries)" OFF)
option(IVP_ENABLE_STATS "Enable runtime statistics counters" OFF)
option(IVP_HOME_BUILD "Compile the engine hooks of the clustering visualizer (reference IVP_HOME_BUILD)" OFF)
option(IVP_MD_INTERPOLATION "Buoyancy multidimensional interpolation (the reference compiles it out with IVP_NO_MD_INTERPOLATION)" OFF)
option(IVP_ENABLE_PARITY_TESTS "Enable C17 vs golden-reference parity tests" OFF)
option(IVP_ENABLE_SCENARIO_TESTS "Enable scenario regression tests (test_scenario_*)" OFF)
option(IVP_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(IVP_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(IVP_ENABLE_TSAN "Enable ThreadSanitizer" OFF)
option(IVP_BUILD_TESTS "Build unit tests" ON)
option(IVP_BUILD_BENCHMARKS "Build the standalone performance benchmark" ${IVP_BUILD_TESTS})
option(IVP_BENCHMARK_DIAGNOSTICS "Enable intrusive benchmark operation counts" OFF)
option(IVP_BUILD_EXAMPLES "Build graphical examples" OFF)

# The core library uses OS threads for inter- and intra-world execution.
find_package(Threads REQUIRED)

function(ivp_apply_strict_fp TARGET_NAME)
    if (MSVC)
        target_compile_options(${TARGET_NAME} PRIVATE /fp:strict /Oi-)
        if (CMAKE_C_COMPILER_ID MATCHES "Clang")
            # clang-cl needs the same explicit contraction rules as Clang.
            target_compile_options(${TARGET_NAME} PRIVATE
                    -ffp-contract=off -fno-fast-math)
        endif ()
    elseif (EMSCRIPTEN)
        # WebAssembly only rounds to nearest, and clang rejects
        # -frounding-math for it (-Wunsupported-floating-point-opt).
        target_compile_options(${TARGET_NAME} PRIVATE
                -ffp-contract=off -fno-fast-math)
    else ()
        target_compile_options(${TARGET_NAME} PRIVATE
                -ffp-contract=off -fno-fast-math -frounding-math)
    endif ()
endfunction()

function(ivp_apply_project_warnings TARGET_NAME)
    if (MSVC)
        target_compile_options(${TARGET_NAME} PRIVATE
                /W4 /WX
                # Aligned engine types deliberately contain tail padding.
                /wd4324
                # Match the non-MSVC policy for callback-shaped interfaces.
                /wd4100)
    else ()
        target_compile_options(${TARGET_NAME} PRIVATE
                -Wall -Wextra -Wpedantic -Werror
                -Wconversion -Wshadow -Wdouble-promotion
                -Wno-unused-parameter
        )
    endif ()
endfunction()

# ── Strict floating-point and warnings will be applied to ivp targets only,
# ── not to third-party dependencies like SDL ──────────────────────────────

# ── Sanitizers ────────────────────────────────────────────────────────────────
if (IVP_ENABLE_ASAN)
    if (MSVC)
        add_compile_options(/fsanitize=address)
    else ()
        add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address)
    endif ()
endif ()

if (IVP_ENABLE_UBSAN)
    if (NOT MSVC)
        add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
        add_link_options(-fsanitize=undefined)
    endif ()
endif ()

if (IVP_ENABLE_TSAN)
    if (MSVC)
        message(FATAL_ERROR "IVP_ENABLE_TSAN is not supported by MSVC")
    endif ()
    if (IVP_ENABLE_ASAN)
        message(FATAL_ERROR
                "IVP_ENABLE_TSAN and IVP_ENABLE_ASAN cannot be enabled together")
    endif ()
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer)
    add_link_options(-fsanitize=thread)
endif ()

# ── WebAssembly (Emscripten) ──────────────────────────────────────────────────
# The wasm build is single-threaded: without -pthread, Threads::Threads is
# empty and the pthread calls are Emscripten's stubs, so environments and
# world schedulers asking for more than one worker fail to create.
if (EMSCRIPTEN)
    if (IVP_ENABLE_TSAN)
        message(FATAL_ERROR "IVP_ENABLE_TSAN is not supported by Emscripten")
    endif ()
    # The default 64 KiB stack is too small for the great matrix and 3DS
    # import frames, and larger scenes outgrow the initial 16 MiB heap.
    # EXIT_RUNTIME makes main's return value the node process exit code.
    add_link_options(-sALLOW_MEMORY_GROWTH=1 -sSTACK_SIZE=2MB -sEXIT_RUNTIME=1)
endif ()

# ── Compile definitions from options ──────────────────────────────────────────
if (IVP_REAL_DOUBLE)
    add_compile_definitions(IVP_REAL_DOUBLE)
endif ()
if (IVP_ENABLE_ASSERTS)
    add_compile_definitions(IVP_ENABLE_ASSERTS)
endif ()
if (IVP_CONTAINER_DEBUG)
    add_compile_definitions(IVP_CONTAINER_DEBUG)
endif ()
if (IVP_ENABLE_STATS)
    add_compile_definitions(IVP_ENABLE_STATS)
endif ()
if (IVP_MD_INTERPOLATION)
    add_compile_definitions(IVP_MD_INTERPOLATION)
endif ()
if (IVP_HOME_BUILD)
    add_compile_definitions(IVP_HOME_BUILD)
endif ()

# ── Source files ──────────────────────────────────────────────────────────────
set(IVP_BASE_SOURCES
        src/base/ivp_assert.c
        src/base/ivp_allocator.c
        src/base/ivp_arena.c
        src/base/ivp_message.c
        src/base/ivp_worker_runtime.c
        src/base/ivp_worker_group.c
        src/base/ivp_task_pool.c
)

set(IVP_MATH_SOURCES
        src/math/ivp_vec3.c
        src/math/ivp_mat3.c
        src/math/ivp_quat.c
        src/math/ivp_transform.c
        src/math/ivp_fast_math.c
        src/math/ivp_geometry.c
        src/math/ivp_mapping.c
)

set(IVP_CONTAINER_SOURCES
        src/containers/ivp_hashmap.c
        src/containers/ivp_minheap.c
        src/containers/ivp_vhash.c
        src/containers/ivp_minlist.c
        src/containers/ivp_min_hash.c
        src/containers/ivp_set.c
        src/containers/ivp_diff_hash.c
)

set(IVP_GEOMETRY_SOURCES
        src/geometry/ivp_compact_edge.c
        src/geometry/ivp_compact_surface.c
        src/geometry/ivp_ledgetree.c
        src/geometry/ivp_surman.c
        src/geometry/ivp_surman_grid.c
        src/geometry/ivp_compact_ledge_geom.c
)

set(IVP_BUILDER_SOURCES
        src/builder/ivp_build_box.c
        src/builder/ivp_build_ledge.c
        src/builder/ivp_build_convex_qhull.c
        src/builder/ivp_build_polygon_convex.c
        src/builder/ivp_rot_inertia.c
        src/builder/ivp_build_ledge_soup.c
        src/builder/ivp_build_bvh_bottomup.c
        src/builder/ivp_build_polygon.c
        src/builder/ivp_build_pointsoup.c
        src/builder/ivp_convex_decompose.c
        src/builder/ivp_geompack.c
        src/builder/ivp_build_concave.c
        src/builder/ivp_build_halfspacesoup.c
        src/builder/ivp_build_compact_recursive.c
        src/builder/ivp_compact_modify.c
        src/builder/ivp_import_3ds.c
        src/builder/ivp_build_q12bsp.c
        src/builder/ivp_build_grid.c
        src/builder/ivp_gridbuild_array.c
)

set(IVP_COLLISION_SOURCES
        src/collision/ivp_collision_tolerance.c
        src/collision/ivp_narrowphase.c
        src/collision/ivp_compact_ledge_solver.c
        src/collision/ivp_sphere_solver.c
        src/collision/ivp_hull_mgr.c
        src/collision/ivp_ov_tree.c
        src/collision/ivp_ov_query.c
        src/collision/ivp_mindist_core.c
        src/collision/ivp_collision_diff.c
        src/collision/ivp_mindist_minimize.c
        src/collision/ivp_mindist_event.c
        src/collision/ivp_ray_solver.c
        src/collision/ivp_range_manager.c
        src/collision/ivp_collision_filter.c
        src/collision/ivp_radar.c
)

set(IVP_DYNAMICS_SOURCES
        src/dynamics/ivp_core.c
        src/dynamics/ivp_object.c
        src/dynamics/ivp_registry.c
        src/dynamics/ivp_object_system.c
        src/dynamics/ivp_environment.c
        src/dynamics/ivp_sim_unit.c
        src/dynamics/ivp_time_manager.c
        src/dynamics/ivp_object_pipeline.c
        src/dynamics/ivp_phantom.c
        src/dynamics/ivp_object_attach.c
        src/dynamics/ivp_merge_core.c
        src/dynamics/ivp_anomaly_manager.c
        src/dynamics/ivp_universe_manager.c
        src/dynamics/ivp_cluster.c
        src/dynamics/ivp_world_scheduler.c
)
if (IVP_ENABLE_CHECKPOINTS)
    list(APPEND IVP_BASE_SOURCES src/base/ivp_memory_region.c)
    list(APPEND IVP_DYNAMICS_SOURCES src/dynamics/ivp_rewind.c)
endif ()

set(IVP_CONTROLLER_SOURCES
        src/controllers/ivp_controller_system.c
        src/controllers/ivp_controller_motion.c
        src/controllers/ivp_actuator_spring.c
        src/controllers/ivp_forcefield.c
        src/controllers/ivp_constraint.c
        src/controllers/ivp_controller_stiff_spring.c
        src/controllers/ivp_controller_phantom.c
        src/controllers/ivp_controller_world_friction.c
        src/controllers/ivp_controller_buoyancy.c
        src/controllers/ivp_buoyancy_solver.c
        src/controllers/ivp_controller_raycast_car.c
        src/controllers/ivp_controller_raycast.c
        src/controllers/ivp_actuator_rot_mot.c
        src/controllers/ivp_actuator_force.c
        src/controllers/ivp_actuator_torque.c
        src/controllers/ivp_actuator_stabilizer.c
        src/controllers/ivp_actuator_suspension.c
        src/controllers/ivp_actuator_check_dist.c
        src/controllers/ivp_controller_floating.c
        src/controllers/ivp_constraint_local.c
        src/controllers/ivp_constraint_local_core.c
        src/controllers/ivp_constraint_fixed_keyed.c
        src/controllers/ivp_controller_airboat.c
        src/controllers/ivp_controller_fake_jetski.c
        src/controllers/ivp_car_system.c
        src/controllers/ivp_constraint_car.c
        src/controllers/ivp_car_wheel_physics.c
        src/controllers/ivp_liquid_surface.c
        src/controllers/ivp_controller_golem.c
        src/controllers/ivp_attacher_to_cores.c
        src/controllers/ivp_actuator_extra.c
        src/controllers/ivp_controller_factory.c
)

set(IVP_UTILITY_SOURCES
        src/utility/ivp_hash.c
        src/utility/ivp_active_value.c
        src/utility/ivp_string.c
        src/utility/ivp_multidimensional_interp.c
    src/utility/ivp_union_find.c
)

set(IVP_EVENT_SOURCES
        src/events/ivp_event_system.c
        src/events/ivp_listener_collision.c
        src/events/ivp_listener_psi.c
)

set(IVP_DIAGNOSTICS_SOURCES
        src/diagnostics/ivp_performance_counter.c
        src/diagnostics/ivp_better_statistics_manager.c
        src/diagnostics/ivp_statistics_console_callback.c
        src/diagnostics/ivp_debug_manager.c
        src/diagnostics/ivp_clustering_visualizer.c
        src/diagnostics/ivp_statistic_manager.c
)

set(IVP_SOLVER_SOURCES
        src/solver/ivp_great_matrix.c
        src/solver/ivp_linear_constraint_solver.c
        src/solver/ivp_solver_core_reaction.c
        src/solver/ivp_contact_point.c
        src/solver/ivp_fr_system.c
        src/solver/ivp_impact_core.c
)

# ── Third-party: external Qhull dependency and private adapter seam ───────
include(cmake/IVPQhull.cmake)
ivp_add_qhull_backend()

# ── Library target ────────────────────────────────────────────────────────
add_library(ivp STATIC
        ${IVP_BASE_SOURCES}
        ${IVP_MATH_SOURCES}
        ${IVP_CONTAINER_SOURCES}
        ${IVP_GEOMETRY_SOURCES}
        ${IVP_BUILDER_SOURCES}
        ${IVP_COLLISION_SOURCES}
        ${IVP_DYNAMICS_SOURCES}
        ${IVP_CONTROLLER_SOURCES}
        ${IVP_EVENT_SOURCES}
        ${IVP_SOLVER_SOURCES}
        ${IVP_UTILITY_SOURCES}
        ${IVP_DIAGNOSTICS_SOURCES}
)

target_include_directories(ivp
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_definitions(ivp PRIVATE $<$<CONFIG:Debug>:IVP_ENABLE_ASSERTS>)
target_compile_definitions(ivp PUBLIC
        IVP_STABLE_INTERNAL_STATE=$<BOOL:${IVP_STABLE_INTERNAL_STATE}>
        IVP_EDIT_LOCALITY=$<BOOL:${IVP_EDIT_LOCALITY}>
        IVP_ENABLE_CHECKPOINTS=$<BOOL:${IVP_ENABLE_CHECKPOINTS}>)
if (IVP_REAL_DOUBLE)
    # ivp_real_t is part of the public ABI: consumers must see the same type
    target_compile_definitions(ivp PUBLIC IVP_REAL_DOUBLE)
endif ()

if (IVP_BUILD_TESTS)
    target_compile_definitions(ivp PRIVATE IVP_ENABLE_TEST_CUSTOM_CONTROLLER_API)
endif ()

ivp_apply_strict_fp(ivp)

# The reference pipeline port mixes IVP_FLOAT and IVP_DOUBLE on purpose
# (float storage, double arithmetic) exactly like the C++ reference.
set(IVP_REFERENCE_PIPELINE_SOURCES
        src/collision/ivp_hull_mgr.c
        src/collision/ivp_ov_tree.c
        src/collision/ivp_mindist_core.c
        src/collision/ivp_mindist_minimize.c
        src/collision/ivp_mindist_event.c
        src/solver/ivp_solver_core_reaction.c
        src/solver/ivp_contact_point.c
        src/solver/ivp_fr_system.c
        src/solver/ivp_impact_core.c
        src/dynamics/ivp_core.c
        src/dynamics/ivp_sim_unit.c
        src/dynamics/ivp_time_manager.c
        src/dynamics/ivp_object_pipeline.c
        src/dynamics/ivp_phantom.c
        src/dynamics/ivp_environment.c
        src/dynamics/ivp_merge_core.c
        src/dynamics/ivp_object_attach.c
        src/controllers/ivp_car_system.c
        src/controllers/ivp_constraint_car.c
        src/controllers/ivp_car_wheel_physics.c
        src/controllers/ivp_controller_stiff_spring.c
        src/controllers/ivp_constraint_fixed_keyed.c
        src/controllers/ivp_controller_floating.c
        src/controllers/ivp_controller_raycast_car.c
)
if (NOT MSVC)
    set_source_files_properties(${IVP_REFERENCE_PIPELINE_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-double-promotion")
    # Reference-internals builds let the friction and dense solvers reuse
    # stale arena bytes.  Keep both build
    # configurations free of type-based alias assumptions so this development
    # path remains valid.
    set_property(SOURCE
            src/base/ivp_arena.c
            src/solver/ivp_great_matrix.c
            src/solver/ivp_linear_constraint_solver.c
            src/solver/ivp_fr_system.c
            APPEND PROPERTY COMPILE_OPTIONS "-fno-strict-aliasing")
endif ()

if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    # GEOMPACK's f2c translation shifts array base pointers to preserve
    # Fortran's one-based indexing. GCC diagnoses the intentional pointer
    # adjustment as an out-of-bounds access after inlining.
    set_property(SOURCE src/builder/ivp_geompack.c APPEND PROPERTY
            COMPILE_OPTIONS "-Wno-array-bounds")
endif ()

# Apply strict warning flags.
ivp_apply_project_warnings(ivp)

target_link_libraries(ivp PRIVATE ivp_qhull)
target_link_libraries(ivp PUBLIC Threads::Threads)

# Link math library on non-Windows
if (NOT MSVC)
    target_link_libraries(ivp PRIVATE m)
endif ()

# ── Render module (optional, requires SDL3 GPU) ──────────────────────────
option(IVP_BUILD_RENDER "Build the render module (requires SDL3 GPU)" OFF)

# Auto-enable render when examples are requested
if (IVP_BUILD_EXAMPLES AND NOT IVP_BUILD_RENDER)
    set(IVP_BUILD_RENDER ON)
endif ()

if (IVP_BUILD_RENDER)
    include(FetchContent)

    # ── SDL3 ─────────────────────────────────────────────────────────────
    if (EMSCRIPTEN)
        # Emscripten's SDL3 port, the version the emsdk release pins.  Its
        # sdl3-config.cmake exists only once the port has been built, so
        # declare the target here rather than through find_package.  (SDL
        # built from source with FetchContent would inherit -std=c17, and
        # its EM_ASM blocks need a gnu dialect.)
        add_library(SDL3::SDL3 INTERFACE IMPORTED)
        set_target_properties(SDL3::SDL3 PROPERTIES
                INTERFACE_COMPILE_OPTIONS "-sUSE_SDL=3"
                INTERFACE_LINK_LIBRARIES "-sUSE_SDL=3")
    else ()
        # Prefer an installed package, fall back to FetchContent
        find_package(SDL3 QUIET CONFIG)
    endif ()
    if (NOT TARGET SDL3::SDL3)
        FetchContent_Declare(
                SDL3
                GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
                GIT_TAG release-3.4.0
                GIT_SHALLOW TRUE
        )
        set(SDL_SHARED ON CACHE BOOL "" FORCE)
        set(SDL_STATIC OFF CACHE BOOL "" FORCE)
        set(SDL_TEST OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(SDL3)
    endif ()

    # ── GPU backend ──────────────────────────────────────────────────────
    # SDL3 GPU natively.  Its GPU API has no web backend, so the Emscripten
    # build draws with OpenGL ES 3.0 (WebGL2) instead.
    if (EMSCRIPTEN)
        set(IVP_RENDER_BACKEND GLES3)
    else ()
        set(IVP_RENDER_BACKEND SDL_GPU)
    endif ()
    message(STATUS "Render backend: ${IVP_RENDER_BACKEND}")

    if (IVP_RENDER_BACKEND STREQUAL "SDL_GPU")
        find_program(GLSLANG_VALIDATOR_EXECUTABLE glslangValidator)
        if (NOT GLSLANG_VALIDATOR_EXECUTABLE)
            message(FATAL_ERROR "IVP_BUILD_RENDER=ON requires glslangValidator in PATH")
        endif ()

        set(IVP_SHADER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/render/shaders)
        set(IVP_SHADER_BIN_DIR ${CMAKE_BINARY_DIR}/shaders)
        file(MAKE_DIRECTORY ${IVP_SHADER_BIN_DIR})

        # Every shader exists twice: GLSL compiled to SPIR-V, and hand-written
        # MSL copied next to it.  Names are <name>.<vert|frag>.
        set(IVP_SHADER_NAMES
            line3d.vert line2d.vert line.frag
            tri3d_inst.vert tri3d_inst.frag
            shadow.vert shadow.frag
            hud_rect.vert hud_rect.frag
        )
        set(IVP_SHADER_OUTPUTS)
        foreach (SHADER ${IVP_SHADER_NAMES})
            string(REGEX REPLACE ".*\\.(vert|frag)$" "\\1" SHADER_STAGE ${SHADER})
            add_custom_command(
                OUTPUT ${IVP_SHADER_BIN_DIR}/${SHADER}.spv
                COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} -V -S ${SHADER_STAGE}
                        ${IVP_SHADER_SRC_DIR}/${SHADER}.glsl
                        -o ${IVP_SHADER_BIN_DIR}/${SHADER}.spv
                DEPENDS ${IVP_SHADER_SRC_DIR}/${SHADER}.glsl
                COMMENT "Compiling ${SHADER}.glsl -> SPIR-V"
                VERBATIM
            )
            add_custom_command(
                OUTPUT ${IVP_SHADER_BIN_DIR}/${SHADER}.msl
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        ${IVP_SHADER_SRC_DIR}/${SHADER}.msl
                        ${IVP_SHADER_BIN_DIR}/${SHADER}.msl
                DEPENDS ${IVP_SHADER_SRC_DIR}/${SHADER}.msl
                COMMENT "Staging ${SHADER}.msl"
                VERBATIM
            )
            list(APPEND IVP_SHADER_OUTPUTS
                ${IVP_SHADER_BIN_DIR}/${SHADER}.spv
                ${IVP_SHADER_BIN_DIR}/${SHADER}.msl)
        endforeach ()

        add_custom_target(ivp_render_shaders ALL DEPENDS ${IVP_SHADER_OUTPUTS})
        set(IVP_RENDER_BACKEND_SOURCES src/render/ivp_render_sdlgpu.c)
    else ()
        # GLSL ES 3.00 sources compiled into the library as strings
        set(IVP_GLES_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/render/shaders/es3)
        set(IVP_GLES_SHADER_NAMES
            line3d.vert line2d.vert line.frag
            tri3d_inst.vert tri3d_inst.frag
            shadow.vert shadow.frag
            hud_rect.vert hud_rect.frag
        )
        set(IVP_GLES_SHADER_HEADER
            ${CMAKE_CURRENT_BINARY_DIR}/generated/render/ivp_render_gles_shaders.h)
        set(IVP_GLES_SHADER_FILES)
        foreach (SHADER ${IVP_GLES_SHADER_NAMES})
            list(APPEND IVP_GLES_SHADER_FILES ${IVP_GLES_SHADER_DIR}/${SHADER}.glsl)
        endforeach ()
        string(REPLACE ";" "," IVP_GLES_SHADER_LIST "${IVP_GLES_SHADER_NAMES}")
        add_custom_command(
            OUTPUT ${IVP_GLES_SHADER_HEADER}
            COMMAND ${CMAKE_COMMAND}
                    -DSHADER_DIR=${IVP_GLES_SHADER_DIR}
                    -DNAMES=${IVP_GLES_SHADER_LIST}
                    -DOUTPUT=${IVP_GLES_SHADER_HEADER}
                    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/IVPEmbedShaders.cmake
            DEPENDS ${IVP_GLES_SHADER_FILES}
                    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/IVPEmbedShaders.cmake
            COMMENT "Embedding the GLSL ES shaders"
            VERBATIM
        )
        set(IVP_RENDER_BACKEND_SOURCES
            src/render/ivp_render_gles.c ${IVP_GLES_SHADER_HEADER})
    endif ()

    # ── ivp_render static library ────────────────────────────────────────
    set(IVP_RENDER_SOURCES
        src/render/ivp_render_camera.c
        src/render/ivp_render_stream.c
        src/render/ivp_render_draw.c
        src/render/ivp_render_hud.c
        src/render/ivp_render_core.c
        src/render/ivp_render_pick.c
        src/render/ivp_render_frustum.c
        src/render/ivp_render_solid.c
        src/render/ivp_render_capture.c
        src/render/ivp_render_shadow.c
        ${IVP_RENDER_BACKEND_SOURCES}
    )
    add_library(ivp_render STATIC ${IVP_RENDER_SOURCES})
    target_include_directories(ivp_render
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/render
    )
    target_link_libraries(ivp_render PUBLIC SDL3::SDL3)
    target_link_libraries(ivp_render PRIVATE ivp)
    if (NOT MSVC)
        target_link_libraries(ivp_render PRIVATE m)
    endif ()
    if (IVP_RENDER_BACKEND STREQUAL "SDL_GPU")
        add_dependencies(ivp_render ivp_render_shaders)
        target_compile_definitions(ivp_render PRIVATE
            IVP_RENDER_SHADER_DIR_FALLBACK="${IVP_SHADER_BIN_DIR}"
        )
    else ()
        target_include_directories(ivp_render PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}/generated/render)
        if (EMSCRIPTEN)
            # WebGL2, and Asyncify: ivp_renderer_end_frame waits for the
            # browser's next animation frame inside the caller's loop.
            target_link_options(ivp_render INTERFACE
                -sMIN_WEBGL_VERSION=2 -sMAX_WEBGL_VERSION=2 -sASYNCIFY=1)
        endif ()
    endif ()

    ivp_apply_strict_fp(ivp_render)

    # Relax warnings for render module (uses float casts and literals freely)
    if (NOT MSVC)
        target_compile_options(ivp_render PRIVATE
                -Wno-conversion -Wno-double-promotion -Wno-sign-conversion
        )
    endif ()
endif ()

# ── Examples (opt-in, requires render module) ──────────────────────────────────
if (IVP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif ()

# The benchmark and the tests run under node and read and write real files.
# This comes after add_subdirectory(examples), so the browser examples, which
# cannot use the host file system, do not get it.
if (EMSCRIPTEN)
    add_link_options(-sNODERAWFS=1)
endif ()

# ── Benchmarks ────────────────────────────────────────────────────────────────
if (IVP_BUILD_BENCHMARKS)
    add_executable(ivp_benchmark benchmarks/ivp_benchmark.c)
    target_link_libraries(ivp_benchmark PRIVATE ivp)
    if (IVP_BENCHMARK_DIAGNOSTICS)
        target_compile_definitions(ivp PRIVATE IVP_BENCHMARK_DIAGNOSTICS)
        target_compile_definitions(ivp_benchmark PRIVATE IVP_BENCHMARK_DIAGNOSTICS)
        target_include_directories(ivp_benchmark PRIVATE src/containers src/collision)
    endif ()
    if (NOT MSVC)
        target_link_libraries(ivp_benchmark PRIVATE m)
    endif ()
    ivp_apply_strict_fp(ivp_benchmark)
endif ()

# ── Tests ─────────────────────────────────────────────────────────────────────
if (IVP_BUILD_TESTS)
    enable_testing()

    find_package(Python3 COMPONENTS Interpreter)
    # Minimal test framework header is in tests/
    function(ivp_add_test TEST_NAME TEST_SOURCE)
        add_executable(${TEST_NAME} ${TEST_SOURCE})
        target_link_libraries(${TEST_NAME} PRIVATE ivp)
        target_compile_definitions(${TEST_NAME} PRIVATE $<$<CONFIG:Debug>:IVP_ENABLE_ASSERTS>)
        target_include_directories(${TEST_NAME} PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/tests
        )
        if (NOT MSVC)
            target_link_libraries(${TEST_NAME} PRIVATE m)
        endif ()
        add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endfunction()

    ivp_add_test(test_base tests/test_base.c)
    target_include_directories(test_base PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_math tests/test_math.c)
    ivp_add_test(test_containers tests/test_containers.c)
    ivp_add_test(test_geometry tests/test_geometry.c)
    ivp_add_test(test_geometry_parity tests/test_geometry_parity.c)
    ivp_add_test(test_ray_solver tests/test_ray_solver.c)
    ivp_add_test(test_convex_decompose tests/test_convex_decompose.c)
    if (NOT IVP_REENTRANT_QHULL)
        target_compile_definitions(test_convex_decompose PRIVATE
                IVP_QHULL_REFERENCE_LAYOUT)
    endif ()
    ivp_add_test(test_compact_grid tests/test_compact_grid.c)
    ivp_add_test(test_collision tests/test_collision.c)
    ivp_add_test(test_dynamics tests/test_dynamics.c)
    target_include_directories(test_dynamics PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_solver tests/test_solver.c)
    ivp_add_test(test_utility tests/test_utility.c)

    # New coverage-expansion tests
    ivp_add_test(test_controllers tests/test_controllers.c)
    ivp_add_test(test_builders tests/test_builders.c)
    ivp_add_test(test_qhull_concurrency tests/test_qhull_concurrency.c)
    target_link_libraries(test_qhull_concurrency PRIVATE Threads::Threads)
    if (IVP_QHULL_SUPPORTS_PARALLEL_BUILDS)
        target_compile_definitions(ivp_qhull PRIVATE IVP_QHULL_TEST_HOOKS)
        add_executable(test_qhull_overlap tests/test_qhull_overlap.c)
        target_compile_definitions(test_qhull_overlap PRIVATE
                IVP_QHULL_TEST_HOOKS)
        target_include_directories(test_qhull_overlap PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/src/builder
                ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(test_qhull_overlap PRIVATE
                ivp_qhull Threads::Threads)
        ivp_apply_strict_fp(test_qhull_overlap)
        ivp_apply_project_warnings(test_qhull_overlap)
        add_test(NAME test_qhull_overlap COMMAND test_qhull_overlap)
        set_tests_properties(test_qhull_overlap PROPERTIES LABELS "unit")
    endif ()
    ivp_add_test(test_parallel_worlds tests/test_parallel_worlds.c)
    target_include_directories(test_parallel_worlds PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_internal_parallel tests/test_internal_parallel.c)
    target_include_directories(test_internal_parallel PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src)
    if (IVP_STABLE_INTERNAL_STATE)
        ivp_add_test(test_internal_state_stability tests/test_internal_state_stability.c)
        target_include_directories(test_internal_state_stability PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/src)
        set_tests_properties(test_internal_state_stability PROPERTIES
                LABELS "unit")
        ivp_add_test(test_replay_equivalence tests/test_replay_equivalence.c)
        set_tests_properties(test_replay_equivalence PROPERTIES LABELS "unit")
        # the mainline convex construction (reference internals keep the reference's)
        ivp_add_test(test_convex_robustness tests/test_convex_robustness.c)
        set_tests_properties(test_convex_robustness PROPERTIES LABELS "unit")
    endif ()
    if (IVP_EDIT_LOCALITY)
        ivp_add_test(test_locality tests/test_locality.c)
        target_include_directories(test_locality PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/src)
        set_tests_properties(test_locality PROPERTIES LABELS "unit")
    endif ()
    if (IVP_ENABLE_CHECKPOINTS)
        ivp_add_test(test_memory_region tests/test_memory_region.c)
        set_tests_properties(test_memory_region PROPERTIES LABELS "unit")
        ivp_add_test(test_checkpoint tests/test_checkpoint.c)
        set_tests_properties(test_checkpoint PROPERTIES LABELS "unit")
        ivp_add_test(test_rewind tests/test_rewind.c)
        set_tests_properties(test_rewind PROPERTIES LABELS "unit")
    endif ()
    if (IVP_BUILD_BENCHMARKS AND IVP_BENCHMARK_DIAGNOSTICS)
        target_compile_definitions(test_parallel_worlds PRIVATE
                IVP_BENCHMARK_DIAGNOSTICS)
    endif ()
    ivp_add_test(test_convex_layout tests/test_convex_layout.c)
    ivp_add_test(test_anomaly tests/test_anomaly.c)
    ivp_add_test(test_object_lifecycle tests/test_object_lifecycle.c)
    target_include_directories(test_object_lifecycle PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_containers_ext tests/test_containers_ext.c)
    ivp_add_test(test_math_ext tests/test_math_ext.c)
    ivp_add_test(test_math_ref tests/test_math_ref.c)
    ivp_add_test(test_lvec tests/test_lvec.c)
    target_include_directories(test_lvec PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_apply_strict_fp(test_lvec)
    ivp_add_test(test_active_value_ref tests/test_active_value_ref.c)
    ivp_add_test(test_utility_ref tests/test_utility_ref.c)
    ivp_add_test(test_collision_ext tests/test_collision_ext.c)
    target_include_directories(test_collision_ext PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_collision_diff tests/test_collision_diff.c)
    target_include_directories(test_collision_diff PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_events tests/test_events.c)
    ivp_add_test(test_diagnostics tests/test_diagnostics.c)
    ivp_add_test(test_env_services tests/test_env_services.c)
    target_include_directories(test_env_services PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

    # Gap-fill tests (legacy alignment)
    ivp_add_test(test_active_value_ext tests/test_active_value_ext.c)
    ivp_add_test(test_collision_filter tests/test_collision_filter.c)
    ivp_add_test(test_bvh_bottomup tests/test_bvh_bottomup.c)
    ivp_add_test(test_polygon_collision_debug tests/test_polygon_collision_debug.c)
    ivp_add_test(test_fake_jetski tests/test_fake_jetski.c)

    # Reference-faithful dense solvers (Many_Zero Gauss, LU, LCS)
    ivp_add_test(test_great_matrix_port tests/test_great_matrix_port.c)
    # Its scalar differential models must have the same rounding and
    # contraction semantics as the engine, including dynamic fenv tests.
    ivp_apply_strict_fp(test_great_matrix_port)
    ivp_add_test(test_matrix_cache tests/test_matrix_cache.c)
    target_include_directories(test_matrix_cache PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_deferred_deletion tests/test_deferred_deletion.c)
    target_include_directories(test_deferred_deletion PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    # These generated cases include libm results and their stored output bits
    # come from the reference platform.  Keep this golden-data comparison on
    # that platform (macOS arm64); portable solver behavior is covered by
    # test_great_matrix_port on every platform.
    if (APPLE AND CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
        ivp_add_test(test_lcs_parity tests/test_lcs_parity.c)
        ivp_apply_strict_fp(test_lcs_parity)
        set(IVP_HAS_LCS_PARITY_TEST ON)
    endif ()

    # P1 controller tests (stabilizer, suspension, constraint API)
    ivp_add_test(test_p1_controllers tests/test_p1_controllers.c)
    ivp_add_test(test_controller_parity tests/test_controller_parity.c)

    # Real-wheel car system + car constraint solver
    ivp_add_test(test_car_system tests/test_car_system.c)
    target_include_directories(test_car_system PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    ivp_add_test(test_controller_active tests/test_controller_active.c)
    ivp_add_test(test_controller_factory tests/test_controller_factory.c)
    ivp_add_test(test_controller_sets tests/test_controller_sets.c)

    # P3 content pipeline tests
    ivp_add_test(test_content_pipeline tests/test_content_pipeline.c)

    # P2 stability benchmarks
    ivp_add_test(test_stability_benchmarks tests/test_stability_benchmarks.c)

    if (IVP_BUILD_BENCHMARKS)
        # A smoke check, not the long-sample performance comparison. An idle
        # world's creation/warmup can dominate its now near-empty PSI loop.
        add_test(NAME benchmark_quick
                 COMMAND ivp_benchmark --profile quick --check --samples 3 --min-sample-ms 5)
        set_tests_properties(benchmark_quick PROPERTIES TIMEOUT 180 LABELS "perf")
    endif ()

    # Scenario parity runner (not a unit test — a JSONL producer)
    if (IVP_ENABLE_SCENARIO_TESTS)
        add_executable(test_scenarios tests/test_scenarios.c)
        target_link_libraries(test_scenarios PRIVATE ivp)
        target_include_directories(test_scenarios PRIVATE
                ${CMAKE_CURRENT_SOURCE_DIR}/tests
        )
        if (IVP_BENCHMARK_DIAGNOSTICS)
            target_compile_definitions(test_scenarios PRIVATE IVP_BENCHMARK_DIAGNOSTICS)
            target_include_directories(test_scenarios PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
        endif ()
        if (NOT MSVC)
            target_link_libraries(test_scenarios PRIVATE m)
            # Scene setup must round exactly like the reference runner (built
            # with -ffp-contract=off): a fused multiply-add in e.g.
            # -6 + spacing * (i + 1) moves an initial position by an ulp and
            # breaks symmetric ties differently.
            target_compile_options(test_scenarios PRIVATE -ffp-contract=off -fno-fast-math)
        else ()
            target_compile_options(test_scenarios PRIVATE /fp:strict)
        endif ()
    endif ()

    set(IVP_REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
    set(IVP_COMPARE_SCRIPT ${IVP_REPO_ROOT}/tools/compare_jsonl.py)
    set(IVP_REFERENCE_JSONL ${IVP_REPO_ROOT}/tools/testdata/compare_smoke.jsonl)
    set(IVP_SCENARIO_REFDIR ${IVP_REPO_ROOT}/tools/testdata)

    if (Python3_Interpreter_FOUND AND EXISTS ${IVP_COMPARE_SCRIPT} AND EXISTS ${IVP_REFERENCE_JSONL})
        add_test(
                NAME test_compare_jsonl_self
                COMMAND ${Python3_EXECUTABLE}
                ${IVP_COMPARE_SCRIPT}
                ${IVP_REFERENCE_JSONL}
                ${IVP_REFERENCE_JSONL}
                --pos-eps 1e-9
                --vel-eps 1e-9
                --orient-eps 1e-9
                --energy-eps 1e-9
                --time-eps 1e-12
        )
        set_tests_properties(test_compare_jsonl_self PROPERTIES
                LABELS "unit;tools")
    else ()
        message(STATUS "Skipping test_compare_jsonl_self (missing Python3 interpreter or JSONL inputs)")
    endif ()
    if (Python3_Interpreter_FOUND)
        add_test(NAME test_cmake_options
                 COMMAND ${Python3_EXECUTABLE}
                         ${IVP_REPO_ROOT}/tests/test_cmake_options.py ${CMAKE_COMMAND})
        set_tests_properties(test_cmake_options PROPERTIES LABELS "unit;tools")
        add_test(NAME test_verification_runner
                 COMMAND ${Python3_EXECUTABLE}
                         ${IVP_REPO_ROOT}/tests/test_verification_runner.py)
        set_tests_properties(test_verification_runner PROPERTIES LABELS "unit;tools")
        add_test(NAME test_benchmark_tool
                 COMMAND ${Python3_EXECUTABLE} ${IVP_REPO_ROOT}/tests/test_benchmark_tool.py)
        set_tests_properties(test_benchmark_tool PROPERTIES LABELS "unit;perf")
        add_test(NAME test_profile_benchmark
                 COMMAND ${Python3_EXECUTABLE} ${IVP_REPO_ROOT}/tests/test_profile_benchmark.py)
        set_tests_properties(test_profile_benchmark PROPERTIES LABELS "unit;perf")
        if (UNIX)
            add_test(NAME test_parity_compare
                     COMMAND ${Python3_EXECUTABLE}
                             ${IVP_REPO_ROOT}/tests/test_parity_compare.py)
            set_tests_properties(test_parity_compare PROPERTIES LABELS "unit;tools")
        endif ()
        if (IVP_ENABLE_SCENARIO_TESTS)
            add_test(NAME test_scenario_counter
                     COMMAND ${Python3_EXECUTABLE} ${IVP_REPO_ROOT}/tests/test_scenario_counter.py
                             $<TARGET_FILE:test_scenarios> ${IVP_REPO_ROOT}/tools/scenarios.txt)
            set_tests_properties(test_scenario_counter PROPERTIES LABELS "unit;perf" TIMEOUT 120)
        endif ()
        # The runner starts the benchmark binary itself and checks
        # worker-count invariance, which needs threads; the single-threaded
        # wasm build has neither a native binary nor workers.
        if (IVP_BUILD_BENCHMARKS AND NOT EMSCRIPTEN)
            add_test(NAME test_benchmark_runner
                     COMMAND ${Python3_EXECUTABLE} ${IVP_REPO_ROOT}/tests/test_benchmark_runner.py
                             $<TARGET_FILE:ivp_benchmark>
                             $<IF:$<OR:$<CONFIG:Debug>,$<BOOL:${IVP_ENABLE_ASSERTS}>>,--expect-serial-islands,--expect-parallel-islands>)
            # The worker-count invariance cases run large scenes several
            # times; unoptimised Debug builds need well over 30 s for them.
            set_tests_properties(test_benchmark_runner PROPERTIES LABELS "unit;perf" TIMEOUT 120)
        endif ()
    endif ()

    # ── CTest labels ──────────────────────────────────────────────────────
    # unit:       focused unit and verification-tool tests
    # threading:  tests that exercise concurrent execution
    # perf:       performance regression tests
    # stability:  long-horizon physical stability tests
    # validation: scenario generation and comparison tests
    # invariants: scenario-specific invariant tests
    # parity:     bit-exact reference comparison tests
    # smoke:      example smoke tests

    set(IVP_UNIT_TESTS
            test_base test_math test_containers test_geometry
            test_geometry_parity test_ray_solver test_convex_decompose
            test_compact_grid test_collision
            test_dynamics test_solver test_utility test_lvec
            test_controllers test_builders test_qhull_concurrency
            test_parallel_worlds
            test_internal_parallel
            test_convex_layout test_anomaly test_object_lifecycle
            test_containers_ext test_math_ext test_collision_ext test_collision_diff test_events test_diagnostics test_env_services
            test_active_value_ext test_collision_filter test_bvh_bottomup
            test_polygon_collision_debug test_fake_jetski
            test_great_matrix_port test_matrix_cache test_deferred_deletion
            test_p1_controllers test_controller_parity test_car_system
            test_controller_active test_controller_sets test_controller_factory
            test_content_pipeline
    )
    if (IVP_HAS_LCS_PARITY_TEST)
        list(APPEND IVP_UNIT_TESTS test_lcs_parity)
    endif ()
    foreach (T ${IVP_UNIT_TESTS})
        set_tests_properties(${T} PROPERTIES LABELS "unit")
    endforeach ()
    if (IVP_HAS_LCS_PARITY_TEST)
        set_tests_properties(test_lcs_parity PROPERTIES LABELS "unit;parity")
    endif ()

    set(IVP_THREADING_TESTS
            test_qhull_concurrency
            test_parallel_worlds
            test_internal_parallel)
    if (IVP_QHULL_SUPPORTS_PARALLEL_BUILDS)
        list(APPEND IVP_THREADING_TESTS test_qhull_overlap)
    endif ()
    if (IVP_STABLE_INTERNAL_STATE)
        list(APPEND IVP_THREADING_TESTS
                test_internal_state_stability
                test_replay_equivalence)
    endif ()
    foreach (T ${IVP_THREADING_TESTS})
        set_property(TEST ${T} APPEND PROPERTY LABELS threading)
    endforeach ()

    # Render module logic that needs no GPU (matrices, rays, frustum, PNG).
    if (IVP_BUILD_RENDER)
        ivp_add_test(test_render_math tests/test_render_math.c)
        target_link_libraries(test_render_math PRIVATE ivp_render)
        target_include_directories(test_render_math PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/render)
        set_tests_properties(test_render_math PROPERTIES LABELS "unit")

        # The examples' parameter panel takes the mouse as arguments.
        ivp_add_test(test_demo_panel tests/test_demo_panel.c)
        target_link_libraries(test_demo_panel PRIVATE ivp_render)
        target_include_directories(test_demo_panel PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
        set_tests_properties(test_demo_panel PROPERTIES LABELS "unit")

        # The smoothing of the driving examples (no SDL needed).
        ivp_add_test(test_demo_drive tests/test_demo_drive.c)
        target_include_directories(test_demo_drive PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
        set_tests_properties(test_demo_drive PROPERTIES LABELS "unit")

        # Exercise the real example pose helpers without opening a window.
        ivp_add_test(test_demo_pose tests/test_demo_pose.c)
        target_link_libraries(test_demo_pose PRIVATE ivp_render)
        target_include_directories(test_demo_pose PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/examples)
        set_tests_properties(test_demo_pose PROPERTIES LABELS "unit")
    endif ()
    set_tests_properties(test_math_ref PROPERTIES LABELS "unit")
    set_tests_properties(test_active_value_ref PROPERTIES LABELS "unit")
    set_tests_properties(test_utility_ref PROPERTIES LABELS "unit")
    # Reference-parity tests evaluate header-inline math in the test TU:
    # compile them with the library's strict FP flags (no FMA contraction).
    if (NOT MSVC)
        target_compile_options(test_math_ref PRIVATE -ffp-contract=off -fno-fast-math)
    endif ()
    set_tests_properties(test_stability_benchmarks PROPERTIES LABELS "perf;stability")

    if (IVP_ENABLE_SCENARIO_TESTS)
        # ── Scenario parity tests (C17 vs reference JSONL) ──────────────
        # Each scenario: run test_scenarios to produce JSONL, then compare
        # against golden reference via compare_jsonl.py.
        # Self-compare mode: run scenario → compare output against itself.
        # When reference JSONL golden files exist, compare against those.

        set(IVP_SCENARIO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/scenario_output)
        file(MAKE_DIRECTORY ${IVP_SCENARIO_OUTPUT_DIR})

        set(IVP_VALIDATION_CONFIG ${IVP_REPO_ROOT}/tools/validation_config.json)

        # Scenario list (name, dt, steps) shared with tools/regen_reference.sh
        # and tools/regen_baselines.sh
        set(IVP_SCENARIO_LIST_FILE ${IVP_REPO_ROOT}/tools/scenarios.txt)
        set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${IVP_SCENARIO_LIST_FILE})
        file(STRINGS ${IVP_SCENARIO_LIST_FILE} IVP_SCENARIO_LINES REGEX "^[a-z]")
        set(IVP_SCENARIOS)
        foreach (LINE ${IVP_SCENARIO_LINES})
            string(REGEX REPLACE "[ \t]+" ";" FIELDS "${LINE}")
            list(GET FIELDS 0 S_NAME)
            list(GET FIELDS 1 S_DT_VALUE)
            list(GET FIELDS 2 S_STEPS_VALUE)
            list(APPEND IVP_SCENARIOS ${S_NAME})
            set(SCENARIO_DT_${S_NAME} ${S_DT_VALUE})
            set(SCENARIO_STEPS_${S_NAME} ${S_STEPS_VALUE})
        endforeach ()

        set(IVP_SCENARIO_BASELINE_DIR ${IVP_REPO_ROOT}/tools/baselines)
        file(MAKE_DIRECTORY ${IVP_SCENARIO_BASELINE_DIR})

        foreach (SCENARIO ${IVP_SCENARIOS})
            set(SCENARIO_OUTPUT ${IVP_SCENARIO_OUTPUT_DIR}/${SCENARIO}.jsonl)
            set(SCENARIO_REF ${IVP_SCENARIO_REFDIR}/ref_${SCENARIO}.jsonl)
            set(SCENARIO_METRICS ${IVP_SCENARIO_OUTPUT_DIR}/metrics_${SCENARIO}.json)
            set(SCENARIO_BASELINE ${IVP_SCENARIO_BASELINE_DIR}/${SCENARIO}.json)
            set(S_STEPS ${SCENARIO_STEPS_${SCENARIO}})
            set(S_DT ${SCENARIO_DT_${SCENARIO}})

        # Generate: run the scenario and produce JSONL
        add_test(
                NAME test_scenario_gen_${SCENARIO}
                COMMAND test_scenarios --scenario ${SCENARIO}
                --steps ${S_STEPS} --dt ${S_DT}
                --output ${SCENARIO_OUTPUT}
        )
        set_tests_properties(test_scenario_gen_${SCENARIO} PROPERTIES
                LABELS "validation"
                TIMEOUT 60
        )

        # Invariant checks: physics correctness without reference comparison
        add_test(
                NAME test_scenario_invariants_${SCENARIO}
                COMMAND test_scenarios --scenario ${SCENARIO}
                --steps ${S_STEPS} --dt ${S_DT}
                --check-invariants
        )
        set_tests_properties(test_scenario_invariants_${SCENARIO} PROPERTIES
                LABELS "invariants;validation"
                TIMEOUT 60
        )

        if (Python3_Interpreter_FOUND AND EXISTS ${IVP_COMPARE_SCRIPT})
            # Self-consistency check: output compares against itself.
            # Orient epsilon is relaxed because float-precision quaternion
            # components don't satisfy |q|²=1 in double (compare_jsonl uses
            # double for the angular metric).
            set(SELF_COMPARE_ARGS
                    ${SCENARIO_OUTPUT} ${SCENARIO_OUTPUT}
            )
            # Use config-driven thresholds if validation_config.json exists
            if (EXISTS ${IVP_VALIDATION_CONFIG})
                list(APPEND SELF_COMPARE_ARGS
                        --config ${IVP_VALIDATION_CONFIG}
                        --scenario ${SCENARIO}
                )
            endif ()

            add_test(
                    NAME test_scenario_self_${SCENARIO}
                    COMMAND ${Python3_EXECUTABLE} ${IVP_COMPARE_SCRIPT}
                    ${SELF_COMPARE_ARGS}
            )
            set_tests_properties(test_scenario_self_${SCENARIO} PROPERTIES
                    DEPENDS test_scenario_gen_${SCENARIO}
                    LABELS "validation"
                    TIMEOUT 15
            )

            # Optional parity comparison against golden JSONL.
            # Correctness should be validated by invariant tests; parity is
            # intentionally opt-in because it can fail on acceptable
            # implementation differences.
            if (IVP_ENABLE_PARITY_TESTS)
                if (EXISTS ${SCENARIO_REF})
                    set(REF_COMPARE_ARGS
                            ${SCENARIO_REF} ${SCENARIO_OUTPUT}
                            --adaptive
                            --save-metrics ${SCENARIO_METRICS}
                    )
                    if (EXISTS ${IVP_VALIDATION_CONFIG})
                        list(APPEND REF_COMPARE_ARGS
                                --config ${IVP_VALIDATION_CONFIG}
                                --scenario ${SCENARIO}
                        )
                    endif ()
                    if (EXISTS ${SCENARIO_BASELINE})
                        list(APPEND REF_COMPARE_ARGS
                                --baseline ${SCENARIO_BASELINE}
                                --max-drift-percent 10.0
                        )
                    endif ()

                    add_test(
                            NAME test_scenario_ref_${SCENARIO}
                            COMMAND ${Python3_EXECUTABLE} ${IVP_COMPARE_SCRIPT}
                            ${REF_COMPARE_ARGS}
                    )
                    set_tests_properties(test_scenario_ref_${SCENARIO} PROPERTIES
                            DEPENDS test_scenario_gen_${SCENARIO}
                            LABELS "parity;validation"
                            TIMEOUT 15
                    )
                else ()
                    message(STATUS "No reference JSONL for ${SCENARIO} (${SCENARIO_REF}). "
                            "Skipping ref comparison — run reference runner to generate.")
                endif ()
            endif ()
        endif ()
        endforeach ()
    else ()
        message(STATUS "Scenario regression tests (test_scenario_*) are disabled (IVP_ENABLE_SCENARIO_TESTS=OFF).")
    endif ()

    # ── Smoke tests for graphical examples ────────────────────────────────
    # Only available when examples are built; requires Python3.
    if (IVP_BUILD_EXAMPLES AND Python3_Interpreter_FOUND)
        set(IVP_SMOKE_SCRIPT ${IVP_REPO_ROOT}/tools/smoke_test_examples.py)
        if (EXISTS ${IVP_SMOKE_SCRIPT})
            # The WebAssembly build runs the examples' .js under node, the
            # toolchain's emulator.
            set(IVP_SMOKE_NODE_ARGS)
            if (EMSCRIPTEN AND CMAKE_CROSSCOMPILING_EMULATOR)
                set(IVP_SMOKE_NODE_ARGS --node ${CMAKE_CROSSCOMPILING_EMULATOR})
            endif ()
            add_test(
                    NAME test_smoke_examples
                    COMMAND ${Python3_EXECUTABLE} ${IVP_SMOKE_SCRIPT}
                    --build-dir ${CMAKE_BINARY_DIR}
                    --timeout 30
                    ${IVP_SMOKE_NODE_ARGS}
            )
            set_tests_properties(test_smoke_examples PROPERTIES
                    LABELS "smoke"
                    TIMEOUT 300
            )
        endif ()
        # examples/COVERAGE.md lists which headers the examples use; it is generated.
        set(IVP_COVERAGE_SCRIPT ${IVP_REPO_ROOT}/tools/example_coverage.py)
        if (EXISTS ${IVP_COVERAGE_SCRIPT})
            add_test(
                    NAME test_example_coverage_table
                    COMMAND ${Python3_EXECUTABLE} ${IVP_COVERAGE_SCRIPT} --check
            )
            set_tests_properties(test_example_coverage_table PROPERTIES LABELS "smoke" TIMEOUT 60)
        endif ()
    endif ()
endif ()
