cmake_minimum_required(VERSION 3.20)
project(ivp LANGUAGES C VERSION 0.1.0)

# ── C17 standard ──────────────────────────────────────────────────────────────
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# ── Options ───────────────────────────────────────────────────────────────────
option(IVP_REAL_DOUBLE "Use double as ivp_real_t (default: float)" OFF)
option(IVP_ENABLE_ASSERTS "Enable IVP_ASSERT in Release builds" OFF)
option(IVP_CONTAINER_DEBUG "Enable container debug checks (iterator, bounds, canaries)" OFF)
option(IVP_ENABLE_STATS "Enable runtime statistics counters" OFF)
option(IVP_ENABLE_PARITY_TESTS "Enable C17 vs golden-reference parity tests" OFF)
option(IVP_ENABLE_ASAN "Enable AddressSanitizer" OFF)
option(IVP_ENABLE_UBSAN "Enable UndefinedBehaviorSanitizer" OFF)
option(IVP_BUILD_TESTS "Build unit tests" ON)
option(IVP_BUILD_EXAMPLES "Build graphical examples" OFF)

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

# ── Source files ──────────────────────────────────────────────────────────────
set(IVP_BASE_SOURCES
        src/base/ivp_assert.c
        src/base/ivp_allocator.c
        src/base/ivp_arena.c
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
        src/containers/ivp_elem_typeinfo.c
        src/containers/ivp_array.c
        src/containers/ivp_hashmap.c
        src/containers/ivp_minheap.c
        src/containers/ivp_vhash.c
        src/containers/ivp_minlist.c
)

set(IVP_GEOMETRY_SOURCES
        src/geometry/ivp_compact_edge.c
        src/geometry/ivp_compact_surface.c
        src/geometry/ivp_ledgetree.c
        src/geometry/ivp_surman.c
        src/geometry/ivp_surman_grid.c
)

set(IVP_BUILDER_SOURCES
        src/builder/ivp_build_box.c
        src/builder/ivp_build_convex_qhull.c
        src/builder/ivp_rot_inertia.c
        src/builder/ivp_build_ledge_soup.c
        src/builder/ivp_build_bvh_bottomup.c
        src/builder/ivp_build_polygon.c
        src/builder/ivp_build_pointsoup.c
        src/builder/ivp_convex_decompose.c
        src/builder/ivp_build_concave.c
        src/builder/ivp_build_halfspacesoup.c
        src/builder/ivp_build_compact_recursive.c
        src/builder/ivp_compact_modify.c
        src/builder/ivp_import_3ds.c
        src/builder/ivp_build_q12bsp.c
        src/builder/ivp_build_grid.c
)

set(IVP_COLLISION_SOURCES
        src/collision/ivp_collision_tolerance.c
        src/collision/ivp_narrowphase.c
        src/collision/ivp_compact_ledge_solver.c
        src/collision/ivp_sphere_solver.c
        src/collision/ivp_mindist.c
        src/collision/ivp_mindist_event.c
        src/collision/ivp_mindist_recursive.c
        src/collision/ivp_oo_watcher.c
        src/collision/ivp_synapse.c
        src/collision/ivp_time_event_queue.c
        src/collision/ivp_ray_solver.c
        src/collision/ivp_range_manager.c
        src/collision/ivp_collision_filter.c
        src/collision/ivp_3d_solver.c
        src/collision/ivp_matrix_cache.c
        src/collision/ivp_broadphase_sap.c
        src/collision/ivp_mindist_persistent.c
        src/collision/ivp_hull_manager.c
        src/collision/ivp_ledgetree_collision.c
        src/collision/ivp_ccd_narrowphase.c
        src/collision/ivp_radar.c
)

set(IVP_DYNAMICS_SOURCES
        src/dynamics/ivp_core.c
        src/dynamics/ivp_object.c
        src/dynamics/ivp_registry.c
        src/dynamics/ivp_object_system.c
        src/dynamics/ivp_environment.c
        src/dynamics/ivp_environment_collision.c
        src/dynamics/ivp_sim_unit.c
        src/dynamics/ivp_object_attach.c
        src/dynamics/ivp_merge_core.c
        src/dynamics/ivp_anomaly_manager.c
        src/dynamics/ivp_universe_manager.c
        src/dynamics/ivp_cluster.c
)

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
        src/controllers/ivp_controller_raycast_car.c
        src/controllers/ivp_actuator_rot_mot.c
        src/controllers/ivp_actuator_force.c
        src/controllers/ivp_actuator_torque.c
        src/controllers/ivp_actuator_stabilizer.c
        src/controllers/ivp_actuator_suspension.c
        src/controllers/ivp_actuator_check_dist.c
        src/controllers/ivp_controller_floating.c
        src/controllers/ivp_constraint_local.c
        src/controllers/ivp_constraint_fixed_keyed.c
        src/controllers/ivp_controller_airboat.c
        src/controllers/ivp_controller_fake_jetski.c
        src/controllers/ivp_car_system.c
        src/controllers/ivp_liquid_surface.c
        src/controllers/ivp_controller_golem.c
)

set(IVP_UTILITY_SOURCES
        src/utility/ivp_hash.c
        src/utility/ivp_active_value.c
        src/utility/ivp_multidimensional_interp.c
    src/utility/ivp_union_find.c
)

set(IVP_EVENT_SOURCES
        src/events/ivp_event_system.c
        src/events/ivp_listener_collision.c
        src/events/ivp_listener_psi.c
)

set(IVP_SOLVER_SOURCES
        src/solver/ivp_great_matrix.c
        src/solver/ivp_contact.c
        src/solver/ivp_impact.c
        src/solver/ivp_friction_system.c
        src/solver/ivp_mindist_friction_update.c
        src/solver/ivp_impact_system.c
        src/solver/ivp_solver_config.c
        src/solver/ivp_solver_metrics.c
        src/solver/ivp_contact_order.c
)

# ── Third-party: QHull (bundled, reentrant C API) ─────────────────────────
set(QHULL_R_SOURCES
        deps/qhull/src/libqhull_r/accessors_r.c
        deps/qhull/src/libqhull_r/geom2_r.c
        deps/qhull/src/libqhull_r/geom_r.c
        deps/qhull/src/libqhull_r/global_r.c
        deps/qhull/src/libqhull_r/io_r.c
        deps/qhull/src/libqhull_r/libqhull_r.c
        deps/qhull/src/libqhull_r/mem_r.c
        deps/qhull/src/libqhull_r/merge_r.c
        deps/qhull/src/libqhull_r/poly2_r.c
        deps/qhull/src/libqhull_r/poly_r.c
        deps/qhull/src/libqhull_r/qset_r.c
        deps/qhull/src/libqhull_r/random_r.c
        deps/qhull/src/libqhull_r/stat_r.c
        deps/qhull/src/libqhull_r/user_r.c
        deps/qhull/src/libqhull_r/usermem_r.c
        deps/qhull/src/libqhull_r/userprintf_r.c
)

add_library(ivp_qhull_r STATIC ${QHULL_R_SOURCES})
target_include_directories(ivp_qhull_r
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/deps/qhull/src
        ${CMAKE_CURRENT_SOURCE_DIR}/deps/qhull/src/libqhull_r
)

# Apply strict floating-point rules
if (MSVC)
    target_compile_options(ivp_qhull_r PRIVATE /fp:strict /Oi-)
    if (CMAKE_C_COMPILER_ID MATCHES "Clang")
        target_compile_options(ivp_qhull_r PRIVATE -ffp-contract=off -fno-fast-math)
    endif ()
else ()
    target_compile_options(ivp_qhull_r PRIVATE
            -ffp-contract=off -fno-fast-math -frounding-math
    )
endif ()

# Relax warnings for third-party code only
if (MSVC)
    target_compile_options(ivp_qhull_r PRIVATE /W0)
else ()
    target_compile_options(ivp_qhull_r PRIVATE
            -Wno-error
            -Wno-conversion
            -Wno-shadow
            -Wno-double-promotion
            -Wno-unused-parameter
            -Wno-sign-conversion
    )
endif ()

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
)

target_include_directories(ivp
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/deps/qhull/src
)

target_compile_definitions(ivp PRIVATE $<$<CONFIG:Debug>:IVP_ENABLE_ASSERTS>)

# Apply strict floating-point rules
if (MSVC)
    target_compile_options(ivp PRIVATE /fp:strict /Oi-)
    if (CMAKE_C_COMPILER_ID MATCHES "Clang")
        # clang-cl
        target_compile_options(ivp PRIVATE -ffp-contract=off -fno-fast-math)
    endif ()
else ()
    # GCC / Clang
    target_compile_options(ivp PRIVATE
            -ffp-contract=off -fno-fast-math -frounding-math
    )
endif ()

# Apply strict warning flags
if (MSVC)
    target_compile_options(ivp PRIVATE /W4 /WX)
else ()
    target_compile_options(ivp PRIVATE
            -Wall -Wextra -Wpedantic -Werror
            -Wconversion -Wshadow -Wdouble-promotion
            -Wno-unused-parameter
    )
endif ()

target_link_libraries(ivp PRIVATE ivp_qhull_r)

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

    # ── SDL3: prefer installed package, fallback to FetchContent ─────────
    find_package(SDL3 QUIET CONFIG)
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

    # ── ivp_render static library ────────────────────────────────────────
    find_program(GLSLANG_VALIDATOR_EXECUTABLE glslangValidator)
    if (NOT GLSLANG_VALIDATOR_EXECUTABLE)
        message(FATAL_ERROR "IVP_BUILD_RENDER=ON requires glslangValidator in PATH")
    endif ()

    set(IVP_SHADER_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/render/shaders)
    set(IVP_SHADER_BIN_DIR ${CMAKE_BINARY_DIR}/shaders)
    file(MAKE_DIRECTORY ${IVP_SHADER_BIN_DIR})

    set(IVP_SHADER_SPV_OUTPUTS
        ${IVP_SHADER_BIN_DIR}/line3d.vert.spv
        ${IVP_SHADER_BIN_DIR}/line2d.vert.spv
        ${IVP_SHADER_BIN_DIR}/line.frag.spv
        ${IVP_SHADER_BIN_DIR}/tri3d.vert.spv
        ${IVP_SHADER_BIN_DIR}/tri3d.frag.spv
        ${IVP_SHADER_BIN_DIR}/tri3d_inst.vert.spv
    )

    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/line3d.vert.spv
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} -V -S vert
                ${IVP_SHADER_SRC_DIR}/line3d.vert.glsl
                -o ${IVP_SHADER_BIN_DIR}/line3d.vert.spv
        DEPENDS ${IVP_SHADER_SRC_DIR}/line3d.vert.glsl
        COMMENT "Compiling line3d.vert.glsl -> SPIR-V"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/line2d.vert.spv
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} -V -S vert
                ${IVP_SHADER_SRC_DIR}/line2d.vert.glsl
                -o ${IVP_SHADER_BIN_DIR}/line2d.vert.spv
        DEPENDS ${IVP_SHADER_SRC_DIR}/line2d.vert.glsl
        COMMENT "Compiling line2d.vert.glsl -> SPIR-V"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/line.frag.spv
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} -V -S frag
                ${IVP_SHADER_SRC_DIR}/line.frag.glsl
                -o ${IVP_SHADER_BIN_DIR}/line.frag.spv
        DEPENDS ${IVP_SHADER_SRC_DIR}/line.frag.glsl
        COMMENT "Compiling line.frag.glsl -> SPIR-V"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/tri3d.vert.spv
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} -V -S vert
                ${IVP_SHADER_SRC_DIR}/tri3d.vert.glsl
                -o ${IVP_SHADER_BIN_DIR}/tri3d.vert.spv
        DEPENDS ${IVP_SHADER_SRC_DIR}/tri3d.vert.glsl
        COMMENT "Compiling tri3d.vert.glsl -> SPIR-V"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/tri3d.frag.spv
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} -V -S frag
                ${IVP_SHADER_SRC_DIR}/tri3d.frag.glsl
                -o ${IVP_SHADER_BIN_DIR}/tri3d.frag.spv
        DEPENDS ${IVP_SHADER_SRC_DIR}/tri3d.frag.glsl
        COMMENT "Compiling tri3d.frag.glsl -> SPIR-V"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/tri3d_inst.vert.spv
        COMMAND ${GLSLANG_VALIDATOR_EXECUTABLE} -V -S vert
                ${IVP_SHADER_SRC_DIR}/tri3d_inst.vert.glsl
                -o ${IVP_SHADER_BIN_DIR}/tri3d_inst.vert.spv
        DEPENDS ${IVP_SHADER_SRC_DIR}/tri3d_inst.vert.glsl
        COMMENT "Compiling tri3d_inst.vert.glsl -> SPIR-V"
        VERBATIM
    )

    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/line3d.vert.msl
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${IVP_SHADER_SRC_DIR}/line3d.vert.msl
                ${IVP_SHADER_BIN_DIR}/line3d.vert.msl
        DEPENDS ${IVP_SHADER_SRC_DIR}/line3d.vert.msl
        COMMENT "Staging line3d.vert.msl"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/line2d.vert.msl
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${IVP_SHADER_SRC_DIR}/line2d.vert.msl
                ${IVP_SHADER_BIN_DIR}/line2d.vert.msl
        DEPENDS ${IVP_SHADER_SRC_DIR}/line2d.vert.msl
        COMMENT "Staging line2d.vert.msl"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/line.frag.msl
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${IVP_SHADER_SRC_DIR}/line.frag.msl
                ${IVP_SHADER_BIN_DIR}/line.frag.msl
        DEPENDS ${IVP_SHADER_SRC_DIR}/line.frag.msl
        COMMENT "Staging line.frag.msl"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/tri3d.vert.msl
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${IVP_SHADER_SRC_DIR}/tri3d.vert.msl
                ${IVP_SHADER_BIN_DIR}/tri3d.vert.msl
        DEPENDS ${IVP_SHADER_SRC_DIR}/tri3d.vert.msl
        COMMENT "Staging tri3d.vert.msl"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/tri3d.frag.msl
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${IVP_SHADER_SRC_DIR}/tri3d.frag.msl
                ${IVP_SHADER_BIN_DIR}/tri3d.frag.msl
        DEPENDS ${IVP_SHADER_SRC_DIR}/tri3d.frag.msl
        COMMENT "Staging tri3d.frag.msl"
        VERBATIM
    )
    add_custom_command(
        OUTPUT ${IVP_SHADER_BIN_DIR}/tri3d_inst.vert.msl
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${IVP_SHADER_SRC_DIR}/tri3d_inst.vert.msl
                ${IVP_SHADER_BIN_DIR}/tri3d_inst.vert.msl
        DEPENDS ${IVP_SHADER_SRC_DIR}/tri3d_inst.vert.msl
        COMMENT "Staging tri3d_inst.vert.msl"
        VERBATIM
    )

    add_custom_target(ivp_render_shaders ALL
        DEPENDS
            ${IVP_SHADER_SPV_OUTPUTS}
            ${IVP_SHADER_BIN_DIR}/line3d.vert.msl
            ${IVP_SHADER_BIN_DIR}/line2d.vert.msl
            ${IVP_SHADER_BIN_DIR}/line.frag.msl
            ${IVP_SHADER_BIN_DIR}/tri3d.vert.msl
            ${IVP_SHADER_BIN_DIR}/tri3d.frag.msl
            ${IVP_SHADER_BIN_DIR}/tri3d_inst.vert.msl
    )

    set(IVP_RENDER_SOURCES
        src/render/ivp_render_camera.c
        src/render/ivp_render_gpu.c
        src/render/ivp_render_draw.c
        src/render/ivp_render_hud.c
        src/render/ivp_render_hud_atlas.c
        src/render/ivp_render_core.c
        src/render/ivp_render_pick.c
        src/render/ivp_render_stats.c
        src/render/ivp_render_frustum.c
        src/render/ivp_render_solid.c
    )
    add_library(ivp_render STATIC ${IVP_RENDER_SOURCES})
    add_dependencies(ivp_render ivp_render_shaders)
    target_include_directories(ivp_render
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/render
    )
    target_link_libraries(ivp_render PUBLIC SDL3::SDL3)
    target_compile_definitions(ivp_render PRIVATE
        IVP_RENDER_SHADER_DIR_FALLBACK="${IVP_SHADER_BIN_DIR}"
    )

    # Apply floating-point strictness
    if (MSVC)
        target_compile_options(ivp_render PRIVATE /fp:strict /Oi-)
        if (CMAKE_C_COMPILER_ID MATCHES "Clang")
            target_compile_options(ivp_render PRIVATE -ffp-contract=off -fno-fast-math)
        endif ()
    else ()
        target_compile_options(ivp_render PRIVATE
                -ffp-contract=off -fno-fast-math -frounding-math
        )
    endif ()

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
    ivp_add_test(test_math tests/test_math.c)
    ivp_add_test(test_containers tests/test_containers.c)
    ivp_add_test(test_geometry tests/test_geometry.c)
    ivp_add_test(test_collision tests/test_collision.c)
    ivp_add_test(test_dynamics tests/test_dynamics.c)
    ivp_add_test(test_solver tests/test_solver.c)
    ivp_add_test(test_utility tests/test_utility.c)

    # New coverage-expansion tests
    ivp_add_test(test_controllers tests/test_controllers.c)
    ivp_add_test(test_builders tests/test_builders.c)
    ivp_add_test(test_anomaly tests/test_anomaly.c)
    ivp_add_test(test_object_lifecycle tests/test_object_lifecycle.c)
    ivp_add_test(test_containers_ext tests/test_containers_ext.c)
    ivp_add_test(test_math_ext tests/test_math_ext.c)
    ivp_add_test(test_collision_ext tests/test_collision_ext.c)
    ivp_add_test(test_events tests/test_events.c)

    # Gap-fill tests (legacy alignment)
    ivp_add_test(test_active_value_ext tests/test_active_value_ext.c)
    ivp_add_test(test_collision_filter tests/test_collision_filter.c)
    ivp_add_test(test_friction_system tests/test_friction_system.c)
    ivp_add_test(test_impact_system tests/test_impact_system.c)
    ivp_add_test(test_3d_solver tests/test_3d_solver.c)
    ivp_add_test(test_bvh_bottomup tests/test_bvh_bottomup.c)
    ivp_add_test(test_polygon_collision_debug tests/test_polygon_collision_debug.c)
    ivp_add_test(test_fake_jetski tests/test_fake_jetski.c)

    # Hierarchical collision and CCD tests
    ivp_add_test(test_hierarchical_collision tests/test_hierarchical_collision.c)
    ivp_add_test(test_ccd_integration tests/test_ccd_integration.c)
    ivp_add_test(test_mindist_event_solver tests/test_mindist_event_solver.c)
    ivp_add_test(test_time_event_queue tests/test_time_event_queue.c)
    ivp_add_test(test_synapse_links tests/test_synapse_links.c)
    ivp_add_test(test_recursive_mindist tests/test_recursive_mindist.c)
    ivp_add_test(test_contact_pipeline_parity tests/test_contact_pipeline_parity.c)

    # P1 controller tests (stabilizer, suspension, constraint API)
    ivp_add_test(test_p1_controllers tests/test_p1_controllers.c)

    # P3 content pipeline tests
    ivp_add_test(test_content_pipeline tests/test_content_pipeline.c)

    # P2 stability benchmarks
    ivp_add_test(test_stability_benchmarks tests/test_stability_benchmarks.c)

    # Performance regression test
    ivp_add_test(test_perf tests/test_perf.c)

    # Scenario parity runner (not a unit test — a JSONL producer)
    add_executable(test_scenarios tests/test_scenarios.c)
    target_link_libraries(test_scenarios PRIVATE ivp)
    target_include_directories(test_scenarios PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/tests
    )
    if (NOT MSVC)
        target_link_libraries(test_scenarios PRIVATE m)
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
    else ()
        message(STATUS "Skipping test_compare_jsonl_self (missing Python3 interpreter or JSONL inputs)")
    endif ()

    # ── CTest labels ──────────────────────────────────────────────────────
    # unit:       all unit tests
    # perf:       performance regression tests
    # validation: scenario gen + compare tests
    # smoke:      example smoke tests

    set(IVP_UNIT_TESTS
            test_base test_math test_containers test_geometry test_collision
            test_dynamics test_solver test_utility
            test_controllers test_builders test_anomaly test_object_lifecycle
            test_containers_ext test_math_ext test_collision_ext test_events
            test_active_value_ext test_collision_filter test_friction_system
            test_impact_system test_3d_solver test_bvh_bottomup
            test_fake_jetski
            test_hierarchical_collision test_ccd_integration
            test_mindist_event_solver test_time_event_queue test_synapse_links
            test_recursive_mindist test_contact_pipeline_parity
            test_p1_controllers
            test_content_pipeline
    )
    foreach (T ${IVP_UNIT_TESTS})
        set_tests_properties(${T} PROPERTIES LABELS "unit")
    endforeach ()
    set_tests_properties(test_perf PROPERTIES LABELS "perf")
    set_tests_properties(test_stability_benchmarks PROPERTIES LABELS "perf;stability")

    # ── Scenario parity tests (C17 vs reference JSONL) ──────────────────
    # Each scenario: run test_scenarios to produce JSONL, then compare
    # against golden reference via compare_jsonl.py.
    # Self-compare mode: run scenario → compare output against itself.
    # When reference JSONL golden files exist, compare against those.

    set(IVP_SCENARIO_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/scenario_output)
    file(MAKE_DIRECTORY ${IVP_SCENARIO_OUTPUT_DIR})

    set(IVP_VALIDATION_CONFIG ${IVP_REPO_ROOT}/tools/validation_config.json)

    # Per-scenario parameters
    # Original 3 scenarios: 240 steps, dt=1/60
    set(SCENARIO_STEPS_freefall 240)
    set(SCENARIO_DT_freefall "0.016666666666666666")
    set(SCENARIO_STEPS_two_balls 240)
    set(SCENARIO_DT_two_balls "0.016666666666666666")
    set(SCENARIO_STEPS_slope_friction 240)
    set(SCENARIO_DT_slope_friction "0.016666666666666666")
    # Remaining scenarios: 480 steps, dt=1/180
    set(SCENARIO_STEPS_springs 480)
    set(SCENARIO_DT_springs "0.005555555555555556")
    set(SCENARIO_STEPS_rope 480)
    set(SCENARIO_DT_rope "0.005555555555555556")
    set(SCENARIO_STEPS_motor 480)
    set(SCENARIO_DT_motor "0.005555555555555556")
    set(SCENARIO_STEPS_buoyancy 480)
    set(SCENARIO_DT_buoyancy "0.005555555555555556")
    set(SCENARIO_STEPS_cubes 480)
    set(SCENARIO_DT_cubes "0.005555555555555556")
    set(SCENARIO_STEPS_force_actuator 480)
    set(SCENARIO_DT_force_actuator "0.005555555555555556")
    set(SCENARIO_STEPS_forcefield 480)
    set(SCENARIO_DT_forcefield "0.005555555555555556")
    set(SCENARIO_STEPS_stiff_spring 480)
    set(SCENARIO_DT_stiff_spring "0.005555555555555556")
    set(SCENARIO_STEPS_check_distance 480)
    set(SCENARIO_DT_check_distance "0.005555555555555556")
    set(SCENARIO_STEPS_collision_filter 480)
    set(SCENARIO_DT_collision_filter "0.005555555555555556")
    set(SCENARIO_STEPS_motion_controller 480)
    set(SCENARIO_DT_motion_controller "0.005555555555555556")
    set(SCENARIO_STEPS_phantom 480)
    set(SCENARIO_DT_phantom "0.005555555555555556")
    set(SCENARIO_STEPS_vehicle 480)
    set(SCENARIO_DT_vehicle "0.005555555555555556")

    set(IVP_SCENARIO_BASELINE_DIR ${IVP_REPO_ROOT}/tools/baselines)
    file(MAKE_DIRECTORY ${IVP_SCENARIO_BASELINE_DIR})

    foreach (SCENARIO
            freefall two_balls slope_friction
            springs rope motor buoyancy
            cubes force_actuator forcefield stiff_spring
            check_distance collision_filter motion_controller phantom vehicle)
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

    # ── Smoke tests for graphical examples ────────────────────────────────
    # Only available when examples are built; requires Python3.
    if (IVP_BUILD_EXAMPLES AND Python3_Interpreter_FOUND)
        set(IVP_SMOKE_SCRIPT ${IVP_REPO_ROOT}/tools/smoke_test_examples.py)
        if (EXISTS ${IVP_SMOKE_SCRIPT})
            add_test(
                    NAME test_smoke_examples
                    COMMAND ${Python3_EXECUTABLE} ${IVP_SMOKE_SCRIPT}
                    --build-dir ${CMAKE_BINARY_DIR}
                    --timeout 5
            )
            set_tests_properties(test_smoke_examples PROPERTIES
                    LABELS "smoke"
                    TIMEOUT 60
            )
        endif ()
    endif ()
endif ()
