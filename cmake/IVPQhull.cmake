# Qhull backends (the reference's bundled Qhull or external Qhull 8.0.2)
# and libivp's private adapter seam.
include_guard(GLOBAL)

include(FetchContent)

function(ivp_add_qhull_backend)
    if (TARGET ivp_qhull)
        message(FATAL_ERROR "ivp_add_qhull_backend() may only be called once")
    endif ()

    if (POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif ()

    set(_ivp_qhull_builder_dir
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../src/builder")
    if (IVP_REENTRANT_QHULL)
        FetchContent_Declare(
                ivp_qhull_modern
                URL https://github.com/qhull/qhull/archive/refs/tags/v8.0.2.tar.gz
                URL_HASH SHA256=8774e9a12c70b0180b95d6b0b563c5aa4bea8d5960c15e18ae3b6d2521d64f8b
                SOURCE_SUBDIR src/libqhull_r
        )
        FetchContent_MakeAvailable(ivp_qhull_modern)
        set(_ivp_qhull_source_dir "${ivp_qhull_modern_SOURCE_DIR}/src")
        set(_ivp_qhull_vendor_sources
                ${_ivp_qhull_source_dir}/libqhull_r/global_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/stat_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/geom2_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/poly2_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/merge_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/libqhull_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/geom_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/poly_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/qset_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/mem_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/random_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/usermem_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/userprintf_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/io_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/user_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/rboxlib_r.c
                ${_ivp_qhull_source_dir}/libqhull_r/userprintf_rbox_r.c
        )
        if (NOT EXISTS "${_ivp_qhull_source_dir}/libqhull_r/libqhull_r.h")
            message(FATAL_ERROR
                    "The Qhull 8.0.2 dependency has an unexpected source layout: ${_ivp_qhull_source_dir}")
        endif ()
        set(_ivp_qhull_adapter_source
                "${_ivp_qhull_builder_dir}/ivp_qhull_api_reentrant.c")
        set(_ivp_qhull_description "Qhull 2020.2 libqhull_r")
        set(_ivp_qhull_parallel_builds ON)
    else ()
        # The reference engine's own Qhull (ivp_compact_builder/qhull*.cxx,
        # "ipion 99/07/10"), ported to C in src/builder/qhull_ipion.
        set(_ivp_qhull_source_dir "${_ivp_qhull_builder_dir}/qhull_ipion")
        set(_ivp_qhull_vendor_sources
                ${_ivp_qhull_source_dir}/qhull.c
                ${_ivp_qhull_source_dir}/qhull_geom.c
                ${_ivp_qhull_source_dir}/qhull_geom2.c
                ${_ivp_qhull_source_dir}/qhull_global.c
                ${_ivp_qhull_source_dir}/qhull_io.c
                ${_ivp_qhull_source_dir}/qhull_mem.c
                ${_ivp_qhull_source_dir}/qhull_merge.c
                ${_ivp_qhull_source_dir}/qhull_poly.c
                ${_ivp_qhull_source_dir}/qhull_poly2.c
                ${_ivp_qhull_source_dir}/qhull_qset.c
                ${_ivp_qhull_source_dir}/qhull_stat.c
                ${_ivp_qhull_source_dir}/qhull_user.c
        )
        set(_ivp_qhull_adapter_source
                "${_ivp_qhull_builder_dir}/ivp_qhull_api_ipion.c")
        set(_ivp_qhull_description "the reference engine's bundled Qhull (serialized)")
        set(_ivp_qhull_parallel_builds OFF)
    endif ()

    # Keep upstream diagnostics separate from libivp-owned code.  This target
    # is the only place that knows Qhull's source layout and warning profile.
    add_library(ivp_qhull_vendor STATIC ${_ivp_qhull_vendor_sources})
    target_include_directories(ivp_qhull_vendor PRIVATE
            "${_ivp_qhull_source_dir}"
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../include")
    if (NOT MSVC)
        target_link_libraries(ivp_qhull_vendor PUBLIC m)
    endif ()
    if (NOT IVP_REENTRANT_QHULL)
        # The bundled Qhull reports through ivp_message: a static cycle
        # (ivp -> ivp_qhull -> ivp_qhull_vendor -> ivp) CMake repeats on
        # the link line for single-pass linkers.
        target_link_libraries(ivp_qhull_vendor PRIVATE ivp)
    endif ()
    ivp_apply_strict_fp(ivp_qhull_vendor)
    if (MSVC)
        target_compile_options(ivp_qhull_vendor PRIVATE /W0)
    else ()
        target_compile_options(ivp_qhull_vendor PRIVATE
                -Wall -Wextra -Wpedantic
                -Wconversion -Wshadow -Wdouble-promotion
                -Wno-unused-parameter
                -Wno-clobbered
                -Wno-implicit-fallthrough
                -Wno-maybe-uninitialized
                -Wno-conversion
                -Wno-sign-conversion
                -Wno-sign-compare
                -Wno-shadow
                -Wno-double-promotion
                -Wno-format
                -Wno-uninitialized
                -Wno-unused-but-set-variable
                -Wno-unused-but-set-parameter
                -Wno-format-pedantic
                -Wno-deprecated-declarations
                -Wno-unknown-warning-option
        )
    endif ()

    # The adapter is first-party code and keeps the project's strict warnings.
    add_library(ivp_qhull STATIC "${_ivp_qhull_adapter_source}")
    target_include_directories(ivp_qhull PRIVATE
            "${_ivp_qhull_builder_dir}"
            "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../include")
    target_include_directories(ivp_qhull SYSTEM PRIVATE
            "${_ivp_qhull_source_dir}")
    target_link_libraries(ivp_qhull PUBLIC ivp_qhull_vendor)
    if (NOT _ivp_qhull_parallel_builds)
        target_link_libraries(ivp_qhull PUBLIC Threads::Threads)
    endif ()
    ivp_apply_strict_fp(ivp_qhull)
    ivp_apply_project_warnings(ivp_qhull)

    set(IVP_QHULL_SUPPORTS_PARALLEL_BUILDS
            ${_ivp_qhull_parallel_builds} PARENT_SCOPE)
    message(STATUS "IVP Qhull backend: ${_ivp_qhull_description}")
endfunction()
