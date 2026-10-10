# External Qhull dependency and libivp's private adapter seam.
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
    string(TOUPPER "${IVP_QHULL_MODE}" _ivp_qhull_mode)

    if (_ivp_qhull_mode STREQUAL "REENTRANT")
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
        set(_ivp_qhull_adapter_source
                "${_ivp_qhull_builder_dir}/ivp_qhull_api_reentrant.c")
        set(_ivp_qhull_description "Qhull 2020.2 libqhull_r")
        set(_ivp_qhull_parallel_builds ON)
    elseif (_ivp_qhull_mode STREQUAL "REFERENCE")
        # Qhull does not publish the reference engine's 2.6 snapshot as a tag.
        # Its earliest immutable upstream tag is the classic global-state line.
        FetchContent_Declare(
                ivp_qhull_classic
                URL https://github.com/qhull/qhull/archive/refs/tags/v3.0.tar.gz
                URL_HASH SHA256=2389b9a1b824f626ea9e50ab536d89d2f44d0c7ed91b285d2bdbd0ae05e316c5
                SOURCE_SUBDIR src
        )
        FetchContent_MakeAvailable(ivp_qhull_classic)
        set(_ivp_qhull_source_dir "${ivp_qhull_classic_SOURCE_DIR}/src")
        set(_ivp_qhull_vendor_sources
                ${_ivp_qhull_source_dir}/user.c
                ${_ivp_qhull_source_dir}/global.c
                ${_ivp_qhull_source_dir}/stat.c
                ${_ivp_qhull_source_dir}/io.c
                ${_ivp_qhull_source_dir}/geom2.c
                ${_ivp_qhull_source_dir}/poly2.c
                ${_ivp_qhull_source_dir}/merge.c
                ${_ivp_qhull_source_dir}/qhull.c
                ${_ivp_qhull_source_dir}/geom.c
                ${_ivp_qhull_source_dir}/poly.c
                ${_ivp_qhull_source_dir}/qset.c
                ${_ivp_qhull_source_dir}/mem.c
        )
        set(_ivp_qhull_adapter_source
                "${_ivp_qhull_builder_dir}/ivp_qhull_api_classic.c")
        set(_ivp_qhull_description "classic Qhull 3.0 (serialized)")
        set(_ivp_qhull_parallel_builds OFF)
    else ()
        message(FATAL_ERROR
                "IVP_QHULL_MODE must be REENTRANT or REFERENCE, got: ${IVP_QHULL_MODE}")
    endif ()

    if (NOT EXISTS "${_ivp_qhull_source_dir}/qhull.h" AND
        NOT EXISTS "${_ivp_qhull_source_dir}/libqhull_r/libqhull_r.h")
        message(FATAL_ERROR
                "The selected Qhull dependency has an unexpected source layout: ${_ivp_qhull_source_dir}")
    endif ()

    # Keep upstream diagnostics separate from libivp-owned code.  This target
    # is the only place that knows Qhull's source layout and warning profile.
    add_library(ivp_qhull_vendor STATIC ${_ivp_qhull_vendor_sources})
    target_include_directories(ivp_qhull_vendor PRIVATE
            "${_ivp_qhull_source_dir}")
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
            "${_ivp_qhull_builder_dir}")
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
