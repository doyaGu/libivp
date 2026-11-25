option(IVP_ENABLE_X87_EMULATION
       "Reproduce x87 rounding and write-back rules when IVP_X87_ARITHMETIC is unset" ON)
set(IVP_X87_ARITHMETIC "${IVP_X87_ARITHMETIC}" CACHE STRING
    "Restored x87 arithmetic: OFF, SOFT or HARD. Empty follows IVP_ENABLE_X87_EMULATION")
set(IVP_X87_ARITHMETIC_TRANSCENDENTAL "${IVP_X87_ARITHMETIC_TRANSCENDENTAL}" CACHE STRING
    "IVP transcendentals: OFF, SOFT or HARD. Empty follows IVP_X87_ARITHMETIC")
set_property(CACHE IVP_X87_ARITHMETIC PROPERTY STRINGS "" OFF SOFT HARD)
set_property(CACHE IVP_X87_ARITHMETIC_TRANSCENDENTAL PROPERTY STRINGS "" OFF SOFT HARD)

set(_ivp_x87_arithmetic "${IVP_X87_ARITHMETIC}")
if (_ivp_x87_arithmetic STREQUAL "")
    if (IVP_ENABLE_X87_EMULATION)
        set(_ivp_x87_arithmetic SOFT)
    else ()
        set(_ivp_x87_arithmetic OFF)
    endif ()
endif ()
set(_ivp_x87_transcendental "${IVP_X87_ARITHMETIC_TRANSCENDENTAL}")
if (_ivp_x87_transcendental STREQUAL "")
    set(_ivp_x87_transcendental "${_ivp_x87_arithmetic}")
endif ()
string(TOUPPER "${_ivp_x87_arithmetic}" _ivp_x87_arithmetic)
string(TOUPPER "${_ivp_x87_transcendental}" _ivp_x87_transcendental)
set(_ivp_x87_modes OFF SOFT HARD)
list(FIND _ivp_x87_modes "${_ivp_x87_arithmetic}" IVP_X87_ARITHMETIC_VALUE)
list(FIND _ivp_x87_modes "${_ivp_x87_transcendental}" IVP_X87_ARITHMETIC_TRANSCENDENTAL_VALUE)
if (IVP_X87_ARITHMETIC_VALUE LESS 0)
    message(FATAL_ERROR "IVP_X87_ARITHMETIC=${IVP_X87_ARITHMETIC}: expected OFF, SOFT or HARD")
endif ()
if (IVP_X87_ARITHMETIC_TRANSCENDENTAL_VALUE LESS 0)
    message(FATAL_ERROR
            "IVP_X87_ARITHMETIC_TRANSCENDENTAL=${IVP_X87_ARITHMETIC_TRANSCENDENTAL}: expected OFF, SOFT or HARD")
endif ()
if (IVP_X87_ARITHMETIC_VALUE EQUAL 2 OR IVP_X87_ARITHMETIC_TRANSCENDENTAL_VALUE EQUAL 2)
    include(CheckCSourceCompiles)
    check_c_source_compiles("
#if !(defined(_M_IX86) || defined(__i386__))
#error not a 32-bit x86 target
#endif
int main(void) { return 0; }" IVP_X87_TARGET_IS_X86_32)
    if (NOT IVP_X87_TARGET_IS_X86_32)
        message(FATAL_ERROR
                "IVP_X87_ARITHMETIC=${_ivp_x87_arithmetic}, IVP_X87_ARITHMETIC_TRANSCENDENTAL=${_ivp_x87_transcendental}: HARD requires a 32-bit x86 target")
    endif ()
endif ()

set(IVP_ENABLE_X87_EMULATION OFF)
if (IVP_X87_ARITHMETIC_VALUE)
    set(IVP_ENABLE_X87_EMULATION ON)
endif ()
set(IVP_X87_LIBRARY OFF)
if (IVP_X87_ARITHMETIC_VALUE OR IVP_X87_ARITHMETIC_TRANSCENDENTAL_VALUE)
    set(IVP_X87_LIBRARY ON)
endif ()

function(ivp_configure_x87 target)
    set(ivp_x87_definitions
            IVP_ENABLE_X87_EMULATION=$<BOOL:${IVP_ENABLE_X87_EMULATION}>
            IVP_X87_ARITHMETIC=${IVP_X87_ARITHMETIC_VALUE}
            IVP_X87_ARITHMETIC_TRANSCENDENTAL=${IVP_X87_ARITHMETIC_TRANSCENDENTAL_VALUE})
    target_compile_definitions(${target} PUBLIC ${ivp_x87_definitions})
    if (NOT IVP_X87_LIBRARY)
        return()
    endif ()
    if (NOT TARGET libx87::x87)
        if (NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libx87/CMakeLists.txt")
            message(FATAL_ERROR "libx87 is required. Run git submodule update --init --recursive")
        endif ()
        add_subdirectory(third_party/libx87)
    endif ()
    # Keep the C++ dependency and its compiler options private to the backend.
    add_library(ivp_x87_backend STATIC src/math/ivp_x87.cxx)
    set_target_properties(ivp_x87_backend PROPERTIES POSITION_INDEPENDENT_CODE ON)
    target_include_directories(ivp_x87_backend PRIVATE include src)
    target_include_directories(ivp_x87_backend SYSTEM PRIVATE
            $<TARGET_PROPERTY:libx87::x87,INTERFACE_INCLUDE_DIRECTORIES>)
    target_compile_definitions(ivp_x87_backend PRIVATE ${ivp_x87_definitions})
    target_compile_features(ivp_x87_backend PRIVATE cxx_std_11)
    target_link_libraries(ivp_x87_backend PRIVATE libx87::x87)
    ivp_apply_project_warnings(ivp_x87_backend)
    target_link_libraries(${target} PRIVATE ivp_x87_backend)
    # The per-language IPO checks do not check whether the C++ linker
    # can read the C compiler's bitcode.
    if (CMAKE_C_COMPILER_ID STREQUAL CMAKE_CXX_COMPILER_ID
            AND CMAKE_C_COMPILER_VERSION VERSION_EQUAL CMAKE_CXX_COMPILER_VERSION
            AND NOT IVP_ENABLE_ASAN AND NOT IVP_ENABLE_UBSAN AND NOT IVP_ENABLE_TSAN)
        include(CheckIPOSupported)
        check_ipo_supported(RESULT ivp_x87_ipo_supported LANGUAGES C CXX)
        if (ivp_x87_ipo_supported)
            set_target_properties(${target} ivp_x87_backend
                    PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
        endif ()
    endif ()
endfunction()
