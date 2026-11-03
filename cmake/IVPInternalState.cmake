# Migrate existing caches without silently changing their internal-state policy.
if (DEFINED IVP_REFERENCE_INTERNAL_STATE)
    if (NOT DEFINED IVP_STABLE_INTERNAL_STATE)
        if (IVP_REFERENCE_INTERNAL_STATE)
            set(IVP_STABLE_INTERNAL_STATE OFF CACHE BOOL "Use history-independent internal state")
        else ()
            set(IVP_STABLE_INTERNAL_STATE ON CACHE BOOL "Use history-independent internal state")
        endif ()
    endif ()
    message(DEPRECATION
            "IVP_REFERENCE_INTERNAL_STATE was replaced by the inverse option IVP_STABLE_INTERNAL_STATE; the new option takes precedence when both are set")
    unset(IVP_REFERENCE_INTERNAL_STATE CACHE)
    unset(IVP_REFERENCE_INTERNAL_STATE)
endif ()

option(IVP_STABLE_INTERNAL_STATE
       "Use history-independent internal state instead of reference internals"
       OFF)
option(IVP_EDIT_LOCALITY
       "Pace movement checks per simulation unit so scene edits stay local (changes trajectories versus the reference)"
       OFF)

if (NOT IVP_STABLE_INTERNAL_STATE AND IVP_EDIT_LOCALITY)
    message(FATAL_ERROR "IVP_EDIT_LOCALITY requires -DIVP_STABLE_INTERNAL_STATE=ON")
endif ()
