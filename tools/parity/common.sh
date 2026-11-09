# tools/parity/common.sh -- shared settings for the parity scripts (sourced).
#
#   IVP_REF_SRC     reference engine sources (default: ../ivp next to the repo)
#   IVP_PARITY_DIR  work directory for builds and outputs (default: build/parity)
#   IVP_JOBS        parallel jobs (default: 8)

PARITY_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IVP_REF_SRC="${IVP_REF_SRC:-$(cd "$PARITY_REPO/.." && pwd)/ivp}"
# Both engines default to their bundled Qhull (IVP_REENTRANT_QHULL=OFF);
# PARITY_REENTRANT_QHULL=ON (check.sh --reentrant-qhull) builds both with
# Qhull 8.0.2 libqhull_r, in a build tree of its own.
PARITY_REENTRANT_QHULL="${PARITY_REENTRANT_QHULL:-OFF}"
if [ "$PARITY_REENTRANT_QHULL" = ON ]; then
  IVP_PARITY_DIR="${IVP_PARITY_DIR:-$PARITY_REPO/build/parity_reentrant}"
else
  IVP_PARITY_DIR="${IVP_PARITY_DIR:-$PARITY_REPO/build/parity}"
fi
IVP_JOBS="${IVP_JOBS:-8}"
PARITY_SCENARIO_LIST="$PARITY_REPO/tools/scenarios.txt"

# reference CMake options: the reentrant Qhull 2020.2 backend and native
# arithmetic.  The reference defaults to its bundled original Qhull and to the
# software x87 emulation, which libivp does not port.
PARITY_REF_CMAKE_OPTIONS="-DIVP_REENTRANT_QHULL=$PARITY_REENTRANT_QHULL -DIVP_ENABLE_X87_EMULATION=OFF -DIVP_X87_ARITHMETIC=OFF -DIVP_X87_ARITHMETIC_TRANSCENDENTAL=OFF"
# the same selection for sources compiled against the reference headers
PARITY_REF_DEFS="-DIVP_ENABLE_X87_EMULATION=0 -DIVP_X87_ARITHMETIC=0 -DIVP_X87_ARITHMETIC_TRANSCENDENTAL=0"

# reference module directories (include paths of the runner and of overrides)
PARITY_REF_MODULES="ivp_compact_builder ivp_surface_manager ivp_collision ivp_physics ivp_utility ivp_controller ivp_intern"
# static libraries of the reference build (listed twice when linking)
PARITY_REF_LIBS="libivp_physics.a libivp_compact_builder.a libivp_collision.a libivp_surface_manager.a libivp_controller.a libivp_utility.a"

# Compile flags of the reference library objects (macOS / Unix build of the
# reference CMake project), used for instrumented override files.
PARITY_OVERRIDE_FLAGS_RELEASE="-DAPPLE -DIVP_VERSION_SDK -DUNIX $PARITY_REF_DEFS -ffp-contract=off -O3 -DNDEBUG -std=c++98 -w -fno-strict-aliasing"
PARITY_OVERRIDE_FLAGS_DEBUG="-DAPPLE -DIVP_VERSION_SDK -DUNIX $PARITY_REF_DEFS -ffp-contract=off -DDEBUG -O3 -std=c++98 -w -fno-strict-aliasing"

parity_die() { echo "parity: $*" >&2; exit 1; }

parity_check_ref_src() {
  [ -d "$IVP_REF_SRC/tools/ivp_reference_runner" ] ||
    parity_die "reference sources not found: $IVP_REF_SRC (set IVP_REF_SRC)"
}

# parity_ref_dir <flavor>: build directory of a reference flavor
parity_ref_dir() { echo "$IVP_PARITY_DIR/ref_$1"; }

# parity_ref_includes <libdir>
parity_ref_includes() {
  local inc="-I$1/include/ivp" d
  for d in $PARITY_REF_MODULES; do inc="$inc -I$IVP_REF_SRC/$d"; done
  echo "$inc"
}

# parity_ref_libs <libdir>
parity_ref_libs() {
  local libs="" l
  for l in $PARITY_REF_LIBS; do libs="$libs $1/$l"; done
  echo "$libs $libs"
}

# parity_scenarios: "name dt steps" lines of tools/scenarios.txt (filtered by
# the SCENARIOS environment variable, a space separated list, when set)
parity_scenarios() {
  grep -v '^#' "$PARITY_SCENARIO_LIST" | awk 'NF==3' | while read -r s dt n; do
    if [ -z "${SCENARIOS:-}" ] || [[ " $SCENARIOS " == *" $s "* ]]; then echo "$s $dt $n"; fi
  done
}
