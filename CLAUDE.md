# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

`libivp` is a C17 reimplementation of the IVP (Ipion Virtual Physics) engine, originally written in C++ by Ipion Software GmbH (1999–2000). It is a rigid-body physics simulation library with continuous collision detection, impulse-based constraint solving, and a rich controller/actuator system.

## Build Commands

```bash
# Configure (from repo root)
cmake -B build -S .

# Build
cmake --build build

# Run all tests
ctest --test-dir build

# Run a specific test
./build/test_dynamics

# Run tests by label
ctest --test-dir build -L unit
ctest --test-dir build -L perf
ctest --test-dir build -L validation

# Build with examples (requires SDL3 GPU)
cmake -B build -S . -DIVP_BUILD_EXAMPLES=ON
cmake --build build

# Useful debug/analysis flags
cmake -B build -S . -DIVP_ENABLE_ASAN=ON -DIVP_ENABLE_UBSAN=ON
cmake -B build -S . -DIVP_CONTAINER_DEBUG=ON
cmake -B build -S . -DIVP_REAL_DOUBLE=OFF  # use float instead of double for ivp_real_t
cmake -B build -S . -DIVP_ENABLE_PARITY_TESTS=ON  # compare against golden JSONL reference
```

## Architecture

### Library Modules (`src/` and `include/ivp/`)

The library is structured into independent modules, each with a matching `src/<module>/` and `include/ivp/<module>/` pair:

| Module | Role |
|---|---|
| `base` | Allocator, arena, `IVP_U_Memory` port (`ivp_u_memory`: the environment's short-term arena with the reference layout, never cleared, used by the friction solver and LCS), assert, scalar types (`ivp_real_t`), time |
| `math` | `ivp_vec3`, `ivp_mat3`, `ivp_quat`, `ivp_transform`, geometry helpers |
| `containers` | Array, hashmap, minheap, vhash, minlist |
| `geometry` | Compact surface/edge/ledge representations, ledgetree, surface manager |
| `builder` | Shape builders: box/pointsoup convex hull (qhull 2.6 + reference polygon/tetra/ledge generator), halfspace soup, BVH, concave decomposition, 3DS import, compact modify (chop/shrink) |
| `collision` | Reference collision pipeline: OV tree, hull managers, mindists (minimize/event solvers, recursive mindists, collision delegators), time events, ray/sphere solvers |
| `dynamics` | `ivp_core` (IVP_Core), `ivp_object`, `ivp_environment` (time manager, PSI loop, sim units, controller manager, listeners, universe manager, static object, phantoms), cluster hierarchy. Cores are heap objects: an object references `physical_core` / `friction_core` / `original_core` (`ivp_object_get_core` = physical core) and a core lists its objects; `merge_objects` (`ivp_merge_core.c`) and attach/detach (`ivp_object_attach.c`) give several objects one core. Teardown follows the reference cluster order, static object first |
| `controllers` | Actuators, constraints, stiff springs, buoyancy, forcefield, motion/golem, floating, car system (real wheels + constraint car), raycast car/airboat/jetski, attacher, controller factory (`IVP_Controller_Factory` create functions), phantom trigger utility |
| `solver` | Impact solver/system, contact points + friction system, LCS (`ivp_linear_constraint_solver`) and great matrix, solver core reaction |
| `events` | Event bus, object / collision / PSI listeners |
| `utility` | Active value, multidimensional interpolation |
| `diagnostics` | Performance counter (`IVP_PerformanceCounter`, wired at the reference PSI / time-event / mindist sites), better statistics manager + console callback, debug managers, clustering visualizer |
| `render` | Optional SDL3 GPU render module (only when `IVP_BUILD_RENDER=ON`) |

### Third-Party Dependencies

- **qhull 2.6** (`deps/qhull/`, Geometry Center 1999): the qhull version the reference compiles (`ivp_compact_builder/qhull*.cxx`), as C17 (global state, not reentrant: the convex builders are not thread safe). Compiled as `ivp_qhull` with strict FP; it reproduces the reference facet order, which the ledge layout depends on. Its error output goes to stderr and through `ivp_message`, as in the reference.
- **SDL3**: Fetched via CMake FetchContent when not system-installed. Only needed for examples/render.

### Simulation Loop (PSI)

libivp ports the reference event-driven pipeline. The time manager fires the PSI event every `delta_psi_time` and mindist time events in between (collisions are solved at their exact time, `ivp_impact_core.c`). Each PSI (`env_simulate_psi` in `src/dynamics/ivp_environment.c`):
1. revive queued cores, universe manager check, active values refresh, PSI listeners
2. controllers of every sim unit (`ivp_sim_unit.c`, descending priority) and integration (`ivp_core.c`, `ivp_cnps_*`)
3. hull managers commit (OV tree / new mindists), exact mindists recalculated, mindist events scheduled for the next interval
Friction (`src/solver/ivp_fr_system.c`) runs as controllers of the sim units.  See `docs/PARITY_GAP_INVENTORY.md` for the remaining differences to the reference and `docs/PORT_WORK_REPORT.md` for the work log, lessons and open items.

### Controller Priority Ordering

Controllers are dispatched within a simulation unit in descending priority order (highest first). The dividing line at `IVP_CP_GRAVITY = 1000` separates synchronous pushes (priority < 1000) from asynchronous pushes (priority > 1000). Key values: `IVP_CP_STATIC_FRICTION=0`, `IVP_CP_CONSTRAINTS=405`, `IVP_CP_MOTION=500`, `IVP_CP_GRAVITY=1000`, `IVP_CP_SPRING=1400`, `IVP_CP_ENERGY_FRICTION=2000`.

### Test Framework

Tests use a minimal custom framework in `tests/ivp_test.h`. Pattern:
```c
IVP_TEST(my_test) {
    IVP_ASSERT_TRUE(expr);
    IVP_ASSERT_NEAR(a, b, eps);
    IVP_ASSERT_EQ(a, b);
}
// At bottom of file:
int main(void) {
    IVP_REGISTER(my_test);
    IVP_RUN_ALL();
}
```

### Scenario / Validation Tests

`tests/test_scenarios.c` is a JSONL producer (not a unit test, built with `-DIVP_ENABLE_SCENARIO_TESTS=ON` and strict FP like the reference runner). It runs named physics scenarios and writes state snapshots. The scenario list (name, dt, steps) is `tools/scenarios.txt`, read by CMake and both regen scripts; a new scenario needs a `run_*` function in `tests/test_scenarios.c`, the same scene in the reference runner (`ivp/tools/ivp_reference_runner`), and a line in that list. `tools/testdata/ref_<scenario>.jsonl` are produced by the legacy C++ engine built with `-ffp-contract=off` (`tools/regen_reference.sh [path/to/ivp]` builds it and the runner and regenerates them; `IVP_REF_FMA=1` builds the FMA flavour of the shipped arm64 libs instead; never regenerate them from libivp). libivp reproduces the strict reference bit for bit in every listed scenario (Release, double, arm64 macOS; other platforms' libm may differ), so the drift baselines written by `tools/regen_baselines.sh <build>` are zero and any deviation fails `-DIVP_ENABLE_PARITY_TESTS=ON` (comparisons via `tools/compare_jsonl.py`). Debug builds (Debug or `IVP_ENABLE_ASSERTS`) run the reference debug code (`IVP_IF`, `IVP_DEBUG_BUILD`, `IVP_CORE` in `ivp_assert.h`; output via `ivp_message.h`) and reproduce a reference debug build (`-ffp-contract=off -DDEBUG`, no `NDEBUG`, runner run with SIGINT ignored) bit for bit, including its debug text; run `test_scenarios --assert-continue` for that comparison (reference asserts that fail on valid data are `IVP_ASSERT_NONFATAL`, reported only to a custom handler). `-DIVP_MINDIST_BEHAVIOUR_DEBUG` enables the mindist `DEBUG_CHECK_LEN` checks. `tools/parity/check.sh` runs the whole comparison (builds both reference flavours and libivp under `build/parity`, `--release`, `--debug`, `--sweep "dt:steps ..."`; see `tools/parity/README.md`). When chasing a difference, instrument both sides: copy a reference `.cxx`, add `fprintf(stderr, "%a")` probes and link it with `tools/parity/build_override_runner.sh` (compiled with the reference library flags so it overrides the library object).

Examples built on `examples/demo_app.h` support a headless self-check: `example_x --headless [frames]` (or `IVP_EXAMPLE_HEADLESS=N`).

### Key CMake Options

| Option | Default | Description |
|---|---|---|
| `IVP_REAL_DOUBLE` | ON | Use `double` for `ivp_real_t`, like the reference `IVP_DOUBLE` (OFF: `float`) |
| `IVP_ENABLE_ASSERTS` | OFF | Assertions in Release builds (always ON in Debug) |
| `IVP_CONTAINER_DEBUG` | OFF | Iterator/bounds/canary checks on containers |
| `IVP_BUILD_TESTS` | ON | Build and register unit tests with CTest |
| `IVP_BUILD_EXAMPLES` | OFF | Build graphical examples (auto-enables render module) |
| `IVP_ENABLE_ASAN` | OFF | AddressSanitizer |
| `IVP_ENABLE_UBSAN` | OFF | UndefinedBehaviorSanitizer |
| `IVP_ENABLE_PARITY_TESTS` | OFF | Compare scenario output against golden JSONL |
| `IVP_HOME_BUILD` | OFF | Clustering visualizer engine hooks (reference `IVP_HOME_BUILD`) |
| `IVP_MD_INTERPOLATION` | OFF | Buoyancy multidimensional interpolation (the reference compiles it out via `IVP_NO_MD_INTERPOLATION`) |

### Floating-Point Discipline

All `ivp`, `ivp_qhull` and `test_scenarios` targets compile with strict FP flags (`-ffp-contract=off -fno-fast-math -frounding-math` / `/fp:strict /Oi-` on MSVC). The render module and examples relax these flags. Do not use fast-math or FMA contractions in library code.

## Reference Documentation

`docs/` contains detailed references compiled from the original C++ source:
- `IVP_CONTACT_SYSTEM_REFERENCE.md`: Mindist, synapse, contact/impact/friction pipeline
- `IVP_CONTROLLER_SYSTEM_REFERENCE.md`: Controller hierarchy, priorities, wake semantics
- `IVP_LEGACY_FRICTION_REFERENCE.md`: Friction spring model, Coulomb disk, LCP solve
