# libivp

libivp is a C17 port of the IVP (Ipion Virtual Physics) engine, written in
C++ by Ipion Software GmbH in 1999–2000. It provides rigid bodies,
continuous collision detection, impulse-based contact and constraint
solving, and a controller and actuator system with springs, motors,
buoyancy, force fields and vehicles.

The 49 parity scenarios exercise two arithmetic modes:

- With `IVP_REFERENCE_TARGET=NATIVE`, x87 emulation off and
  `IVP_STABLE_INTERNAL_STATE=OFF`, libivp
  is compared bit for bit with the original C++ IVP using native arithmetic
  on the same platform. The macOS arm64 CI runs this check in Release and Debug.
- With software x87 arithmetic and transcendentals enabled, compare libivp
  trajectories across platforms using the same internal-state policy.

`IVP_STABLE_INTERNAL_STATE=ON` removes dependencies on incidental container
order and allocator history. In the native arithmetic comparison, its expected
trajectory differences from the reference are listed in `tools/parity/`.

## Building

You need CMake 3.20 or later and a C17 compiler. The x87 backend also
uses a C++11 compiler. The default build uses the
reference engine's bundled Qhull, kept in `src/builder/qhull_ipion`; with
`IVP_REENTRANT_QHULL=ON` CMake fetches the pinned Qhull 2020.2 instead. The
examples also fetch SDL3.

```bash
git submodule update --init --recursive
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

To use libivp from another CMake project, add it as a subdirectory and link
the `ivp` target. Enable CXX in the parent project when either x87 backend
is enabled:

```cmake
project(my_app LANGUAGES C CXX)
set(IVP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(libivp)
target_link_libraries(my_app PRIVATE ivp)
```

### Options

| Option | Default | Description |
|---|---|---|
| `IVP_REAL_DOUBLE` | ON | `double` for `ivp_real_t`, like the reference `IVP_DOUBLE` (OFF: `float`) |
| `IVP_ENABLE_X87_EMULATION` | ON | Use `SOFT` arithmetic when `IVP_X87_ARITHMETIC` is empty |
| `IVP_X87_ARITHMETIC` | Follows `IVP_ENABLE_X87_EMULATION` | `OFF`, `SOFT` or `HARD` for the restored arithmetic sequences |
| `IVP_X87_ARITHMETIC_TRANSCENDENTAL` | Follows `IVP_X87_ARITHMETIC` | `OFF`, `SOFT` or `HARD` for the original IVP math wrappers |
| `IVP_REFERENCE_TARGET` | RETAIL | Reference build. RETAIL uses scalar solver rows and the Ballance hull merge order. NATIVE follows the host pointer width for solver rows and the native Qhull merge order. |
| `IVP_STABLE_INTERNAL_STATE` | OFF | Use history-independent internals; OFF preserves the reference's address- and history-sensitive behavior |
| `IVP_EDIT_LOCALITY` | OFF | Adding or removing objects changes only the objects they interact with; requires `IVP_STABLE_INTERNAL_STATE=ON` and changes trajectories |
| `IVP_ENABLE_CHECKPOINTS` | OFF | Memory regions with in-place checkpoints, and rewinding to any recorded step |
| `IVP_REENTRANT_QHULL` | OFF | Like the reference: OFF uses the reference engine's bundled Qhull (process-global, hull builds serialized); ON fetches Qhull 2020.2 `libqhull_r`, whose hull builds can run concurrently |
| `IVP_ENABLE_ASSERTS` | OFF | Assertions and the reference debug code in Release (always on in Debug) |
| `IVP_CONTAINER_DEBUG` | OFF | Iterator, bounds and canary checks on containers |
| `IVP_ENABLE_STATS` | OFF | Runtime statistics counters |
| `IVP_BUILD_TESTS` | ON | Unit tests |
| `IVP_ENABLE_SCENARIO_TESTS` | OFF | Scenario JSONL producer and invariant tests |
| `IVP_ENABLE_PARITY_TESTS` | OFF | Compare scenario output with the reference JSONL |
| `IVP_BUILD_EXAMPLES` | OFF | Graphical examples (enables the render module) |
| `IVP_BUILD_RENDER` | OFF | Render module: SDL3 GPU natively, WebGL2 under Emscripten |
| `IVP_ENABLE_ASAN` / `IVP_ENABLE_UBSAN` | OFF | Sanitizers |

`SOFT` uses libx87, the arithmetic backend shared with the C++ IVP engine. Its
intermediates use a 64-bit significand and follow the active C rounding
mode. Scalar `double` results are stored at binary64 precision before
returning to C on every target. `HARD` requires a 32-bit x86 target and uses
the host x87 when its precision and rounding allow it. Hardware
transcendental results follow the executing CPU.
The arithmetic backend covers inverse square root, vector normalization,
double-precision vector length, quaternion multiplication and small-angle
updates. The reference target fixes solver row windows, the bundled Qhull
merge order and contact energy decay write-backs for either arithmetic backend.
The 64-bit native reference uses four-coefficient VecFPU windows and padded
row storage. The retail reference uses scalar rows on every host.
The transcendental setting controls sine, cosine, arcsine, arccosine,
arctangent, and exponential calls through the IVP math wrappers.

For bitwise trajectory comparisons across targets, use `SOFT` for both
selections with the same reference target. `IVP_STABLE_INTERNAL_STATE`
controls allocation- and history-sensitive behavior inside the solver and
whether the hull builder derives its Qhull roundoff tolerances from the input.

Set both selections to `OFF` for a pure C17 build without libx87. Compare
this mode with the original C++ IVP on the same platform using
`IVP_REFERENCE_TARGET=NATIVE`, both selections set to `OFF`,
floating-point contraction disabled, and `IVP_STABLE_INTERNAL_STATE=OFF`
in libivp. This keeps the reference
engine's original hull tolerances. On WebAssembly, compile both scenario
runners with `-fno-builtin-sin -fno-builtin-cos` to use the same runtime
libm when initializing scenes.

Existing CMake caches using `IVP_REFERENCE_INTERNAL_STATE` are migrated to the
inverse `IVP_STABLE_INTERNAL_STATE` value. When both are set, the new option
takes precedence. Pass
`-DIVP_STABLE_INTERNAL_STATE=OFF` or configure a fresh directory to select the
new default in a previously configured build.

Library code is compiled with `-ffp-contract=off -fno-fast-math
-frounding-math` (`/fp:strict` on MSVC). Don't build it with fast-math or
FMA contraction: results would no longer match the reference.

### WebAssembly

With Emscripten the library, the tests and the examples build for
WebAssembly; the tests and the examples' headless checks run under node, and
the example pages are checked in headless Chrome when one is installed.

```bash
emcmake cmake -B build-wasm -S . -DCMAKE_BUILD_TYPE=Release -DIVP_BUILD_EXAMPLES=ON
cmake --build build-wasm
ctest --test-dir build-wasm -L unit -LE threading      # under node
ctest --test-dir build-wasm -L browser                 # the pages in headless Chrome
python3 -m http.server -d build-wasm/examples 8000      # http://localhost:8000/
```

The wasm build is single-threaded: an environment with
`parallel_worker_count` above 1, or a world scheduler with more than one
worker, fails to create. CMake refuses `IVP_ENABLE_CHECKPOINTS` there
(Emscripten's `mmap` commits the whole reserved region at once).
WebAssembly has only round-to-nearest, so `-frounding-math` is left out, and
results are not compared with the reference: libm differs, and some layouts
differ on a 32-bit target.

## Example

A ball dropped onto a static box. IVP worlds are Y down: gravity points
along +Y, so "above the ground" means a smaller Y.

```c
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <stdio.h>

int main(void)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);

    /* A static 20 x 1 x 20 box as the ground, its top face at y = 0. */
    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 10.0f, 0.5f, 10.0f);
    ivp_surman_polygon_t ground_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(1e6);
    t.physical_unmoveable = true;
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.5), IVP_REAL(0.0));
    ivp_environment_create_object(env, &t, &ground_sm.base);

    /* A 1 kg ball of radius 0.5, its center 5 m above the ground. */
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    t.ball_radius = IVP_REAL(0.5);
    t.material.elasticity = IVP_REAL(0.6);
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(-5.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &t, NULL);

    for (int frame = 0; frame <= 180; frame++) {
        if (frame % 30 == 0) {
            ivp_vec3r_t p;
            ivp_object_get_position(ball, &p);
            printf("t=%.1fs  height=%.3f\n", frame / 60.0, -(double)p.k[1] - 0.5);
        }
        ivp_environment_simulate_dtime(env, IVP_REAL(1.0) / 60);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}
```

The ball bounces a few times and comes to rest on the box. Surfaces and
surface managers are borrowed by the objects that use them, so free them
after the environment.

`ivp_object_get_position()` and `ivp_object_get_orientation()` read the core's
pose stored at the last PSI. They do not sample the current environment time
or include the core-to-object offset. For rendering an object's current world
pose, use `ivp_object_get_m_world_f_object_AT()`; for a current mass-centre
position, use `ivp_core_get_at_position()` with the environment time. Physics
controllers should retain the sampling time and coordinate frame required by
their reference algorithm.

### Rewinding and rollout searches

Build with `IVP_ENABLE_CHECKPOINTS=ON` to use `ivp_rewind_step()` and
`ivp_rewind_seek()`. The environment and all mutable simulation/controller
state must use one memory region. A recording restores that region in place,
in the same process; it is not a portable save file or a clone into another
world. Replay callbacks must depend only on their recorded inputs, PSI and
region state.

A step after seeking backwards discards the recorded future. Keep the search
tree, candidate action sequences and scores outside the region, and size the
keyframe ring so the search root remains reachable. Each parallel rollout
needs an independent world, region and rewinder. The world scheduler advances
environments directly and bypasses rewind recording; an application executor
must call `ivp_rewind_step()` on each recorded slot instead, with one owner
per slot and matching floating-point environments on its workers.

The default `IVP_STABLE_INTERNAL_STATE=OFF` preserves the reference's address
and allocation-history sensitivity. Scheduling one world on a different
worker does not change its result, but independently reconstructed worlds
need not match. Select `IVP_STABLE_INTERNAL_STATE=ON` when equivalent search
slots must agree despite those histories; it deliberately differs from the
reference in the 24 registered native arithmetic scenarios. Use reference
internal state and disable x87 emulation for native C++ IVP parity.

### Graphical examples

`examples/` has 51 examples on an SDL3 GPU renderer, from falling balls and
stacks to constraints, buoyancy, ray casting and drivable vehicles, plus
`example_gallery`, a launcher with a card for each one.

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DIVP_BUILD_EXAMPLES=ON
cmake --build build
./build/examples/example_gallery
./build/examples/example_shapes --headless    # self-check without a window
```

Building them needs `glslangValidator`. `examples/README.md` lists the
examples and their controls. They also run in a browser (WebAssembly and
WebGL2; see WebAssembly above).

## Layout

Each module has a source directory `src/<module>/` and a public header
directory `include/ivp/<module>/`.

| Module | Contents |
|---|---|
| `base` | Allocator, memory regions and checkpoints, arena, asserts, messages, scalar types, time |
| `math` | Vectors, matrices, quaternions, transforms, geometry helpers |
| `containers` | Arrays, pointer vectors, sets, hash maps, min-heap, min-list |
| `geometry` | Compact surfaces, ledges and edges, ledge trees, compact grids, surface managers |
| `builder` | Convex hulls (Qhull), ledge and halfspace soups, BVH, concave decomposition (GEOMPACK), 3DS import |
| `collision` | OV tree, hull managers, mindists and their solvers, time events, ray and sphere solvers |
| `dynamics` | Cores, objects, environment and PSI loop, simulation units, clusters, phantoms, rewinding, parallel worlds |
| `controllers` | Actuators, constraints, springs, buoyancy, force fields, motion, vehicles, attacher, controller factory |
| `solver` | Impact solver, contact points and friction, LCS solver |
| `events` | Object, collision and PSI listeners |
| `utility` | Active values, strings, interpolation |
| `diagnostics` | Performance counter, statistics, debug managers |
| `render` | Optional SDL3 GPU renderer used by the examples |

## Parity with the reference engine

The parity workflow builds the C++ reference engine and its scenario runner
and compares their output with libivp's. It needs the reference checked
out next to this repository, a C++17 compiler and Python 3:

```bash
git clone https://github.com/doyaGu/ivp.git ../ivp
tools/regen_reference.sh     # build the reference, write tools/testdata/ref_*.jsonl
tools/parity/check.sh        # reference-development 49/49 + mainline difference gate
```

`tools/parity/README.md` describes the checks and how to chase a
difference down to the first differing value.

## License

libivp is released under the [MIT License](LICENSE). Qhull, GEOMPACK and
SDL3 keep their own licenses; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
