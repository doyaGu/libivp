# libivp examples

Graphical examples on the SDL3 GPU renderer in `src/render`, plus the
shared framework they sit on (`demo_app.h`). Every example also runs
without a window.

```bash
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DIVP_BUILD_EXAMPLES=ON
cmake --build build
./build/examples/example_shapes                  # window
./build/examples/example_shapes --headless       # self-check, no window or GPU
python3 tools/smoke_test_examples.py --build-dir build   # all of them (ctest -L smoke)
```

Building needs `glslangValidator` (the shaders) and fetches SDL3 unless it is
installed. The renderer prefers Metal shaders on macOS; the SPIR-V path is
built but has not been run.

## In the window

| Keys | |
|---|---|
| Enter, `.`, `[` `]`, `\` | pause, one step, halve / double the time scale, 1x |
| Space, Home | reset the scene, reset the camera |
| F1 | solid or wireframe |
| F2, F3, F4 | mass centre axes, velocity arrows, recent impacts |
| F5 | parameter panel: gravity, time scale, sun, ambient, shadows, then the example's own sliders (listed below); it flows into a second column when it gets tall |
| mouse | hover outlines a body, click selects it (Esc clears) and shows its details; hold the left button on a dynamic body to drag it |
| WASD QE RF, right drag, middle drag, wheel | pan, orbit, pan, zoom |
| Tab, C | focus the next body, follow it |

Each example lists its own keys at the bottom left.

IVP worlds are **Y down**: gravity is +Y and the camera's up vector is -Y, so
"above the ground" means a smaller Y. Draw bodies with `demo_draw_object`,
which handles solid / wire, shadows and the hover and selection outlines.

## Command line

| Option | |
|---|---|
| `--headless [N]`, `-H [N]` | N frames (default 600) at 1/60 s, no window; prints stats and `self-check PASSED` / `FAILED`, exit code 0 / 1. `IVP_EXAMPLE_HEADLESS=N` does the same |
| `--screenshot FILE [N]` | render N frames (default 60) at a fixed 1/60 s step, save the last as a PNG, exit |
| `--overlays` | start with F2-F4 and the F5 panel on and the first dynamic body selected |
| `--no-shadows` | start with the shadow map off |
| `--mouse X Y` | pretend the mouse is at window pixel (X, Y): hover outline and details in a screenshot |

## The examples

Physics and controllers:

| Example | Shows |
|---|---|
| `freefall` | balls of different mass and elasticity bouncing |
| `cubes` | stacking, a ramp, friction and elasticity |
| `domino` | a chain of toppling tiles |
| `bridge`, `rope`, `hinge`, `maxforce` | ball-socket and hinge constraints, a maximum impulse |
| `springs`, `stiff_spring`, `snowflake` | spring lattices, explicit against implicit springs |
| `motor`, `motion_controller`, `push`, `onthefly` | a rotation motor, driving a body directly, speed and impulse changes, changing mass and friction at run time |
| `force_actuator`, `forcefield`, `black_hole` | a magnet, a force field with modes, an own radial controller |
| `check_distance`, `phantom`, `callback`, `active_value` | distance triggers, trigger volumes, object and collision listeners, active value chains |
| `collision_filter`, `raycasting`, `universe` | collision filters, the ray solver, streaming static objects |
| `buoyancy` | bodies of different density in water with a current |

Geometry:

| Example | Shows |
|---|---|
| `shapes` | convex hulls (cylinders, cones, prisms, pyramids ...) built with the qhull based builder |
| `cut`, `ring`, `3ds_import` | chopping a surface, a concave ring from a ledge soup, loading a `.3ds` file |
| `terrain` | a heightfield as a compact grid with bodies rolling into the valleys |

Larger scenes:

| Example | Shows |
|---|---|
| `vehicle`, `offroad` | the four-wheel raycast car on a flat ground with ramps, and over 120 m of hills |
| `boat` | a hull floating through the buoyancy controller, driven by an own controller |
| `platform` | an elevator, a slider and a turntable driven by golem controllers, carrying crates |
| `compound` | rigid bodies made of several objects with `ivp_object_attach_object`; B breaks them apart |

Examples with their own F5 sliders: `offroad` and `vehicle` (engine, brake, suspension,
tyres, anti-roll bar), `boat` (thrust, rudder, keel, water drag), `buoyancy` (water density,
current, drag), `springs` (bridge and bungee stiffness, damping), `forcefield` (field
strength), `force_actuator` (magnet force, push speed, automatic switching) and `black_hole`
(pull strength). A change wakes the sleeping bodies, since a sleeping core runs no controllers.

## Writing an example

```c
#include "demo_app.h"

static void my_draw(demo_app_t *app, demo_viewer_t *v) { /* demo_draw_object(...), demo_hud(...) */ }
static bool my_check(demo_app_t *app) { return demo_check(app->cfg.tag, true, "it works"); }

int main(int argc, char **argv) {
    demo_app_t app = {0};
    demo_config_defaults(&app.cfg);
    app.cfg.tag = "mine";
    if (!demo_world_init(&app.world, &app.cfg)) return 1;
    /* demo_create_box / _ball / _polygon ... */
    app.draw = my_draw;
    app.check = my_check;      /* runs at the end of --headless */
    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    return rc;
}
```

Add it to `examples/CMakeLists.txt` with `ivp_add_example(example_mine example_mine.c)`.
`update` runs every frame with `viewer == NULL` when headless, so give it a
script instead of key presses. Extra sliders and checkboxes for the F5 panel:
`demo_param_float` / `demo_param_bool` from `viewer_init`, or `demo_panel_add_float`
with a callback that pushes the value to the engine.

The framework headers: `demo_app.h` (world, viewer, drawing), `demo_debug.h`
(picking, impact ring), `demo_panel.h` (the panel widgets, testable without a
window), `demo_terrain.h` (heightfield terrain).
