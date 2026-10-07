# Third-party notices

libivp contains or uses the following third-party code.  Their copyright
notices are kept in the source files listed here.

## Qhull 2020.2 (8.0.2)

- Location: `deps/qhull/`
- Copyright (c) 1993-2020 C.B. Barber and The Geometry Center, University
  of Minnesota.
- License: the qhull license, see `deps/qhull/COPYING.txt`.
- The official reentrant `libqhull_r` C sources are vendored unmodified.
  `qhull_api.[ch]` is libivp's separate adapter.  Source provenance is in
  `deps/qhull/README.libivp.md`; the original source may be obtained from
  https://github.com/qhull/qhull.

## GEOMPACK

- Location: `src/builder/ivp_geompack.c` (convex decomposition, `cvdec3`
  and the routines it uses).
- GEOMPACK Version 3, written and copyright by Barry Joe, Dept. of Computing
  Science, University of Alberta.  The routines are a C port of an f2c
  translation of the Fortran 77 code; the author's notices are kept in the
  routine headers.

## SDL3

- Not vendored.  Fetched at configure time (`release-3.4.0`) only when the
  render module or the examples are built (`IVP_BUILD_RENDER` /
  `IVP_BUILD_EXAMPLES`).
- License: zlib, see https://github.com/libsdl-org/SDL.
