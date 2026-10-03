# Third-party notices

libivp contains or uses the following third-party code.  Their copyright
notices are kept in the source files listed here.

## qhull 2.6

- Location: `deps/qhull/`
- Copyright (c) 1993-1999 C.B. Barber and The Geometry Center, University
  of Minnesota.
- License: the qhull license, see `deps/qhull/COPYING.txt`.
- Modified for libivp (converted to C17, small build fixes, output through
  a message sink).  The modification notice is in `deps/qhull/qhull_compat.h`.
  The original qhull source code may be obtained from http://www.qhull.org.

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
