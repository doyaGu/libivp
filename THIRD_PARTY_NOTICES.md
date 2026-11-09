# Third-party notices

libivp contains or uses the following third-party code.  Copyright and
license notices are kept with the relevant source or reproduced here.

## Qhull

- Default build (`IVP_REENTRANT_QHULL=OFF`): `src/builder/qhull_ipion/`
  holds the Qhull the IVP reference engine bundles in
  `ivp_compact_builder/qhull*.cxx`, a Qhull 2.x snapshot copyright 1993-1999
  The Geometry Center (version label "ipion 99/07/10").  Its license is
  reproduced below as "Geometry Center Qhull license"; the original source
  may be obtained from http://www.qhull.org.
  Modifications, as required by condition 3 of that license:
  - Ipion Software GmbH, 1999: adapted Qhull for the IVP engine (C++
    sources, IVP message output and memory functions, ivp_qh_version).
  - The IVP reference engine (https://github.com/doyaGu/ivp), 2026: kept the
    arithmetic of the retail engine binaries (the order of the plane
    distance sum in `qh_distplane`, binary64 stores in `qh_backnormal` and
    `qh_sethyperplane_gauss`) and guarded a diagnostic against a null error
    stream.
  - libivp (doyaGu), 2026-11-09: compiled as C17 instead of C++ so a default
    libivp build matches the reference engine's default build; the reference
    engine's optional software x87 arithmetic was removed, IVP's message,
    memory and integer helpers are provided by `qhull_ivp.h`, a C++
    functional cast and a C++ overload declaration were rewritten in C, and
    two unused parameters were named.
- `IVP_REENTRANT_QHULL=ON`: not vendored.  CMake fetches the immutable
  official Qhull 2020.2 (8.0.2) archive from https://github.com/qhull/qhull,
  which contains its original `COPYING.txt`; that license is reproduced
  below as "Qhull 2020.2 license".

### Qhull 2020.2 license

                    Qhull, Copyright (c) 1993-2020

                            C.B. Barber
                           Arlington, MA

                               and

       The National Science and Technology Research Center for
        Computation and Visualization of Geometric Structures
                        (The Geometry Center)
                       University of Minnesota

                       email: qhull@qhull.org

This software includes Qhull from C.B. Barber and The Geometry Center.
Files derived from Qhull 1.0 are copyrighted by the Geometry Center.  The
remaining files are copyrighted by C.B. Barber.  Qhull is free software
and may be obtained via http from www.qhull.org.  It may be freely copied,
modified, and redistributed under the following conditions:

1. All copyright notices must remain intact in all files.

2. A copy of this text file must be distributed along with any copies
   of Qhull that you redistribute; this includes copies that you have
   modified, or copies of programs or other software products that
   include Qhull.

3. If you modify Qhull, you must include a notice giving the
   name of the person performing the modification, the date of
   modification, and the reason for such modification.

4. When distributing modified versions of Qhull, or other software
   products that include Qhull, you must provide notice that the original
   source code may be obtained as noted above.

5. There is no warranty or other guarantee of fitness for Qhull, it is
   provided solely "as is".  Bug reports or fixes may be sent to
   qhull_bug@qhull.org; the authors may or may not act on them as
   they desire.

### Geometry Center Qhull license

The license under which The Geometry Center distributed Qhull 2.x and 3.0;
it applies to `src/builder/qhull_ipion/`.

                    Qhull, Copyright (c) 1993-2001

       The National Science and Technology Research Center for
        Computation and Visualization of Geometric Structures
                        (The Geometry Center)
                       University of Minnesota
                            400 Lind Hall
                        207 Church Street S.E.
                      Minneapolis, MN 55455  USA

                       email: qhull@geom.umn.edu

This software includes Qhull from The Geometry Center.  Qhull is
copyrighted as noted above.  Qhull is free software and may be obtained
via anonymous ftp from geom.umn.edu.  It may be freely copied, modified,
and redistributed under the following conditions:

1. All copyright notices must remain intact in all files.

2. A copy of this text file must be distributed along with any copies
   of Qhull that you redistribute; this includes copies that you have
   modified, or copies of programs or other software products that
   include Qhull.

3. If you modify Qhull, you must include a notice giving the
   name of the person performing the modification, the date of
   modification, and the reason for such modification.

4. When distributing modified versions of Qhull, or other software
   products that include Qhull, you must provide notice that the original
   source code may be obtained as noted above.

5. There is no warranty or other guarantee of fitness for Qhull, it is
   provided solely "as is".  Bug reports or fixes may be sent to
   qhull_bug@geom.umn.edu; the authors may or may not act on them as
   they desire.

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
