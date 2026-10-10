# Third-party notices

libivp contains or uses the following third-party code.  Copyright and
license notices are kept with the relevant source or reproduced here.

## Qhull

- Not vendored.  CMake fetches an immutable official archive from
  https://github.com/qhull/qhull: Qhull 2020.2 (8.0.2) by default, or Qhull
  3.0 for reference-development builds.
- Each fetched archive contains its original `COPYING.txt`.  Both license
  texts are reproduced below for distributions of libivp.

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

### Qhull 3.0 license

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
