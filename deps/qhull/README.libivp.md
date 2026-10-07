# Vendored Qhull

This directory contains the official reentrant C implementation from Qhull
2020.2 (8.0.2), tag `v8.0.2`:

- upstream: <https://github.com/qhull/qhull>
- imported subtree: `src/libqhull_r/*.[ch]`
- archive: `https://github.com/qhull/qhull/archive/refs/tags/v8.0.2.tar.gz`
- downloaded archive SHA-256:
  `8774e9a12c70b0180b95d6b0b563c5aa4bea8d5960c15e18ae3b6d2521d64f8b`

The files under `libqhull_r/` are unmodified upstream sources.  The adjacent
`qhull_api.[ch]` files are libivp's adapter and give each convex build its own
`qhT` context.  Independent contexts may execute concurrently; Qhull does not
permit concurrent calls using the same context.
