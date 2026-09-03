# TODO

Work that is written but not yet closed out. Each entry says what is already
verified, so nobody repeats it, and what specifically remains.

---

## Smaller loose ends

* **`E1126` is disabled in `.cmake-format`.** cmakelang 0.6.13 predates
  `file(REAL_PATH)` (CMake 3.19) and cannot parse it, so the two
  toolchain-sentinel calls in the top-level `CMakeLists.txt` report as errors
  with the code enabled.

  **Blocked upstream, not merely unactioned.** 0.6.13 is cmakelang's latest
  release, not just the version pinned in
  `.github/workflows/cmake_lint.yml` — there is no newer parser to move to.
  The only alternative is rewriting correct CMake to suit a linter's gap,
  which is worse than the entry. Drop it when cmakelang next releases and the
  pin moves with it.
