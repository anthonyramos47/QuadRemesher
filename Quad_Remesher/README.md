# Quad_Remesher

The remeshing tools themselves. See the [repository README](../README.md) for
build instructions, the full command line, and the viewer controls.

- `quadRemesher.cpp` — the pipeline: frame field interpolation
  (`comiso::frame_field`), anisotropic deformation (`frame_field_deformer`),
  cross field and combing, MIQ global parameterization, and quad extraction.
  Also hosts the interactive libigl viewer.
- `principalDirections.cpp` — an optional helper that generates a frame field
  from principal curvature (`igl::principal_curvature`) and writes it as DMAT,
  for when you have no field of your own. The remesher accepts any frame field;
  nothing in the pipeline assumes curvature.
- `CMakeLists.txt` — fetches libigl, CoMISo, OpenMesh and libQEx.

This started as a modification of libigl's
[anisotropic remeshing tutorial 506](https://libigl.github.io/tutorial/#frame-fields)
and has since diverged: flexible field input (DMAT or plain text, on vertices or
faces, whole-mesh or a subset), constraint thinning, libQEx extraction, a batch
mode, and field/seam diagnostics.

Quick check that the build works:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
./build/principalDirections ../test_meshes/metro.out.obj /tmp/m/
./build/quadRemesher ../test_meshes/metro.out.obj /tmp/m/D1.dmat /tmp/m/D2.dmat \
    --out /tmp/m --name metro qex batch constraints=0.1
```

## Prebuilt binaries (Linux, Windows, macOS)

`.github/workflows/release.yml` builds `quadRemesher` and `principalDirections`
on GitHub's Linux, Windows and macOS runners. Run it from the Actions tab
(*build-binaries → Run workflow*) to get the zips as artifacts, or push a tag
(`git tag v1.0.0 && git push origin v1.0.0`) to attach them to a Release.

- **Linux**: built on Ubuntu 22.04 with libstdc++, OpenMesh and BLAS linked
  statically; the only runtime requirement is glibc ≥ 2.35. `batch` runs work
  headless; the viewer needs OpenGL/X11, as on any desktop.
- **Windows**: MSVC with the static runtime — no redistributable needed.
- **macOS**: one universal (Apple Silicon + Intel) binary, macOS 11+. It is
  unsigned, so after unzipping run `xattr -dr com.apple.quarantine <folder>`.

Local builds are portable by default; add `-DQUADREMESHER_NATIVE=ON` to
optimize for your own CPU (the result may not run on other machines).

The binaries link libQEx and are therefore GPLv3; distribute them together
with (a link to) this source.
