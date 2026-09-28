# Quad_Remesher

The remeshing tools themselves. See the [repository README](../README.md) for
build instructions, the full command line, and the viewer controls.

- `quadRemesher.cpp` — the pipeline: frame field interpolation
  (`comiso::frame_field`), anisotropic deformation (`frame_field_deformer`),
  cross field and combing, MIQ global parameterization, and quad extraction.
  Also hosts the interactive libigl viewer.
- `principalDirections.cpp` — computes per-vertex principal curvature
  directions (`igl::principal_curvature`) and writes them as DMAT, to be used
  as the D1/D2 input.
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
