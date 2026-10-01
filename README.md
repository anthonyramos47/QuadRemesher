# QuadRemesher

Frame-field aligned quad remeshing built on [libigl](https://libigl.github.io/)'s
Mixed-Integer Quadrangulation (MIQ), with robust quad extraction via
[libQEx](https://github.com/hcebke/libQEx).

Given a triangle mesh and a pair of direction fields defining the desired quad
orientation, it interpolates a frame field over the surface, computes a seamless
global parameterization with MIQ, and extracts a quad mesh whose edges follow
the prescribed directions.

The frame field is the input and can come from anywhere — a conjugate or
principal-stress field, a designed or hand-authored field, a field exported from
another tool, or curvature. It may be given on vertices or on faces, for the
whole mesh or for a subset of elements, and the solver interpolates it across
the rest. `principalDirections` is included as one convenient source, computing
principal curvature directions, but it is optional and nothing in the pipeline
assumes curvature.

## Build

Dependencies (libigl, CoMISo, OpenMesh, libQEx) are fetched automatically by
CMake. The first configure downloads and compiles them, which takes several
minutes.

```bash
cd Quad_Remesher
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Produces `build/quadRemesher` and `build/principalDirections`.

Requires CMake ≥ 3.16 and a C++17 compiler. On CMake ≥ 4.0 the build sets
`CMAKE_POLICY_VERSION_MINIMUM=3.5` automatically, since several vendored
dependencies still declare pre-3.5 minimums.

libQEx is GPLv3, so linking it makes the resulting binary GPLv3. Configure with
`-DUSE_LIBQEX=OFF` to build without it; the internal grid extractor is then the
only extractor available.

## Usage

```
quadRemesher <mesh.obj> <D1> <D2> [frame_indices] [options]
```

`D1` and `D2` hold the two direction fields, one row per element. With a
`frame_indices` file, the rows instead correspond to the listed indices in
order; without one they must cover every element of the mesh.

| option | values | meaning |
|---|---|---|
| `--type` | `DMAT` (default), `vec` | `vec` is one `vx vy vz` per line |
| `--defined_on` | `vertices` (default), `faces` | vertex fields are averaged onto each face barycenter and projected into the face plane |
| `--out`, `--name` | | output is `<out>/<name>_Remeshed.obj` |
| `constraints=<f>` | 0 < f ≤ 1 | keep only a fraction of the constraint faces (farthest-point sampled) |
| `direct` / `integration` | | skip or run the integer solver (`qex` forces `integration`) |
| `grid` / `qex` | | quad extractor (see below) |
| `batch` | | extract, write and exit without opening the viewer |
| `arbitrary[=x,y,z]` | | ignore D1/D2 and use an arbitrary 4-RoSy (diagnostic) |

### Examples

Remesh with your own per-face frame field, in plain text, given on a subset of
faces (the remaining faces are filled in by the interpolation):

```bash
cd Quad_Remesher
./build/quadRemesher mesh.obj D1.vec D2.vec frame_faces.dat \
    --type vec --defined_on faces --out ./out --name mesh qex batch
```

The same with the field on vertices and covering the whole mesh, so no index
file is needed:

```bash
./build/quadRemesher mesh.obj D1.vec D2.vec \
    --type vec --defined_on vertices --out ./out --name mesh qex batch
```

If you have no field of your own, `principalDirections` generates one from
principal curvature:

```bash
./build/principalDirections ../test_meshes/lilium.obj /tmp/lil/
./build/quadRemesher ../test_meshes/lilium.obj /tmp/lil/D1.dmat /tmp/lil/D2.dmat \
    --out /tmp/lil --name lilium qex batch constraints=0.1
```

## Interactive viewer

Omitting `batch` opens the libigl viewer:

| key | view |
|---|---|
| `1` / `2` | original / deformed mesh with the UV texture |
| `3` | input frame field, as supplied |
| `4` | interpolated frame field (`comiso::frame_field`) |
| `5` | integrated (combed) cross field — what MIQ integrated |
| `6` | seams and singularities |
| `7` | deformed mesh with its cross field |
| `R` / `S` | preview / save the extracted quad mesh |
| `+` / `-` | coarser / finer quads (re-runs MIQ) |
| `0` | toggle the seam overlay on views 3–7 |

Red is the first direction, blue the second; magenta lines are seams and
green/purple dots are +1/4 and −1/4 singularities.

## Quad extractors

- **`qex`** — libQEx (Ebke et al. 2013), robust to the imperfect integer-grid
  maps MIQ produces near seams and singularities. Requires `integration` mode,
  which it selects automatically. May emit a few n-gons alongside the quads.
- **`grid`** (default) — a built-in extractor that rasterizes the integer UV
  grid. Fine where the parameterization is non-degenerate, but its
  merge-by-position heuristic tears near seams, so prefer `qex` for quality.

## Constraint density

How much of the mesh you constrain matters. Prescribing a frame on every face
leaves the solver no freedom and forces the field through every local wobble of
the input, each of which can become a singularity. Constraining a sparse subset
and letting the solver interpolate — the way libigl tutorial 506 drives this —
gives markedly cleaner results.

Supply a sparse field directly via the index file, or thin a dense one with
`constraints=<f>`, which keeps a farthest-point-sampled fraction of the
constrained faces. On the bundled test meshes `constraints=0.1` reduces
singularity counts by roughly 75–85%.

## Field input

D1 and D2 are the two representative directions of the frame at each element.
They are projected into each face plane and need not be orthogonal; because the
field is a 4-RoSy, the sign and the order of the two directions carry no
meaning.

Note that the directions are **normalized**, so their magnitudes are ignored:
the field controls quad *orientation*, not quad size. Element size is set by
the gradient-size parameter (`+` / `-` in the viewer) and is uniform. Feeding
in frame lengths to drive anisotropic sizing, as in Panozzo et al., is not
wired up.

Rows correspond to every element of the mesh, or, when an index file is given,
to the listed elements in that order. With `--defined_on vertices` a face is
constrained only when all three of its vertices carry a direction.

`principalDirections` is a helper, not part of the pipeline: it computes
per-vertex principal curvature directions with `igl::principal_curvature` and
writes them as DMAT, for use as D1/D2 when you have no field of your own. It
emits only the per-vertex field; the interpolation onto faces happens in
`quadRemesher`.

## References

- Bommes et al., *Mixed-Integer Quadrangulation*, SIGGRAPH 2009
- Panozzo et al., *Frame Fields: Anisotropic and Non-Orthogonal Cross Fields*, SIGGRAPH 2014
- Ebke et al., *QEx: Robust Quad Mesh Extraction*, SIGGRAPH Asia 2013
