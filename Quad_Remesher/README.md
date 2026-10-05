# QuadRemesher — user guide

`quadRemesher` turns a triangle mesh into a quad mesh whose edges follow a
frame field (two directions per point of the surface). It interpolates the
field, integrates it into a seamless parameterization (MIQ), and extracts the
quad mesh (libQEx).

It runs in two ways:

- **Interactive** (default): opens a viewer showing the mesh, the frame field
  and the integration lines. Change the density with `+`/`-`, preview the quad
  mesh with `R`, save it with `S`.
- **Batch** (`--batch`): no window. Integrates, extracts, writes the quad mesh
  and exits. Use this from scripts and other programs.

```
quadRemesher <mesh.obj|mesh.off> [options]
quadRemesher --help
```

## Installing

Download the zip for your system from the repository's *Releases* page and
unzip it. The programs are in `bin/`; nothing else needs to be installed.

- **Windows**: `bin\quadRemesher.exe` (keep the `.dll` files next to it).
- **macOS** (Apple Silicon and Intel, macOS 11+): the binary is not signed, so
  run once after unzipping: `xattr -dr com.apple.quarantine QuadRemesher-macos-universal`
- **Linux** (x86-64, glibc 2.35+, e.g. Ubuntu 22.04 or newer): `bin/quadRemesher`.
  `--batch` also works on headless servers.

## Input mesh

A **triangle mesh** in **OBJ** or **OFF** format. Meshes with quads or other
polygons are rejected; triangulate them first. Vertex and face indices in the
field files below refer to the order of vertices and faces in this file,
**starting at 0**.

## Choosing the frame field: `--field`

| `--field` | where the directions come from | extra inputs |
|---|---|---|
| `data` (default) | your files | `--d1`, `--d2`, optional `--indices` |
| `principal` | principal curvature directions | `--radius` |
| `conjugate` | conjugate directions (see below) | optional `--d1` or `--angle`; `--radius` |
| `orthogonal` | the smoothest orthogonal cross field | none |

### `data` — directions from files

```
--d1 D1.dat --d2 D2.dat [--indices indices.dat] [--defined_on vertices|faces] [--format vec|dmat]
```

- **`--defined_on`** says what the rows describe: `vertices` (default) or `faces`.
- **`--d1`, `--d2`**: the first and second direction, one unit vector per row.
- **`--format`** says how D1/D2 are written:
  - `vec` (default): plain text, one `vx vy vz` per line. Blank lines and `#`
    comments are ignored.
    ```
    # D1.dat
    0.707 0.707 0
    1 0 0
    ```
  - `dmat`: libigl's DMAT format (`igl::writeDMAT`), either `n×3` or `3×n`.
- **`--indices`** (optional): whitespace-separated indices of the vertices or
  faces the rows belong to, in the same order as the rows, e.g. `0 12 13 40`.
  - Omit it when D1/D2 have exactly one row per vertex (or face) of the mesh.
  - With it, only the listed elements carry a direction and the field is
    interpolated everywhere else (like libigl tutorial 506).

How the data becomes one frame per face:

- **Per-face data** is used on its face, projected into the face plane.
- **Per-vertex data** is averaged at each face's barycenter over the face's
  vertices that **have** data (all three if available, otherwise only those
  given), then projected into the face plane. Faces with no vertex data are
  filled by the interpolation.

The directions are normalized and their sign is ignored: they set the
*orientation* of the quads, not their size. D1 and D2 need not be orthogonal.

### `principal` — curvature directions

Principal curvature directions from `igl::principal_curvature` at every vertex.
`--radius R` sets the k-ring used to estimate curvature (default 5; larger is
smoother).

### `conjugate` — conjugate directions

Two directions are conjugate when the second fundamental form II(D1, D2) = 0.
Quad meshes along conjugate directions have (nearly) planar faces. Three ways:

| options | D1 | D2 |
|---|---|---|
| `--d1 FILE` (+ `--indices`, `--defined_on`, `--format` as for `data`) | given | conjugate of D1 |
| `--angle DEG` | on each face, the first principal direction rotated by DEG in the face plane | conjugate of D1 |
| neither | symmetric pair about the principal directions where the Gaussian curvature K > 0; the principal directions elsewhere | |

Conjugate directions are undefined where the surface is flat. They are also
degenerate where D1 is near an asymptotic direction, since there the conjugate
coincides with D1. Elements in either situation are skipped and filled by the
interpolation.

### `orthogonal` — smoothest cross field

The smoothest orthogonal cross field on the surface (libigl tutorial 504). It
needs no input and follows the shape of the surface rather than its curvature.

### Constraint density: `--constraints F`

Keeps only a fraction `F` (0 < F ≤ 1, default 1) of the faces that received a
direction, spread evenly over the surface. The solver then interpolates the
rest. Constraining every face forces the field through every small wobble of
the input, and each wobble the solver can't smooth out becomes a singularity.
`--constraints 0.1` usually gives a much cleaner mesh, especially with
`principal` and `conjugate`.

## Quad density: `--gradient H`

The integration gradient size (default 40). **Higher = denser quads.** The
value is relative to the mesh's bounding box, so the same `H` gives a similar
density on large and small meshes. As a rough guide, on the test mesh
`lilium`, `H` = 20 / 40 / 80 gave 181 / 793 / 3364 quads.

In the viewer, `+` multiplies it by 1.5 and `-` divides it by 1.5.

## Output

`--out DIR --name NAME` writes `DIR/NAME_Remeshed.obj` (`DIR` is created if
needed). The defaults are the current folder and the input mesh's file name.
The file is a polygon OBJ: mostly quads, plus a few other polygons where the
extractor needs them.

Other options:

- `--extractor qex|grid`: `qex` (default) is robust near seams and
  singularities. `grid` is a simple built-in extractor that can tear there.
- `--dump-fields`: also writes the intermediate fields as DMAT files, for
  debugging.

The exit code is 0 on success and 1 on any error, with the reason printed on
stderr.

## Interactive viewer

Without `--batch` the viewer opens on the input mesh, showing the frame field
(red = first direction, blue = second) and the integration lines.

| key | action |
|---|---|
| `+` / `-` | denser / coarser quads (re-runs the integration) |
| `R` | extract the quad mesh and show it |
| `S` | extract the quad mesh and save it to the output file |
| `1` | input mesh with frame field and integration lines |
| `2` | the internally deformed mesh (anisotropy) |
| `3` | the input field as given |
| `4` | the interpolated frame field |
| `5` | the integrated (combed) cross field |
| `6` | seams and singularities |
| `7` | deformed mesh with its cross field |
| `0` | toggle seams/singularities on views 3–7 |

Magenta lines are seams; green / purple dots are +1/4 / −1/4 singularities.

## Examples

```bash
# Your own per-vertex field covering every vertex, interactive
quadRemesher mesh.obj --d1 D1.dat --d2 D2.dat

# Your own field on a subset of faces, batch
quadRemesher mesh.off --d1 D1.dat --d2 D2.dat --indices faces.dat \
    --defined_on faces --gradient 60 --batch --out results --name mesh

# Same with libigl DMAT files
quadRemesher mesh.obj --d1 D1.dmat --d2 D2.dmat --format dmat --batch

# Curvature-aligned
quadRemesher mesh.obj --field principal --constraints 0.1 --batch

# Conjugate: from a given D1, at a constant 30° angle, or automatic
quadRemesher mesh.obj --field conjugate --d1 D1.dat --constraints 0.1 --batch
quadRemesher mesh.obj --field conjugate --angle 30 --constraints 0.1 --batch
quadRemesher mesh.obj --field conjugate --constraints 0.1 --batch

# Smoothest cross field
quadRemesher mesh.obj --field orthogonal --batch
```

`principalDirections mesh.obj outdir/ [radius]` is a small helper that writes
the principal curvature directions as `outdir/D1.dmat` and `outdir/D2.dmat`,
handy as a starting point for your own field files.

## License

GPLv3, see `LICENSE`: the program links CoMISo and libQEx, which are GPLv3.
The source is at https://github.com/anthonyramos47/QuadRemesher. The bundled
third-party components and their licenses are listed in `THIRD_PARTY_NOTICES.md`.

## References

- Bommes et al., *Mixed-Integer Quadrangulation*, SIGGRAPH 2009
- Panozzo et al., *Frame Fields: Anisotropic and Non-Orthogonal Cross Fields*, SIGGRAPH 2014
- Ebke et al., *QEx: Robust Quad Mesh Extraction*, SIGGRAPH Asia 2013
