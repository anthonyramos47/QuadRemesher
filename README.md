# QuadRemesher

Frame-field aligned quad remeshing built on [libigl](https://libigl.github.io/)'s
Mixed-Integer Quadrangulation (MIQ), with robust quad extraction via
[libQEx](https://github.com/hcebke/libQEx).

Given a triangle mesh and a pair of direction fields defining the desired quad
orientation, it interpolates a frame field over the surface, computes a seamless
global parameterization with MIQ, and extracts a quad mesh whose edges follow
the prescribed directions.

The frame field can be read from files (on vertices or faces, for the whole
mesh or a subset of elements; the solver interpolates the rest) or computed
from the surface: principal curvature, conjugate directions, or the smoothest
orthogonal cross field.

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

CoMISo (used for MIQ) and libQEx are GPLv3, so the resulting binaries are
GPLv3. `-DUSE_LIBQEX=OFF` builds without libQEx (the built-in grid extractor is
then the only one); it does not change the license, since CoMISo remains.

Local builds are portable by default. Add `-DQUADREMESHER_NATIVE=ON` to
optimize for your own CPU (the result may not run on other machines).

## Usage

See the **[user guide](Quad_Remesher/README.md)**: input formats, the field
options (`data`, `principal`, `conjugate`, `orthogonal`), quad density, batch
mode, the viewer keys and examples. The same guide ships inside each release
zip. `quadRemesher --help` prints a summary.

```bash
./build/quadRemesher ../test_meshes/lilium.obj --field principal --constraints 0.1 \
    --batch --out /tmp/lil
```

## Prebuilt binaries (Linux, Windows, macOS)

`.github/workflows/release.yml` builds `quadRemesher` and `principalDirections`
on GitHub's Linux, Windows and macOS runners and smoke-tests them on
`test_meshes/lilium.obj`. Push a tag (`git tag v1.0.0 && git push origin v1.0.0`)
to attach the zips to a GitHub Release, or start it from the Actions tab
(*build-binaries → Run workflow*) to get them as run artifacts.

- **Linux**: built on Ubuntu 22.04 with libstdc++, OpenMesh and BLAS linked
  statically; needs only glibc ≥ 2.35.
- **Windows**: MSVC with the static runtime; CoMISo's OpenBLAS DLLs ship in `bin/`.
- **macOS**: one universal (Apple Silicon + Intel) binary for macOS 11+, unsigned.

## References

- Bommes et al., *Mixed-Integer Quadrangulation*, SIGGRAPH 2009
- Panozzo et al., *Frame Fields: Anisotropic and Non-Orthogonal Cross Fields*, SIGGRAPH 2014
- Ebke et al., *QEx: Robust Quad Mesh Extraction*, SIGGRAPH Asia 2013
