# Quad Remesher (MIQ)

Anisotropic quad remeshing using Mixed Integer Quadrangulation (MIQ). This project is a modification of the [anisotropic remeshing tutorial (Tutorial 506)](https://libigl.github.io/tutorial/#frame-fields) from [libigl](https://github.com/libigl/libigl).

Created with assistance from Claude (Anthropic).

## Overview

Given a triangle mesh and per-vertex direction fields (D1, D2), this tool:

1. Interpolates per-vertex directions to per-face cross field constraints
2. Computes a smooth frame field via CoMISo
3. Deforms the mesh for anisotropy support
4. Runs MIQ global parameterization
5. Extracts a quad mesh from the integer-grid UV map

## Dependencies

- [libigl](https://github.com/libigl/libigl) v2.5.0 (fetched automatically via CMake)
- [CoMISo](https://www.graphics.rwth-aachen.de/software/comiso/) (via libigl copyleft module)
- Eigen, OpenGL, GLFW (brought in by libigl)

## Build

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

On Apple Silicon, use the provided shell scripts:

```bash
./setup_arm.sh
./build_arm.sh
```

## Usage

```bash
./quadRemesher <mesh.obj> <D1.dmat> <D2.dmat> [output_folder] [name] [method]
```

- `method`: `direct` (default, skips integer solver) or `integration` (uses integer solver)

### Viewer Controls

| Key | Action |
|-----|--------|
| `+` / `-` | Increase / decrease gradient size (coarser / finer quads) |
| `S` | Extract and save quad mesh |
| `R` | Preview extracted quad mesh |
| `1` | Display original mesh |
| `2` | Display deformed mesh |
| `3` | Show constrained faces |
| `4` | Show cross field on deformed mesh |
| `5` | Show original input D1/D2 directions |
| `7` | Show singularities and seams |
