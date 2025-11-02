# FieldMesh - Directional Field Visualization

A clean C++ project for mesh processing and directional field visualization using:
- **libigl** - Geometry processing library
- **Directional** - Directional field synthesis and processing
- **Polyscope** - Modern 3D visualization

## Quick Start

### Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j
```

### Run

```bash
# View default sphere
./FieldMesh

# Load custom mesh
./FieldMesh -i data/bunny.obj

# View standard cube
./FieldMesh --cube

# View standard sphere
./FieldMesh --sphere
```

## Usage

```
Usage: FieldMesh [options]

Options:
  -i, --input <file>     Load mesh from OBJ file
  --cube                 Load standard cube mesh
  --sphere               Load standard sphere mesh
  -h, --help             Show this help message
```

## Polyscope Controls

- **Click and drag** - Rotate view
- **Scroll** - Zoom in/out
- **Right-click + drag** - Pan view
- **ESC** - Exit

## Project Structure

```
Qremesh/
├── CMakeLists.txt         # Build configuration
├── src/
│   └── main.cpp          # Main application
├── data/                 # Mesh files (optional)
└── build/                # Build directory
```

## Dependencies

All dependencies are automatically fetched by CMake:
- libigl v2.5.0
- Polyscope v2.3.0
- Directional (latest)

## Features

- ✅ Simple mesh loading from OBJ files
- ✅ Built-in cube and sphere meshes
- ✅ Interactive 3D visualization with Polyscope
- ✅ Ready for directional field computation integration
- ✅ Clean, minimal codebase

## Next Steps

This is a starter template ready for:
- Cross field computation
- Directional field visualization
- Quad remeshing
- Field-aligned parameterization

See the [Directional library examples](https://github.com/avaxman/Directional) for field processing features.
