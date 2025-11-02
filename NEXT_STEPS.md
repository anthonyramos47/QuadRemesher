# Next Steps - Adding Directional Field Features

## Current Status ✅

Your project has:
- ✅ Working Polyscope visualization
- ✅ Mesh loading (OBJ + primitives)
- ✅ libigl integrated
- ✅ Directional library available
- ✅ Clean, extensible code

## How to Add Cross Field Computation

### Step 1: Add includes to main.cpp

```cpp
#include <igl/principal_curvature.h>
#include <igl/local_basis.h>

// Note: Directional v2.0 uses CartesianField API
// #include <directional/CartesianField.h>
// #include <directional/TangentBundle.h>
```

### Step 2: Compute cross field from curvature

```cpp
// After loading mesh (V, F)...

// Compute principal curvature directions
Eigen::MatrixXd PD1, PD2;  // Principal directions
Eigen::VectorXd PV1, PV2;  // Principal values
igl::principal_curvature(V, F, PD1, PD2, PV1, PV2);

// Compute local tangent basis per face
Eigen::MatrixXd B1, B2, B3;
igl::local_basis(V, F, B1, B2, B3);

// Convert vertex-based to face-based directions
Eigen::MatrixXd facePD1(F.rows(), 3);
Eigen::MatrixXd facePD2(F.rows(), 3);

for (int f = 0; f < F.rows(); f++) {
    // Average directions from face vertices
    Eigen::Vector3d avg1 = (PD1.row(F(f,0)) + PD1.row(F(f,1)) + PD1.row(F(f,2))).normalized();
    Eigen::Vector3d avg2 = (PD2.row(F(f,0)) + PD2.row(F(f,1)) + PD2.row(F(f,2))).normalized();

    facePD1.row(f) = avg1;
    facePD2.row(f) = avg2;
}

std::cout << "Cross field computed!\n";
```

### Step 3: Visualize fields in Polyscope

```cpp
// Compute face centers for vector placement
Eigen::MatrixXd faceCenters(F.rows(), 3);
for (int f = 0; f < F.rows(); f++) {
    faceCenters.row(f) = (V.row(F(f,0)) + V.row(F(f,1)) + V.row(F(f,2))) / 3.0;
}

// Add to Polyscope
auto* psMesh = polyscope::registerSurfaceMesh("input mesh", V, F);

// Visualize first principal direction
psMesh->addFaceVectorQuantity("PD1", facePD1)
    ->setVectorColor({1.0, 0.0, 0.0})  // Red
    ->setVectorLengthScale(0.1)
    ->setEnabled(true);

// Visualize second principal direction
psMesh->addFaceVectorQuantity("PD2", facePD2)
    ->setVectorColor({0.0, 0.0, 1.0})  // Blue
    ->setVectorLengthScale(0.1)
    ->setEnabled(true);

polyscope::show();
```

## Example: Complete main.cpp with Fields

Here's what to add after loading the mesh in `src/main.cpp`:

```cpp
// After the mesh loading section...

#ifdef ENABLE_FIELD_COMPUTATION
    std::cout << "\nComputing cross field from principal curvature...\n";

    Eigen::MatrixXd PD1, PD2;
    Eigen::VectorXd PV1, PV2;
    igl::principal_curvature(V, F, PD1, PD2, PV1, PV2);

    // Convert to face-based
    Eigen::MatrixXd facePD1(F.rows(), 3);
    Eigen::MatrixXd facePD2(F.rows(), 3);

    for (int f = 0; f < F.rows(); f++) {
        facePD1.row(f) = (PD1.row(F(f,0)) + PD1.row(F(f,1)) + PD1.row(F(f,2))).normalized();
        facePD2.row(f) = (PD2.row(F(f,0)) + PD2.row(F(f,1)) + PD2.row(F(f,2))).normalized();
    }

    std::cout << "Cross field computed successfully!\n";
#endif

#ifdef USE_POLYSCOPE
    // ... existing Polyscope code ...
    auto* psMesh = polyscope::registerSurfaceMesh("input mesh", V, F);

#ifdef ENABLE_FIELD_COMPUTATION
    // Add field vectors
    psMesh->addFaceVectorQuantity("PD1", facePD1)
        ->setVectorColor({1.0, 0.0, 0.0})
        ->setVectorLengthScale(0.1);
    psMesh->addFaceVectorQuantity("PD2", facePD2)
        ->setVectorColor({0.0, 0.0, 1.0})
        ->setVectorLengthScale(0.1);
#endif

    polyscope::show();
#endif
```

## Directional v2.0 Note

The current Directional library (v2.0+) uses a new API with:
- `CartesianField` class for fields
- `TangentBundle` class for mesh structure
- Object-oriented design

For full integration, you'll need to:
1. Create a `TangentBundle` from your mesh
2. Use `CartesianField` to store directional data
3. Use Directional's field processing functions

See [Directional documentation](https://github.com/avaxman/Directional) for v2.0 examples.

## Quick Test

1. Add the includes:
```cpp
#include <igl/principal_curvature.h>
#include <igl/local_basis.h>
```

2. Add the computation code after mesh loading

3. Rebuild:
```bash
cd build
cmake --build .
```

4. Run:
```bash
./FieldMesh --sphere
```

You should see red and blue vectors showing the cross field directions!

## Resources

- **Polyscope Docs**: https://polyscope.run/
- **libigl Tutorial**: https://libigl.github.io/tutorial/
- **Directional**: https://github.com/avaxman/Directional
- **Cross Fields Paper**: "Designing N-PolyVector Fields with Complex Polynomials"

---

The foundation is ready - now you can build amazing directional field applications! 🚀
