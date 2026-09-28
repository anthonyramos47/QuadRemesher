// Compute per-vertex principal curvature directions and write them as the
// D1/D2 DMAT files consumed by quadRemesher.
//
// The quad remesher expects two per-vertex direction fields. Principal
// curvature directions are a natural choice for a test field: they are
// well-defined wherever the surface is not umbilic, they are orthogonal to each
// other, and aligning a quad mesh to them is the classic curvature-aligned
// remeshing setup.
//
// Note on where the projection happens: this tool only produces the *per-vertex*
// field. quadRemesher then averages D1/D2 over each triangle's three vertices to
// get a value at the barycenter and projects that onto the face plane (see the
// per-face interpolation step in quadRemesher.cpp). We deliberately do not
// duplicate that step here.

#include <igl/readOBJ.h>
#include <igl/principal_curvature.h>
#include <igl/writeDMAT.h>
#include <igl/per_vertex_normals.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

int main(int argc, char *argv[])
{
    if (argc < 2) {
        cout << "Usage: ./principalDirections <mesh.obj> [output_prefix] [radius]" << endl;
        cout << endl;
        cout << "  Writes <output_prefix>D1.dmat and <output_prefix>D2.dmat," << endl;
        cout << "  the per-vertex principal curvature directions of the mesh." << endl;
        cout << "  output_prefix defaults to the current directory ('./')." << endl;
        cout << "  radius is the k-ring neighbourhood size for fitting (default 5)." << endl;
        return 1;
    }

    const string meshFile = argv[1];
    const string prefix = (argc >= 3) ? argv[2] : "./";
    const unsigned radius = (argc >= 4) ? (unsigned)atoi(argv[3]) : 5u;

    MatrixXd V;
    MatrixXi F;
    if (!igl::readOBJ(meshFile, V, F)) {
        cerr << "Error: could not read " << meshFile << endl;
        return 1;
    }
    cout << "Loaded " << meshFile << ": " << V.rows() << " vertices, " << F.rows() << " faces" << endl;

    if (F.cols() != 3) {
        cerr << "Error: expected a triangle mesh, got faces with " << F.cols() << " vertices." << endl;
        return 1;
    }

    // Principal directions (PD1/PD2) and magnitudes (PV1/PV2) per vertex.
    // The quadric-fitting used here needs a neighbourhood per vertex; vertices
    // where the fit fails are reported back in bad_vertices.
    MatrixXd PD1, PD2;
    VectorXd PV1, PV2;
    vector<int> bad_vertices;
    cout << "Computing principal curvature (k-ring radius " << radius << ")..." << endl;
    igl::principal_curvature(V, F, PD1, PD2, PV1, PV2, bad_vertices, radius, true);

    if (!bad_vertices.empty()) {
        cout << "  Warning: curvature fit failed at " << bad_vertices.size()
             << " / " << V.rows() << " vertices." << endl;
    }

    // Repair degenerate directions. On flat or umbilic regions the principal
    // directions are arbitrary and can come back as (near) zero vectors, which
    // would make the downstream per-face projection produce NaNs. Substitute an
    // arbitrary but consistent tangent frame derived from the vertex normal.
    MatrixXd N;
    igl::per_vertex_normals(V, F, N);

    int repaired = 0;
    for (int i = 0; i < V.rows(); i++) {
        Vector3d d1 = PD1.row(i);
        Vector3d n  = N.row(i);

        if (!d1.allFinite() || d1.norm() < 1e-8 || !n.allFinite() || n.norm() < 1e-8) {
            // Build any unit tangent: cross the normal with the least-aligned axis.
            if (!n.allFinite() || n.norm() < 1e-8) n = Vector3d::UnitZ();
            n.normalize();
            Vector3d axis = (fabs(n.x()) < 0.9) ? Vector3d::UnitX() : Vector3d::UnitY();
            d1 = n.cross(axis).normalized();
            repaired++;
        } else {
            n.normalize();
            // Project d1 into the tangent plane so D1 and D2 stay orthogonal
            // to the normal and to each other.
            d1 = (d1 - d1.dot(n) * n);
            if (d1.norm() < 1e-8) {
                Vector3d axis = (fabs(n.x()) < 0.9) ? Vector3d::UnitX() : Vector3d::UnitY();
                d1 = n.cross(axis);
                repaired++;
            }
            d1.normalize();
        }

        // Take D2 as the in-plane orthogonal companion rather than the raw PD2:
        // this guarantees an exactly orthogonal frame even where the fit was noisy.
        Vector3d d2 = n.cross(d1).normalized();

        PD1.row(i) = d1.transpose();
        PD2.row(i) = d2.transpose();
    }

    if (repaired > 0) {
        cout << "  Substituted an arbitrary tangent frame at " << repaired
             << " degenerate (flat/umbilic) vertices." << endl;
    }

    const string d1File = prefix + "D1.dmat";
    const string d2File = prefix + "D2.dmat";

    // Write ASCII DMAT (igl::writeDMAT's third argument is `ascii`). readDMAT
    // accepts binary too, but ASCII keeps the dumps inspectable.
    if (!igl::writeDMAT(d1File, PD1, true) || !igl::writeDMAT(d2File, PD2, true)) {
        cerr << "Error: could not write DMAT files to prefix '" << prefix << "'" << endl;
        return 1;
    }

    cout << "Wrote " << d1File << " and " << d2File
         << " (" << PD1.rows() << " x " << PD1.cols() << ")" << endl;

    // Report curvature spread: a mesh that is nearly flat everywhere gives a
    // near-arbitrary field and will not produce a meaningful curvature-aligned
    // quad layout.
    cout << "  |k1| range: [" << PV1.cwiseAbs().minCoeff() << ", " << PV1.cwiseAbs().maxCoeff() << "]" << endl;
    cout << "  |k2| range: [" << PV2.cwiseAbs().minCoeff() << ", " << PV2.cwiseAbs().maxCoeff() << "]" << endl;

    return 0;
}
