// Quad remesher using MIQ (Mixed Integer Quadrangulation)
// Based on libigl tutorial 506_FrameField
//
// This implements proper anisotropic quad remeshing using:
// 1. Cross field from input D1/D2 directions
// 2. MIQ global parameterization
// 3. Quad extraction from UV grid

#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/readDMAT.h>
#include <igl/comb_cross_field.h>
#include <igl/comb_frame_field.h>
#include <igl/compute_frame_field_bisectors.h>
#include <igl/cross_field_mismatch.h>
#include <igl/cut_mesh_from_singularities.h>
#include <igl/find_cross_field_singularities.h>
#include <igl/jet.h>
#include <igl/avg_edge_length.h>
#include <igl/barycenter.h>
#include <igl/local_basis.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/rotate_vectors.h>
#include <igl/frame_field_deformer.h>
#include <igl/frame_to_cross_field.h>
#include <igl/copyleft/comiso/frame_field.h>
#include <igl/copyleft/comiso/miq.h>
#include <igl/copyleft/comiso/nrosy.h>
#include <igl/PI.h>
#include <igl/principal_curvature.h>
#include <igl/writeDMAT.h>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <array>
#include <functional>
#include <sstream>
#include <sys/stat.h>
#include <limits>
#include <algorithm>
#include <igl/opengl/glfw/Viewer.h>

#ifdef HAVE_LIBQEX
#include <qex.h>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <cstdlib>
#endif

using namespace std;
using namespace Eigen;

// Global state
MatrixXd V, V_uv;
MatrixXi F, F_uv;
MatrixXd D1, D2;       // Input cross field (per-face)
MatrixXd X1, X2;       // Combed cross field
MatrixXd B;            // Face barycenters
double global_scale;

// Display options
int display_mode = 1;
bool show_cross_field = false;
// Overlay seams + singularities on the field views ('0' toggles).
bool show_seams_overlay = true;

// Draw cross field as lines from face barycenters
void drawCrossField(igl::opengl::glfw::Viewer& viewer,
                    const MatrixXd& B_centers,
                    const MatrixXd& X1,
                    const MatrixXd& X2,
                    double scale)
{
    int nF = B_centers.rows();

    // Each face has 4 lines (cross field = 4 directions)
    MatrixXd P1(nF * 4, 3);  // Line start points
    MatrixXd P2(nF * 4, 3);  // Line end points
    MatrixXd C(nF * 4, 3);   // Colors

    for (int i = 0; i < nF; i++) {
        Vector3d center = B_centers.row(i);
        Vector3d d1 = X1.row(i);
        Vector3d d2 = X2.row(i);

        // Draw 4 directions of the cross field (±X1, ±X2)
        // +X1 direction (red)
        P1.row(i * 4 + 0) = center;
        P2.row(i * 4 + 0) = center + scale * d1;
        C.row(i * 4 + 0) = RowVector3d(1, 0, 0);

        // -X1 direction (red)
        P1.row(i * 4 + 1) = center;
        P2.row(i * 4 + 1) = center - scale * d1;
        C.row(i * 4 + 1) = RowVector3d(1, 0, 0);

        // +X2 direction (blue)
        P1.row(i * 4 + 2) = center;
        P2.row(i * 4 + 2) = center + scale * d2;
        C.row(i * 4 + 2) = RowVector3d(0, 0, 1);

        // -X2 direction (blue)
        P1.row(i * 4 + 3) = center;
        P2.row(i * 4 + 3) = center - scale * d2;
        C.row(i * 4 + 3) = RowVector3d(0, 0, 1);
    }

    viewer.data().add_edges(P1, P2, C);
}

// Create line texture for visualizing UV
void line_texture(Matrix<unsigned char,Dynamic,Dynamic> &texture_R,
                  Matrix<unsigned char,Dynamic,Dynamic> &texture_G,
                  Matrix<unsigned char,Dynamic,Dynamic> &texture_B)
{
    unsigned size = 128;
    unsigned size2 = size/2;
    unsigned lineWidth = 3;
    texture_R.setConstant(size, size, 255);
    for (unsigned i=0; i<size; ++i)
        for (unsigned j=size2-lineWidth; j<=size2+lineWidth; ++j)
            texture_R(i,j) = 0;
    for (unsigned i=size2-lineWidth; i<=size2+lineWidth; ++i)
        for (unsigned j=0; j<size; ++j)
            texture_R(i,j) = 0;
    texture_G = texture_R;
    texture_B = texture_R;
}

// Helper: compute barycentric coordinates of point p in triangle (a, b, c)
bool barycentricCoords(const Vector2d& p, const Vector2d& a, const Vector2d& b, const Vector2d& c,
                       double& u, double& v, double& w, double tol = 0.01)
{
    Vector2d v0 = b - a, v1 = c - a, v2 = p - a;
    double d00 = v0.dot(v0);
    double d01 = v0.dot(v1);
    double d11 = v1.dot(v1);
    double d20 = v2.dot(v0);
    double d21 = v2.dot(v1);
    double denom = d00 * d11 - d01 * d01;
    if (abs(denom) < 1e-12) return false;

    v = (d11 * d20 - d01 * d21) / denom;
    w = (d00 * d21 - d01 * d20) / denom;
    u = 1.0 - v - w;

    return (u >= -tol && v >= -tol && w >= -tol && u <= 1+tol && v <= 1+tol && w <= 1+tol);
}

// Helper: find 3D position for a UV point by searching all triangles
bool findPositionForUV(const Vector2d& uvPoint, const MatrixXd& V_3d, const MatrixXi& F_tri,
                       const MatrixXd& UV, const MatrixXi& F_UV, Vector3d& pos3d, int& foundFace)
{
    double tol = 0.02;
    for (int fi = 0; fi < F_UV.rows(); fi++) {
        Vector2d uv0 = UV.row(F_UV(fi, 0)).transpose();
        Vector2d uv1 = UV.row(F_UV(fi, 1)).transpose();
        Vector2d uv2 = UV.row(F_UV(fi, 2)).transpose();

        double u, v, w;
        if (barycentricCoords(uvPoint, uv0, uv1, uv2, u, v, w, tol)) {
            // Clamp barycentric coords
            u = max(0.0, min(1.0, u));
            v = max(0.0, min(1.0, v));
            w = max(0.0, min(1.0, w));
            double sum = u + v + w;
            u /= sum; v /= sum; w /= sum;

            // Use original triangle for 3D position
            int origFi = fi % F_tri.rows();
            pos3d = u * V_3d.row(F_tri(origFi, 0)).transpose() +
                    v * V_3d.row(F_tri(origFi, 1)).transpose() +
                    w * V_3d.row(F_tri(origFi, 2)).transpose();
            foundFace = fi;
            return true;
        }
    }
    return false;
}

// Global grid quad extraction from UV parameterization
// Searches all triangles for each grid point, then creates quads from the global grid
bool extractQuadMesh(const MatrixXd& V_3d, const MatrixXi& F_tri,
                     const MatrixXd& UV, const MatrixXi& F_UV,
                     MatrixXd& V_quad, MatrixXi& F_quad,
                     const Eigen::Matrix<int, Eigen::Dynamic, 1>* singularityIndex = nullptr)
{
    cout << "Extracting quad mesh from UV (global grid method)..." << endl;

    // Find UV bounding box
    double uMin = UV.col(0).minCoeff();
    double uMax = UV.col(0).maxCoeff();
    double vMin = UV.col(1).minCoeff();
    double vMax = UV.col(1).maxCoeff();

    int uStart = (int)floor(uMin);
    int uEnd = (int)ceil(uMax);
    int vStart = (int)floor(vMin);
    int vEnd = (int)ceil(vMax);

    cout << "  UV range: [" << uMin << "," << uMax << "] x [" << vMin << "," << vMax << "]" << endl;
    cout << "  Grid range: [" << uStart << "," << uEnd << "] x [" << vStart << "," << vEnd << "]" << endl;

    // Build spatial hash for triangle lookup
    map<pair<int, int>, vector<int>> cellToFaces;
    for (int fi = 0; fi < F_UV.rows(); fi++) {
        Vector2d uv0 = UV.row(F_UV(fi, 0)).transpose();
        Vector2d uv1 = UV.row(F_UV(fi, 1)).transpose();
        Vector2d uv2 = UV.row(F_UV(fi, 2)).transpose();

        int minU = (int)floor(min({uv0(0), uv1(0), uv2(0)})) - 1;
        int maxU = (int)ceil(max({uv0(0), uv1(0), uv2(0)})) + 1;
        int minV = (int)floor(min({uv0(1), uv1(1), uv2(1)})) - 1;
        int maxV = (int)ceil(max({uv0(1), uv1(1), uv2(1)})) + 1;

        for (int u = minU; u <= maxU; u++) {
            for (int v = minV; v <= maxV; v++) {
                cellToFaces[make_pair(u, v)].push_back(fi);
            }
        }
    }

    // For merging vertices by 3D position
    auto pos3dKey = [](const Vector3d& p) -> tuple<long,long,long> {
        return make_tuple(
            (long)round(p(0) * 100000),
            (long)round(p(1) * 100000),
            (long)round(p(2) * 100000)
        );
    };

    vector<Vector3d> quadVertices;
    map<tuple<long,long,long>, int> posToVertex;

    auto getOrCreateVertex = [&](const Vector3d& pos) -> int {
        auto key = pos3dKey(pos);
        auto it = posToVertex.find(key);
        if (it != posToVertex.end()) {
            return it->second;
        }
        int idx = quadVertices.size();
        quadVertices.push_back(pos);
        posToVertex[key] = idx;
        return idx;
    };

    // Global grid to vertex mapping
    map<pair<int,int>, int> gridToVertex;

    // For each integer grid point, find a containing triangle and compute 3D position
    for (int u = uStart; u <= uEnd; u++) {
        for (int v = vStart; v <= vEnd; v++) {
            Vector2d uvPoint(u, v);

            // Get candidate triangles
            auto it = cellToFaces.find(make_pair(u, v));
            if (it == cellToFaces.end()) continue;

            // Search triangles for one containing this point
            for (int fi : it->second) {
                Vector2d uv0 = UV.row(F_UV(fi, 0)).transpose();
                Vector2d uv1 = UV.row(F_UV(fi, 1)).transpose();
                Vector2d uv2 = UV.row(F_UV(fi, 2)).transpose();

                double bu, bv, bw;
                if (barycentricCoords(uvPoint, uv0, uv1, uv2, bu, bv, bw, 0.01)) {
                    // Clamp barycentric coords
                    bu = max(0.0, min(1.0, bu));
                    bv = max(0.0, min(1.0, bv));
                    bw = max(0.0, min(1.0, bw));
                    double sum = bu + bv + bw;
                    if (sum > 0) {
                        bu /= sum; bv /= sum; bw /= sum;
                    }

                    // Compute 3D position
                    int origFi = fi % F_tri.rows();
                    Vector3d pos3d = bu * V_3d.row(F_tri(origFi, 0)).transpose() +
                                     bv * V_3d.row(F_tri(origFi, 1)).transpose() +
                                     bw * V_3d.row(F_tri(origFi, 2)).transpose();

                    int vidx = getOrCreateVertex(pos3d);
                    gridToVertex[make_pair(u, v)] = vidx;
                    break;  // Found a triangle, stop searching
                }
            }
        }
    }

    cout << "  Found " << gridToVertex.size() << " grid vertices" << endl;

    // Create quads from the global grid
    vector<Vector4i> quadFaces;
    for (int u = uStart; u < uEnd; u++) {
        for (int v = vStart; v < vEnd; v++) {
            auto it00 = gridToVertex.find(make_pair(u, v));
            auto it10 = gridToVertex.find(make_pair(u+1, v));
            auto it11 = gridToVertex.find(make_pair(u+1, v+1));
            auto it01 = gridToVertex.find(make_pair(u, v+1));

            if (it00 != gridToVertex.end() && it10 != gridToVertex.end() &&
                it11 != gridToVertex.end() && it01 != gridToVertex.end()) {

                int v0 = it00->second;
                int v1 = it10->second;
                int v2 = it11->second;
                int v3 = it01->second;

                // Skip degenerate quads (vertices merged due to seams)
                if (v0 != v1 && v1 != v2 && v2 != v3 && v3 != v0 && v0 != v2 && v1 != v3) {
                    quadFaces.push_back(Vector4i(v0, v1, v2, v3));
                }
            }
        }
    }

    cout << "  Created " << quadFaces.size() << " quads" << endl;

    if (quadVertices.empty() || quadFaces.empty()) {
        cerr << "  No quads extracted" << endl;
        return false;
    }

    // Remove unused vertices and reindex
    set<int> usedVertices;
    for (const auto& face : quadFaces) {
        for (int i = 0; i < 4; i++) {
            usedVertices.insert(face(i));
        }
    }

    map<int, int> oldToNew;
    vector<Vector3d> finalVertices;
    for (int oldIdx : usedVertices) {
        oldToNew[oldIdx] = finalVertices.size();
        finalVertices.push_back(quadVertices[oldIdx]);
    }

    V_quad.resize(finalVertices.size(), 3);
    for (size_t i = 0; i < finalVertices.size(); i++) {
        V_quad.row(i) = finalVertices[i].transpose();
    }

    F_quad.resize(quadFaces.size(), 4);
    for (size_t i = 0; i < quadFaces.size(); i++) {
        F_quad(i, 0) = oldToNew[quadFaces[i](0)];
        F_quad(i, 1) = oldToNew[quadFaces[i](1)];
        F_quad(i, 2) = oldToNew[quadFaces[i](2)];
        F_quad(i, 3) = oldToNew[quadFaces[i](3)];
    }

    // Analyze vertex valences
    map<int, int> valenceCount;
    for (int i = 0; i < F_quad.rows(); i++) {
        for (int j = 0; j < 4; j++) {
            valenceCount[F_quad(i, j)]++;
        }
    }

    int regular = 0, valence3 = 0, valence5 = 0, boundary = 0, other = 0;
    for (const auto& vc : valenceCount) {
        if (vc.second == 4) regular++;
        else if (vc.second == 3) valence3++;
        else if (vc.second == 5) valence5++;
        else if (vc.second < 3) boundary++;
        else other++;
    }

    cout << "  Final: " << V_quad.rows() << " vertices, " << F_quad.rows() << " quads" << endl;
    cout << "  Vertex valences: regular(4)=" << regular << ", valence-3=" << valence3
         << ", valence-5=" << valence5 << ", boundary=" << boundary;
    if (other > 0) cout << ", other=" << other;
    cout << endl;

    return true;
}

// ---------------------------------------------------------------------------
// Direction-field input
// ---------------------------------------------------------------------------

// Read a plain "vec" file: one vector per line, "vx vy vz". Blank lines and
// lines starting with '#' are ignored, so files can carry comments.
bool readVecFile(const string& filename, MatrixXd& M)
{
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: could not open " << filename << endl;
        return false;
    }

    vector<array<double, 3>> rows;
    string line;
    int lineNo = 0;
    while (getline(file, line)) {
        lineNo++;
        // Strip a trailing comment and surrounding whitespace.
        const size_t hash = line.find('#');
        if (hash != string::npos) line = line.substr(0, hash);
        istringstream ss(line);
        array<double, 3> v{};
        if (!(ss >> v[0])) continue;              // blank / comment-only line
        if (!(ss >> v[1] >> v[2])) {
            cerr << "Error: " << filename << ":" << lineNo
                 << " expected three numbers per line (vx vy vz)." << endl;
            return false;
        }
        rows.push_back(v);
    }

    if (rows.empty()) {
        cerr << "Error: " << filename << " contained no vectors." << endl;
        return false;
    }

    M.resize((int)rows.size(), 3);
    for (size_t i = 0; i < rows.size(); i++)
        M.row((int)i) << rows[i][0], rows[i][1], rows[i][2];
    return true;
}

// Read the index file: whitespace-separated integers (the spec puts them on one
// line, but any whitespace works). '#' comments allowed.
bool readIndexFile(const string& filename, vector<int>& idx)
{
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: could not open " << filename << endl;
        return false;
    }
    idx.clear();
    string line;
    while (getline(file, line)) {
        const size_t hash = line.find('#');
        if (hash != string::npos) line = line.substr(0, hash);
        istringstream ss(line);
        long long v;
        while (ss >> v) idx.push_back((int)v);
    }
    if (idx.empty()) {
        cerr << "Error: " << filename << " contained no indices." << endl;
        return false;
    }
    return true;
}

// Load D1/D2 in either format, orienting to #rows x 3.
bool loadDirections(const string& file, bool asVec, MatrixXd& out)
{
    MatrixXd raw;
    if (asVec) {
        if (!readVecFile(file, raw)) return false;
    } else {
        if (!igl::readDMAT(file, raw)) {
            cerr << "Error: could not read DMAT " << file << endl;
            return false;
        }
        // DMAT is often stored transposed (3 x n).
        if (raw.rows() == 3 && raw.cols() != 3) raw.transposeInPlace();
    }
    if (raw.cols() != 3) {
        cerr << "Error: " << file << " must have 3 columns, got " << raw.cols() << endl;
        return false;
    }
    out = raw;
    return true;
}

#ifdef HAVE_LIBQEX
// Quad extraction via libQEx (Ebke et al. 2013, "QEx: Robust Quad Mesh Extraction").
//
// QEx consumes an integer-grid map given as per-triangle-corner UVs, which is
// exactly the representation MIQ produces: V_uv holds per-corner UVs and
// F_uv(f,k) indexes the corner of face f. Seam corners already carry distinct
// UVs there, so no seam reconstruction is needed on our side -- we hand QEx the
// original geometry (V, F) plus the corner UVs and let it do the rest.
//
// QEx is robust to the imperfections real MIQ output has (near-integer rather
// than exactly-integer transitions, small fold-overs), which is the reason to
// prefer it over the hand-written grid extractor.
// Quad extraction via libQEx (Ebke et al. 2013, "QEx: Robust Quad Mesh Extraction").
//
// QEx consumes an integer-grid map given as per-triangle-corner UVs, which is
// exactly the representation MIQ produces: V_uv holds per-corner UVs and
// F_uv(f,k) indexes the corner of face f. Seam corners already carry distinct
// UVs there, so no seam reconstruction is needed on our side.
//
// We call the C++ entry point extractQuadMeshOM() rather than the C API
// qex_extractQuadMesh(). The C wrapper keeps ONLY 4-valent faces -- it skips
// everything else with a `continue` -- and QEx legitimately returns n-gons for
// relaxed integer-grid maps (its own header says so). Those skipped faces were
// showing up as small holes in the output. Going through OpenMesh directly
// keeps every face, so D_quad carries the per-face degree.
bool extractQuadMeshQEx(const MatrixXd& V_3d, const MatrixXi& F_tri,
                        const MatrixXd& UV, const MatrixXi& F_UV,
                        MatrixXd& V_quad, MatrixXi& F_quad, VectorXi& D_quad)
{
    cout << "Extracting quad mesh with libQEx..." << endl;

    if (UV.rows() == 0 || F_UV.rows() != F_tri.rows()) {
        cerr << "  Invalid UV map for QEx (UV rows=" << UV.rows()
             << ", F_UV rows=" << F_UV.rows() << ", F rows=" << F_tri.rows() << ")" << endl;
        return false;
    }

    cout << "  UV range: u=[" << UV.col(0).minCoeff() << ", " << UV.col(0).maxCoeff()
         << "], v=[" << UV.col(1).minCoeff() << ", " << UV.col(1).maxCoeff() << "]" << endl;

    // Build the OpenMesh triangle mesh QEx expects.
    QEx::TriMesh triMesh;
    triMesh.reserve(V_3d.rows(), V_3d.rows() + F_tri.rows(), F_tri.rows());

    vector<QEx::TriMesh::VertexHandle> vh(V_3d.rows());
    for (int i = 0; i < V_3d.rows(); i++)
        vh[i] = triMesh.add_vertex(QEx::TriMesh::Point(V_3d(i, 0), V_3d(i, 1), V_3d(i, 2)));

    vector<QEx::TriMesh::FaceHandle> fh(F_tri.rows());
    for (int f = 0; f < F_tri.rows(); f++)
        fh[f] = triMesh.add_face(vh[F_tri(f, 0)], vh[F_tri(f, 1)], vh[F_tri(f, 2)]);

    // Per-halfedge UVs, in the same order OpenMesh iterates each face's halfedges.
    vector<OpenMesh::Vec2d> uvs(triMesh.n_halfedges());
    for (int f = 0; f < F_tri.rows(); f++) {
        if (!fh[f].is_valid()) continue;
        int k = 0;
        for (auto fh_it = triMesh.fh_begin(fh[f]); fh_it != triMesh.fh_end(fh[f]) && k < 3; ++fh_it, ++k)
            uvs[fh_it->idx()] = OpenMesh::Vec2d(UV(F_UV(f, k), 0), UV(F_UV(f, k), 1));
    }

    // extractQuadMeshOM() requires the status properties (it requests them
    // internally and releases them again in mergePolyToQuad), and it runs its
    // own garbage collection, so the mesh comes back already compacted.
    QEx::QuadMesh quadMesh;
    QEx::extractQuadMeshOM(&triMesh, &uvs, nullptr, &quadMesh);

    if (quadMesh.n_faces() == 0) {
        cerr << "  QEx produced no faces." << endl;
        return false;
    }

    // Copy out every face, whatever its degree.
    vector<vector<int>> faces;
    faces.reserve(quadMesh.n_faces());
    int maxDeg = 0;
    for (auto f_it = quadMesh.faces_begin(); f_it != quadMesh.faces_end(); ++f_it) {
        vector<int> face;
        for (auto fv_it = quadMesh.fv_begin(*f_it); fv_it != quadMesh.fv_end(*f_it); ++fv_it)
            face.push_back(fv_it->idx());
        if (face.size() < 3) continue;            // degenerate, nothing to emit
        bool dup = false;                          // repeated corner => not a usable face
        for (size_t a = 0; a < face.size() && !dup; a++)
            for (size_t b = a + 1; b < face.size() && !dup; b++)
                if (face[a] == face[b]) dup = true;
        if (dup) continue;
        maxDeg = max(maxDeg, (int)face.size());
        faces.push_back(std::move(face));
    }

    if (faces.empty()) {
        cerr << "  QEx returned no usable faces." << endl;
        return false;
    }

    V_quad.resize(quadMesh.n_vertices(), 3);
    {
        int i = 0;
        for (auto v_it = quadMesh.vertices_begin(); v_it != quadMesh.vertices_end(); ++v_it, ++i) {
            const QEx::QuadMesh::Point p = quadMesh.point(*v_it);
            V_quad.row(i) << p[0], p[1], p[2];
        }
    }

    F_quad.setConstant((int)faces.size(), maxDeg, -1);
    D_quad.resize((int)faces.size());
    for (size_t i = 0; i < faces.size(); i++) {
        D_quad((int)i) = (int)faces[i].size();
        for (size_t k = 0; k < faces[i].size(); k++)
            F_quad((int)i, (int)k) = faces[i][k];
    }

    // Drop vertices no surviving face references, and reindex.
    {
        vector<int> oldToNew(V_quad.rows(), -1);
        int next = 0;
        for (int i = 0; i < F_quad.rows(); i++)
            for (int k = 0; k < D_quad(i); k++)
                if (oldToNew[F_quad(i, k)] < 0) oldToNew[F_quad(i, k)] = next++;
        if (next < V_quad.rows()) {
            MatrixXd Vc(next, 3);
            for (int i = 0; i < V_quad.rows(); i++)
                if (oldToNew[i] >= 0) Vc.row(oldToNew[i]) = V_quad.row(i);
            V_quad = Vc;
            for (int i = 0; i < F_quad.rows(); i++)
                for (int k = 0; k < D_quad(i); k++)
                    F_quad(i, k) = oldToNew[F_quad(i, k)];
        }
    }

    // Face-degree and valence statistics.
    map<int, int> degCount, valenceCount;
    for (int i = 0; i < F_quad.rows(); i++) {
        degCount[D_quad(i)]++;
        for (int k = 0; k < D_quad(i); k++) valenceCount[F_quad(i, k)]++;
    }

    int regular = 0, valence3 = 0, valence5 = 0, boundary = 0, other = 0;
    for (const auto& vc : valenceCount) {
        if (vc.second == 4) regular++;
        else if (vc.second == 3) valence3++;
        else if (vc.second == 5) valence5++;
        else if (vc.second < 3) boundary++;
        else other++;
    }

    cout << "  Final: " << V_quad.rows() << " vertices, " << F_quad.rows() << " faces (";
    bool first = true;
    for (const auto& dc : degCount) {
        if (!first) cout << ", ";
        cout << dc.second << "x" << dc.first << "-gon";
        first = false;
    }
    cout << ")" << endl;
    cout << "  Vertex valences: regular(4)=" << regular << ", valence-3=" << valence3
         << ", valence-5=" << valence5 << ", boundary=" << boundary;
    if (other > 0) cout << ", other=" << other;
    cout << endl;

    return true;
}
#endif // HAVE_LIBQEX

// Helper function to run MIQ with direct or integration method
bool runMIQ(const MatrixXd& V_def, const MatrixXi& F,
            const MatrixXd& X1_combed, const MatrixXd& X2_combed,
            const Eigen::Matrix<int, Eigen::Dynamic, 3>& MMatch,
            const Eigen::Matrix<int, Eigen::Dynamic, 1>& isSingularity,
            const Eigen::Matrix<int, Eigen::Dynamic, 3>& Seams,
            MatrixXd& UV, MatrixXi& FUV,
            double gradient_size, double stiffness,
            bool use_direct_round)
{
    cout << "  Attempting MIQ with gradient_size=" << gradient_size
         << ", stiffness=" << stiffness
         << ", method=" << (use_direct_round ? "direct" : "integration") << endl;

    if (use_direct_round) {
        cout << "Direct (skip integer solver)===================" << endl;
        igl::copyleft::comiso::miq(
            V_def, F,
            X1_combed, X2_combed,
            MMatch, isSingularity, Seams,
            UV, FUV,
            gradient_size,
            stiffness,
            true,      // direct_round = TRUE (skips integer solver)
            15,        // iterations
            5,         // local_iter
            true,     // do_round
            false);    // singularityRound
    } else {
        cout << "Integration (integer solver)===================" << endl;
        igl::copyleft::comiso::miq(
            V_def, F,
            X1_combed, X2_combed,
            MMatch, isSingularity, Seams,
            UV, FUV,
            gradient_size,
            stiffness,
            false,     // direct_round = FALSE (use integer solver)
            15,        // iterations
            5,         // local_iter
            true,      // do_round
            false);    // singularityRound
    }

    return true;
}

int main(int argc, char *argv[])
{
    using namespace Eigen;
    using namespace std;

    // ---------------------------------------------------------------------
    // Command line
    //
    //   quadRemesher mesh.obj D1 D2 [frame_indices] [options]
    //
    // Options are flags (--type, --defined_on, --out, --name) plus the older
    // bare keywords (direct/integration, grid/qex, batch, ...), which still
    // work so existing scripts keep running.
    // ---------------------------------------------------------------------
    auto usage = [&]() {
        cout << "Usage: ./quadRemesher <mesh.obj> <D1> <D2> [frame_indices] [options]" << endl;
        cout << endl;
        cout << "  D1, D2            per-direction files (see --type). One row per element:" << endl;
        cout << "                    all elements of the mesh, or -- if an index file is" << endl;
        cout << "                    given -- one row per listed index, in that order." << endl;
        cout << "  frame_indices     optional; whitespace-separated indices of the faces or" << endl;
        cout << "                    vertices the rows of D1/D2 refer to. Omit to mean all." << endl;
        cout << endl;
        cout << "  --type DMAT|vec   input format (default DMAT). 'vec' is one 'vx vy vz'" << endl;
        cout << "                    per line." << endl;
        cout << "  --defined_on vertices|faces" << endl;
        cout << "                    where the directions live (default vertices). Vertex" << endl;
        cout << "                    fields are averaged onto each face barycenter and" << endl;
        cout << "                    projected into the face plane; face fields are used" << endl;
        cout << "                    as given (still projected into the face plane)." << endl;
        cout << "  --out <folder>    output folder      --name <name>  output basename" << endl;
        cout << endl;
        cout << "  method:    'direct' (default, skip integer solver) or 'integration'" << endl;
        cout << "  extractor: 'grid' (default, built-in) or 'qex' (libQEx, implies 'integration')" << endl;
        cout << "  batch:     extract, save and exit without opening the viewer" << endl;
        cout << "  dumpfields:        write the intermediate fields as DMAT" << endl;
        cout << "  arbitrary[=x,y,z]  ignore D1/D2; constrain to an arbitrary 4-RoSy instead" << endl;
        cout << "  constraints=<f>    keep only fraction f of the constraint faces (default 1.0)" << endl;
    };

    if (argc < 4) { usage(); return 1; }

    bool use_direct_round = true;
    bool method_explicit = false;
    bool use_qex = false;
    bool batch_mode = false;
    bool dump_fields = false;
    string field_dump_prefix = "fields";
    double constraint_fraction = 1.0;
    bool arbitrary_field = false;
    Vector3d arbitrary_dir(1.0, 0.0, 0.0);

    bool input_is_vec = false;          // --type
    bool defined_on_faces = false;      // --defined_on
    string frame_index_file;            // optional 4th positional
    string out_folder;                  // --out  / 4th-5th positional (legacy)
    string out_name;                    // --name

    // Keywords that may appear as bare positional arguments; anything else in
    // positional slot 4/5 is treated as the legacy [output_folder] [name].
    auto isKeyword = [](const string& a) {
        return a == "direct" || a == "integration" || a == "qex" || a == "grid" ||
               a == "batch" || a == "dumpfields" || a == "arbitrary" ||
               a.rfind("arbitrary=", 0) == 0 || a.rfind("constraints=", 0) == 0 ||
               a.rfind("--", 0) == 0;
    };

    int positional = 0;                 // counts non-keyword args after argv[3]
    for (int i = 4; i < argc; i++) {
        const string arg = argv[i];

        if (arg == "--type" || arg == "--defined_on" || arg == "--out" || arg == "--name") {
            if (i + 1 >= argc) {
                cerr << "Error: " << arg << " needs a value." << endl;
                return 1;
            }
            const string val = argv[++i];
            if (arg == "--type") {
                if (val == "vec" || val == "VEC") input_is_vec = true;
                else if (val == "DMAT" || val == "dmat") input_is_vec = false;
                else { cerr << "Error: --type must be DMAT or vec, got '" << val << "'" << endl; return 1; }
            } else if (arg == "--defined_on") {
                if (val == "faces" || val == "face") defined_on_faces = true;
                else if (val == "vertices" || val == "vertex") defined_on_faces = false;
                else { cerr << "Error: --defined_on must be vertices or faces, got '" << val << "'" << endl; return 1; }
            } else if (arg == "--out") {
                out_folder = val;
            } else {
                out_name = val;
            }
            continue;
        }

        if (arg == "--help" || arg == "-h") { usage(); return 0; }

        if (arg == "integration") {
            use_direct_round = false; method_explicit = true;
            cout << "Using integration method (integer solver)" << endl;
        } else if (arg == "direct") {
            use_direct_round = true; method_explicit = true;
            cout << "Using direct method (skip integer solver)" << endl;
        } else if (arg == "qex") {
            use_qex = true;
        } else if (arg == "grid") {
            use_qex = false;
        } else if (arg == "batch") {
            batch_mode = true;
        } else if (arg == "dumpfields") {
            dump_fields = true;
        } else if (arg == "arbitrary") {
            arbitrary_field = true;
        } else if (arg.rfind("arbitrary=", 0) == 0) {
            arbitrary_field = true;
            const string v = arg.substr(10);
            double c[3] = {1.0, 0.0, 0.0};
            size_t start = 0;
            for (int k = 0; k < 3 && start <= v.size(); k++) {
                size_t comma = v.find(',', start);
                const string tok = v.substr(start, comma == string::npos ? string::npos : comma - start);
                if (!tok.empty()) { try { c[k] = stod(tok); } catch (const exception&) {} }
                if (comma == string::npos) break;
                start = comma + 1;
            }
            arbitrary_dir = Vector3d(c[0], c[1], c[2]);
            if (arbitrary_dir.norm() < 1e-12) arbitrary_dir = Vector3d(1, 0, 0);
        } else if (arg.rfind("constraints=", 0) == 0) {
            try {
                constraint_fraction = stod(arg.substr(12));
            } catch (const exception&) {
                cerr << "Could not parse '" << arg << "'; expected e.g. constraints=0.1" << endl;
                return 1;
            }
            if (!(constraint_fraction > 0.0) || constraint_fraction > 1.0) {
                cerr << "constraints= must be in (0, 1], got " << constraint_fraction << endl;
                return 1;
            }
        } else if (!isKeyword(arg)) {
            // Positional. The first is the frame index file if it exists on
            // disk; otherwise fall back to the legacy [output_folder] [name].
            positional++;
            if (positional == 1) {
                // A readable *file* here is the frame index list; a directory
                // (or a non-existent path) is the legacy [output_folder].
                struct stat st{};
                const bool isFile = (stat(arg.c_str(), &st) == 0) && S_ISREG(st.st_mode);
                if (isFile) frame_index_file = arg;
                else out_folder = arg;
            } else if (positional == 2 && out_folder.empty()) {
                out_folder = arg;
            } else if (out_name.empty()) {
                out_name = arg;
            }
        } else {
            cerr << "Unknown option '" << arg << "'" << endl;
            return 1;
        }
    }

    // Field dumps land next to the quad mesh, under the output folder if given.
    if (dump_fields && !out_folder.empty()) {
        field_dump_prefix = out_folder + (out_folder.back() == '/' ? "" : "/") + "fields";
    }

#ifndef HAVE_LIBQEX
    if (use_qex) {
        cerr << "This binary was built without libQEx (USE_LIBQEX=OFF)." << endl;
        cerr << "Reconfigure with -DUSE_LIBQEX=ON to use the 'qex' extractor." << endl;
        return 1;
    }
#endif

    // libQEx consumes an integer-grid map. MIQ only produces one when the
    // integer solver runs, i.e. direct_round=false. Force it, and say so if the
    // user explicitly asked for 'direct'.
    if (use_qex && use_direct_round) {
        if (method_explicit) {
            cout << "NOTE: 'qex' requires an integer-grid map; overriding 'direct'"
                    " with 'integration'." << endl;
        } else {
            cout << "Using integration method (required by the 'qex' extractor)" << endl;
        }
        use_direct_round = false;
    }

    cout << "Extractor: " << (use_qex ? "libQEx" : "built-in grid") << endl;

    // 1. Load Mesh
    MatrixXd V;
    MatrixXi F;
    MatrixXd B;
    MatrixXd B_deformed;

    igl::readOBJ(argv[1], V, F);
    cout << "Loaded mesh: " << V.rows() << " vertices, " << F.rows() << " faces" << endl;

    // Compute face barycenters
    igl::barycenter(V, F, B);

    // Compute average edge length for cross field visualization scale
    double avg_edge_len = igl::avg_edge_length(V, F);
    global_scale = avg_edge_len * 0.4;

    // 2. Load D1 and D2. Rows correspond either to every element of the mesh,
    //    or -- when a frame index file is given -- to the listed indices in
    //    order. Elements are vertices or faces depending on --defined_on.
    MatrixXd D1_in, D2_in;
    if (!loadDirections(argv[2], input_is_vec, D1_in) ||
        !loadDirections(argv[3], input_is_vec, D2_in)) {
        return 1;
    }
    if (D1_in.rows() != D2_in.rows()) {
        cerr << "Error: D1 has " << D1_in.rows() << " rows but D2 has "
             << D2_in.rows() << "." << endl;
        return 1;
    }

    const int numElements = defined_on_faces ? (int)F.rows() : (int)V.rows();
    const char* elementName = defined_on_faces ? "face" : "vertex";
    const char* elementPlural = defined_on_faces ? "faces" : "vertices";

    vector<int> frameIdx;
    if (!frame_index_file.empty()) {
        if (!readIndexFile(frame_index_file, frameIdx)) return 1;
        if ((int)frameIdx.size() != D1_in.rows()) {
            cerr << "Error: " << frame_index_file << " lists " << frameIdx.size()
                 << " indices but D1/D2 have " << D1_in.rows() << " rows." << endl;
            return 1;
        }
        for (int id : frameIdx) {
            if (id < 0 || id >= numElements) {
                cerr << "Error: index " << id << " in " << frame_index_file
                     << " is out of range (mesh has " << numElements << " "
                     << (numElements == 1 ? elementName : elementPlural) << ")." << endl;
                return 1;
            }
        }
    } else {
        // No index file: the rows must cover every element, in order.
        if (D1_in.rows() != numElements) {
            cerr << "Error: D1/D2 have " << D1_in.rows() << " rows but the mesh has "
                 << numElements << " " << (numElements == 1 ? elementName : elementPlural)
                 << ". Provide a frame index file to specify a subset." << endl;
            return 1;
        }
        frameIdx.resize(numElements);
        for (int i = 0; i < numElements; i++) frameIdx[i] = i;
    }

    cout << "Loaded " << D1_in.rows() << " direction pairs defined on "
         << elementPlural
         << " (" << (input_is_vec ? "vec" : "DMAT") << " format)." << endl;
    cout << "Mesh has " << V.rows() << " vertices, " << F.rows() << " faces." << endl;

    // Scatter the supplied rows into full per-element arrays, tracking which
    // elements actually received a direction.
    MatrixXd D1_elem = MatrixXd::Zero(numElements, 3);
    MatrixXd D2_elem = MatrixXd::Zero(numElements, 3);
    vector<bool> hasDir(numElements, false);
    for (size_t r = 0; r < frameIdx.size(); r++) {
        D1_elem.row(frameIdx[r]) = D1_in.row((int)r);
        D2_elem.row(frameIdx[r]) = D2_in.row((int)r);
        hasDir[frameIdx[r]] = true;
    }

    // 3. Bring the field to a per-face frame.
    //
    //    vertices: average over the face's three vertices (i.e. evaluate at the
    //              barycenter) and project into the face plane. A face is only
    //              constrained if all three of its vertices carry a direction.
    //    faces:    already per face; still projected into the face plane so the
    //              frame is tangent even if the input was not exactly so.
    int numFaces = F.rows();
    MatrixXd bc1 = MatrixXd::Zero(numFaces, 3);
    MatrixXd bc2 = MatrixXd::Zero(numFaces, 3);
    vector<bool> faceHasDir(numFaces, false);

    for (int fi = 0; fi < numFaces; fi++) {
        const int v0 = F(fi, 0), v1 = F(fi, 1), v2 = F(fi, 2);
        const Vector3d e1 = V.row(v1) - V.row(v0);
        const Vector3d e2 = V.row(v2) - V.row(v0);
        const Vector3d n = e1.cross(e2).normalized();

        Vector3d d1, d2;
        if (defined_on_faces) {
            if (!hasDir[fi]) continue;
            d1 = D1_elem.row(fi);
            d2 = D2_elem.row(fi);
        } else {
            if (!hasDir[v0] || !hasDir[v1] || !hasDir[v2]) continue;
            d1 = (D1_elem.row(v0) + D1_elem.row(v1) + D1_elem.row(v2)) / 3.0;
            d2 = (D2_elem.row(v0) + D2_elem.row(v1) + D2_elem.row(v2)) / 3.0;
        }

        // Project onto the tangent plane
        d1 = d1 - d1.dot(n) * n;
        d2 = d2 - d2.dot(n) * n;

        bc1.row(fi) = d1.normalized();
        bc2.row(fi) = d2.normalized();
        faceHasDir[fi] = true;
    }

    {
        const int constrained = (int)count(faceHasDir.begin(), faceHasDir.end(), true);
        if (defined_on_faces)
            cout << "Using " << constrained << " / " << numFaces << " per-face directions." << endl;
        else
            cout << "Interpolated to " << constrained << " / " << numFaces
                 << " per-face directions." << endl;
        if (constrained == 0) {
            cerr << "Error: no face received a direction." << endl;
            return 1;
        }
    }

    // 5. Filter constraints: only keep faces with valid directions
    MatrixXd FN;
    igl::per_face_normals(V, F, FN);

    vector<int> good_faces;
    for (int fi = 0; fi < numFaces; fi++) {
        if (!faceHasDir[fi]) continue;   // no direction supplied for this face
        Vector3d d1 = bc1.row(fi);
        Vector3d d2 = bc2.row(fi);
        Vector3d n = FN.row(fi);

        double norm1 = d1.norm();
        double norm2 = d2.norm();

        if (norm1 < 0.1 || norm2 < 0.1) continue;

        Vector3d d1_proj = d1 - d1.dot(n) * n;
        Vector3d d2_proj = d2 - d2.dot(n) * n;

        if (d1_proj.norm() < 0.1 || d2_proj.norm() < 0.1) continue;

        d1_proj.normalize();
        d2_proj.normalize();

        bc1.row(fi) = d1_proj;
        bc2.row(fi) = d2_proj;
        good_faces.push_back(fi);
    }

    cout << "Valid constraint faces: " << good_faces.size() << " / " << numFaces << endl;

    // Optionally keep only a fraction of the constraints, which is how libigl
    // tutorial 506 drives this: a *sparse* set of frames defines the desired
    // quad shape and comiso::frame_field interpolates over the rest. With every
    // face constrained the solver has no freedom and returns its input, so a
    // fraction < 1 is what actually exercises the interpolation.
    //
    // Constraints are spread over the surface rather than taken at random:
    // farthest-point sampling on face barycenters, so the kept faces cover the
    // mesh evenly instead of clumping and leaving large unconstrained regions.
    if (constraint_fraction < 1.0 && good_faces.size() > 2) {
        const int target = max(1, (int)llround(good_faces.size() * constraint_fraction));
        if (target < (int)good_faces.size()) {
            vector<int> picked;
            picked.reserve(target);
            vector<double> dist(good_faces.size(), numeric_limits<double>::max());

            int current = 0;  // deterministic seed point
            for (int k = 0; k < target; k++) {
                picked.push_back(good_faces[current]);
                const RowVector3d p = B.row(good_faces[current]);
                int farthest = -1;
                double farthestDist = -1.0;
                for (size_t i = 0; i < good_faces.size(); i++) {
                    const double d = (B.row(good_faces[i]) - p).squaredNorm();
                    if (d < dist[i]) dist[i] = d;
                    if (dist[i] > farthestDist) { farthestDist = dist[i]; farthest = (int)i; }
                }
                if (farthest < 0) break;
                current = farthest;
            }
            sort(picked.begin(), picked.end());
            good_faces.swap(picked);
            cout << "  Thinned to " << good_faces.size() << " constraint faces ("
                 << constraint_fraction * 100.0 << "% of valid, farthest-point sampled)" << endl;
        }
    }

    // Optionally discard the curvature directions and constrain the kept faces
    // to an arbitrary 4-RoSy instead. A 4-RoSy has no distinguished axis, so any
    // unit tangent direction is as valid a constraint as a principal one; this
    // tests how much the curvature data actually determines the result versus
    // the surface topology and the smoother.
    //
    // The reference direction is a fixed world vector projected into each face
    // plane, so the constraints are mutually consistent (a "combed" global
    // direction) rather than random per face.
    if (arbitrary_field) {
        const Vector3d worldDir = arbitrary_dir.normalized();
        int flipped = 0;
        for (int fi : good_faces) {
            Vector3d n = FN.row(fi);
            n.normalize();
            Vector3d d = worldDir - worldDir.dot(n) * n;
            if (d.norm() < 1e-6) {
                // Face is perpendicular to the reference: fall back to any tangent.
                Vector3d axis = (fabs(n.x()) < 0.9) ? Vector3d::UnitX() : Vector3d::UnitY();
                d = n.cross(axis);
                flipped++;
            }
            d.normalize();
            bc1.row(fi) = d.transpose();
            bc2.row(fi) = n.cross(d).normalized().transpose();
        }
        cout << "  Using an ARBITRARY 4-RoSy field (world dir ["
             << worldDir.transpose() << "]) instead of principal directions";
        if (flipped) cout << "; " << flipped << " faces fell back to a local tangent";
        cout << endl;
    }

    // Create b with only good faces
    VectorXi b(good_faces.size());
    MatrixXd bc1_filtered(good_faces.size(), 3);
    MatrixXd bc2_filtered(good_faces.size(), 3);
    for (size_t i = 0; i < good_faces.size(); i++) {
        b(i) = good_faces[i];
        bc1_filtered.row(i) = bc1.row(good_faces[i]);
        bc2_filtered.row(i) = bc2.row(good_faces[i]);
    }

    cout << "Interpolating Frame Field from " << good_faces.size() << " constraints..." << endl;
    MatrixXd FF1, FF2;
    igl::copyleft::comiso::frame_field(V, F, b, bc1_filtered, bc2_filtered, FF1, FF2);

    // Optional debug dump of the direction fields, for offline visualization.
    // Written at the stage where each quantity is defined:
    //   *_bary.dmat  face barycenters
    //   *_face*.dmat bc1/bc2, the per-vertex D1/D2 averaged onto the barycenter
    //                and projected into the face plane (pipeline stage 1)
    //   *_ff*.dmat   FF1/FF2, the smooth frame field interpolated from those
    //                constraints (stage 2)
    //   *_constrained.dmat  1 per face if it survived constraint filtering
    if (dump_fields) {
        string base = field_dump_prefix;
        VectorXd constrained = VectorXd::Zero(F.rows());
        for (int fi : good_faces) constrained(fi) = 1.0;

        bool ok = igl::writeDMAT(base + "_bary.dmat", B, true)
               && igl::writeDMAT(base + "_face1.dmat", bc1, true)
               && igl::writeDMAT(base + "_face2.dmat", bc2, true)
               && igl::writeDMAT(base + "_ff1.dmat", FF1, true)
               && igl::writeDMAT(base + "_ff2.dmat", FF2, true)
               && igl::writeDMAT(base + "_constrained.dmat", constrained, true);
        if (ok) cout << "  Dumped field data to " << base << "_*.dmat" << endl;
        else    cerr << "  Warning: failed to write field dump to " << base << "_*.dmat" << endl;
    }

    cout << "Deforming Mesh for Anisotropy..." << endl;
    MatrixXd V_deformed, FF1_def, FF2_def;
    igl::frame_field_deformer(V, F, FF1, FF2, V_deformed, FF1_def, FF2_def);

    igl::barycenter(V_deformed, F, B_deformed);

    cout << "Converting to Cross Field..." << endl;
    MatrixXd X1_def;
    igl::frame_to_cross_field(V_deformed, F, FF1_def, FF2_def, X1_def);

    // Recompute local basis on deformed mesh
    MatrixXd B1_def, B2_def, B3_def;
    igl::local_basis(V_deformed, F, B1_def, B2_def, B3_def);

    // Filter constraints on deformed mesh
    vector<int> good_faces_def;
    MatrixXd X1_proj_def = X1_def;
    for (int fi = 0; fi < numFaces; fi++) {
        Vector3d x1 = X1_def.row(fi);
        Vector3d n = B3_def.row(fi);

        Vector3d x1_proj = x1 - x1.dot(n) * n;
        if (x1_proj.norm() < 0.1) continue;

        x1_proj.normalize();
        X1_proj_def.row(fi) = x1_proj;
        good_faces_def.push_back(fi);
    }

    cout << "Valid deformed constraints: " << good_faces_def.size() << " / " << numFaces << endl;

    VectorXi b_def(good_faces_def.size());
    MatrixXd bc_x(good_faces_def.size(), 3);
    for (size_t i = 0; i < good_faces_def.size(); i++) {
        b_def(i) = good_faces_def[i];
        bc_x.row(i) = X1_proj_def.row(good_faces_def[i]);
    }

    cout << "Running nrosy with " << good_faces_def.size() << " constraints..." << endl;
    VectorXd S;
    igl::copyleft::comiso::nrosy(
             V_deformed,
             F,
             b_def,
             bc_x,
             VectorXi(),
             VectorXd(),
             MatrixXd(),
             4,
             0.5,
             X1_def,
             S);

    // Check singularities from nrosy
    cout << "Singularities from nrosy (S field):" << endl;
    int nrosy_sing_count = 0;
    for (int i = 0; i < S.size(); i++) {
        if (abs(S(i)) > 0.01) {
            cout << "  Vertex " << i << ": S=" << S(i) << endl;
            nrosy_sing_count++;
        }
    }
    cout << "  Total nrosy singularities: " << nrosy_sing_count << endl;

    // Project X1_def onto tangent plane and normalize
    MatrixXd B1, B2, B3;
    igl::local_basis(V_deformed, F, B1, B2, B3);

    for (int i = 0; i < F.rows(); i++) {
        Vector3d x1 = X1_def.row(i);
        Vector3d n = B3.row(i);
        Vector3d x1_proj = x1 - x1.dot(n) * n;
        X1_def.row(i) = x1_proj.normalized();
    }

    // Compute X2 as 90 degree rotation in the tangent plane
    MatrixXd X2_def = igl::rotate_vectors(X1_def, VectorXd::Constant(F.rows(), igl::PI/2), B1, B2);

    // MIQ gradient size (controls quad density - higher = coarser)
    double miq_gradient_size = 40.0;
    double miq_stiffness = 10.0;

    // === Diagnostic checks on cross field ===
    cout << "\n=== Cross field diagnostics ===" << endl;
    cout << "  X1_def size: " << X1_def.rows() << " x " << X1_def.cols() << endl;
    cout << "  X2_def size: " << X2_def.rows() << " x " << X2_def.cols() << endl;

    int nan_count_x1 = (X1_def.array() != X1_def.array()).count();
    int nan_count_x2 = (X2_def.array() != X2_def.array()).count();
    cout << "  NaN in X1_def: " << nan_count_x1 << endl;
    cout << "  NaN in X2_def: " << nan_count_x2 << endl;

    VectorXd norms1 = X1_def.rowwise().norm();
    VectorXd norms2 = X2_def.rowwise().norm();
    cout << "  X1_def norm range: [" << norms1.minCoeff() << ", " << norms1.maxCoeff() << "]" << endl;
    cout << "  X2_def norm range: [" << norms2.minCoeff() << ", " << norms2.maxCoeff() << "]" << endl;

    VectorXd dots(F.rows());
    for (int i = 0; i < F.rows(); i++) {
        dots(i) = X1_def.row(i).dot(X2_def.row(i));
    }
    cout << "  X1·X2 dot product range: [" << dots.minCoeff() << ", " << dots.maxCoeff() << "]" << endl;

    MatrixXd N;
    igl::per_face_normals(V_deformed, F, N);
    VectorXd tangent1(F.rows()), tangent2(F.rows());
    for (int i = 0; i < F.rows(); i++) {
        tangent1(i) = abs(X1_def.row(i).dot(N.row(i)));
        tangent2(i) = abs(X2_def.row(i).dot(N.row(i)));
    }
    cout << "  X1·N (should be ~0): max = " << tangent1.maxCoeff() << endl;
    cout << "  X2·N (should be ~0): max = " << tangent2.maxCoeff() << endl;

    int bad_faces_count = 0;
    for (int i = 0; i < F.rows(); i++) {
        if (norms1(i) < 0.01 || norms2(i) < 0.01 ||
            abs(dots(i)) > 0.1 || tangent1(i) > 0.1 || tangent2(i) > 0.1) {
            bad_faces_count++;
        }
    }
    cout << "  Problematic faces: " << bad_faces_count << " / " << F.rows() << endl;

    cout << "\n=== Running MIQ Parameterization ===" << endl;

    V_uv = MatrixXd::Zero(V_deformed.rows(), 2);
    F_uv = F;

    // === Full MIQ Pipeline ===

    // 1. Compute bisectors from the cross field
    cout << "  Step 1: Computing bisectors..." << endl;
    MatrixXd BIS1, BIS2;
    igl::compute_frame_field_bisectors(V_deformed, F, X1_def, X2_def, BIS1, BIS2);

    // 2. Comb the bisector field
    cout << "  Step 2: Combing bisector field..." << endl;
    MatrixXd BIS1_combed, BIS2_combed;
    igl::comb_cross_field(V_deformed, F, X1_def, BIS2, BIS1_combed, BIS2_combed);

    // 3. Find the integer mismatches
    cout << "  Step 3: Computing mismatches..." << endl;
    Eigen::Matrix<int, Eigen::Dynamic, 3> MMatch;
    igl::cross_field_mismatch(V_deformed, F, BIS1_combed, BIS2_combed, true, MMatch);
    cout << "    MMatch range: [" << MMatch.minCoeff() << ", " << MMatch.maxCoeff() << "]" << endl;

    // 4. Find the singularities
    cout << "  Step 4: Finding singularities..." << endl;
    Eigen::Matrix<int, Eigen::Dynamic, 1> isSingularity, singularityIndex;
    igl::find_cross_field_singularities(V_deformed, F, MMatch, isSingularity, singularityIndex);
    int numSingularities = isSingularity.sum();
    cout << "    Found " << numSingularities << " singularities" << endl;

    // Analyze singularities (Poincaré-Hopf check)
    int pos_sing = 0, neg_sing = 0;
    cout << "    Singularity details:" << endl;
    for (int i = 0; i < singularityIndex.size(); i++) {
        if (isSingularity(i)) {
            // libigl convention: index 1 = +1/4, index 3 = -1/4
            if (singularityIndex(i) == 1) {
                pos_sing++;
                cout << "      Vertex " << i << ": index=1 (+1/4, positive)" << endl;
            } else if (singularityIndex(i) == 3) {
                neg_sing++;
                cout << "      Vertex " << i << ": index=3 (-1/4, negative)" << endl;
            } else {
                cout << "      Vertex " << i << ": index=" << singularityIndex(i) << " (unusual)" << endl;
            }
        }
    }
    cout << "    Positive singularities: " << pos_sing << ", Negative: " << neg_sing << endl;
    cout << "    Index sum (should be 4 for sphere, 0 for torus): " << (pos_sing - neg_sing) << endl;

    // 5. Cut the mesh from singularities
    cout << "  Step 5: Cutting mesh from singularities..." << endl;
    Eigen::Matrix<int, Eigen::Dynamic, 3> Seams;
    igl::cut_mesh_from_singularities(V_deformed, F, MMatch, Seams);
    cout << "    Seams: " << Seams.rows() << "x" << Seams.cols() << ", total cuts: " << Seams.sum() << endl;

    // 6. Comb the original frame field using the combed bisectors
    cout << "  Step 6: Combing frame field..." << endl;
    MatrixXd X1_combed, X2_combed;
    igl::comb_frame_field(V_deformed, F, X1_def, X2_def, BIS1_combed, BIS2_combed, X1_combed, X2_combed);

    // Optional dump of the field as it is conditioned for integration. The
    // combing stages are what make the field integrable: they resolve the
    // 4-RoSy rotational ambiguity into a consistent branch across the seams,
    // so the quantity MIQ actually integrates is X1_combed, not the raw
    // interpolated constraints.
    if (dump_fields) {
        string base = field_dump_prefix;
        MatrixXd B_def_dump;
        igl::barycenter(V_deformed, F, B_def_dump);
        VectorXd seamFlag = VectorXd::Zero(F.rows());
        for (int i = 0; i < Seams.rows(); i++)
            for (int j = 0; j < 3; j++)
                if (Seams(i, j) != 0) seamFlag(i) = 1.0;
        VectorXd singFlag = VectorXd::Zero(V.rows());
        for (int i = 0; i < singularityIndex.size(); i++)
            if (isSingularity(i)) singFlag(i) = (double)singularityIndex(i);

        bool ok2 = igl::writeDMAT(base + "_barydef.dmat", B_def_dump, true)
                && igl::writeDMAT(base + "_x1def.dmat", X1_def, true)
                && igl::writeDMAT(base + "_x2def.dmat", X2_def, true)
                && igl::writeDMAT(base + "_x1combed.dmat", X1_combed, true)
                && igl::writeDMAT(base + "_x2combed.dmat", X2_combed, true)
                && igl::writeDMAT(base + "_vdef.dmat", V_deformed, true)
                && igl::writeDMAT(base + "_seamfaces.dmat", seamFlag, true)
                && igl::writeDMAT(base + "_singularity.dmat", singFlag, true);
        if (ok2) cout << "  Dumped combing-stage fields to " << base << "_{x1def,x1combed,...}.dmat" << endl;
    }

    // 7. Run MIQ with fallback strategies
    cout << "  Step 7: Running MIQ solver..." << endl;
    
    bool miq_success = runMIQ(V_deformed, F, X1_combed, X2_combed,
                               MMatch, isSingularity, Seams,
                               V_uv, F_uv,
                               miq_gradient_size, miq_stiffness,
                               use_direct_round);

    // Dump the integration result: per-corner UVs, the UV-space triangle
    // layout (which is the cut mesh flattened), and the seam/singularity
    // markers, so the parameterization can be inspected offline.
    if (dump_fields && miq_success) {
        string base = field_dump_prefix;
        MatrixXd FUVd(F_uv.rows(), 3);
        for (int i = 0; i < F_uv.rows(); i++)
            for (int j = 0; j < 3; j++) FUVd(i, j) = (double)F_uv(i, j);
        MatrixXd Fd(F.rows(), 3);
        for (int i = 0; i < F.rows(); i++)
            for (int j = 0; j < 3; j++) Fd(i, j) = (double)F(i, j);
        MatrixXd seamEdges(Seams.rows(), 3);
        for (int i = 0; i < Seams.rows(); i++)
            for (int j = 0; j < 3; j++) seamEdges(i, j) = (double)Seams(i, j);

        bool ok3 = igl::writeDMAT(base + "_uv.dmat", V_uv, true)
                && igl::writeDMAT(base + "_fuv.dmat", FUVd, true)
                && igl::writeDMAT(base + "_ftri.dmat", Fd, true)
                && igl::writeDMAT(base + "_seamedges.dmat", seamEdges, true);
        if (ok3) cout << "  Dumped integration result to " << base << "_{uv,fuv,ftri,seamedges}.dmat" << endl;
    }

    if (miq_success) {
        cout << "\nMIQ completed successfully!" << endl;
        cout << "  UV size: " << V_uv.rows() << " x " << V_uv.cols() << endl;
        cout << "  UV range: u=[" << V_uv.col(0).minCoeff() << ", " << V_uv.col(0).maxCoeff()
             << "], v=[" << V_uv.col(1).minCoeff() << ", " << V_uv.col(1).maxCoeff() << "]" << endl;
    } else {
        cerr << "\nAll MIQ strategies failed!" << endl;
        cerr << "Possible issues:" << endl;
        cerr << "  - Invalid cross field (check for NaN or zero vectors)" << endl;
        cerr << "  - Degenerate mesh faces" << endl;
        cerr << "  - Incompatible singularity configuration" << endl;
        return 1;
    }

    // Write a quad (or general polygon) mesh as OBJ; igl::writeOBJ is triangles only.
    auto writeQuadOBJ = [](const string& filename, const MatrixXd& Vq, const MatrixXi& Fq,
                           const VectorXi& Dq = VectorXi()) -> bool {
        ofstream file(filename);
        if (!file.is_open()) {
            cerr << "Error: Could not open file " << filename << " for writing" << endl;
            return false;
        }

        file << "# Quad mesh exported from MIQ remesher" << endl;
        file << "# Vertices: " << Vq.rows() << ", Faces: " << Fq.rows() << endl;

        for (int i = 0; i < Vq.rows(); i++) {
            file << "v " << Vq(i, 0) << " " << Vq(i, 1) << " " << Vq(i, 2) << endl;
        }

        // OBJ uses 1-based indexing. Dq, when given, holds the per-face degree
        // so n-gons (which QEx can legitimately return) are written correctly.
        for (int i = 0; i < Fq.rows(); i++) {
            const int deg = (Dq.size() == Fq.rows()) ? Dq(i) : (int)Fq.cols();
            file << "f";
            for (int j = 0; j < deg; j++) {
                file << " " << (Fq(i, j) + 1);
            }
            file << endl;
        }

        file.close();
        return true;
    };

    // Output path, from the optional [output_folder] and [name] arguments.
    auto outputFilename = [&]() -> string {
        string folder = out_folder;
        if (!folder.empty() && folder.back() != '/') folder += '/';
        const string base = out_name.empty() ? string("meshRemeshed")
                                             : out_name + "_Remeshed";
        return folder + base + ".obj";
    };

    // Dispatch to the extractor chosen on the command line. Falls back to the
    // built-in grid extractor if QEx fails, so a failed extraction still yields
    // something to look at.
    auto runExtractor = [&](MatrixXd& V_quad, MatrixXi& F_quad, VectorXi& D_quad) -> bool {
#ifdef HAVE_LIBQEX
        if (use_qex) {
            if (extractQuadMeshQEx(V, F, V_uv, F_uv, V_quad, F_quad, D_quad))
                return true;
            cout << "  QEx failed; falling back to the built-in grid extractor." << endl;
        }
#endif
        const bool ok = extractQuadMesh(V, F, V_uv, F_uv, V_quad, F_quad);
        if (ok) D_quad = VectorXi::Constant(F_quad.rows(), 4);  // grid path is all quads
        return ok;
    };

    // Batch mode: extract, save, and exit without opening a window. This is what
    // makes the remesher usable headless / from a script.
    if (batch_mode) {
        MatrixXd V_quad;
        MatrixXi F_quad;
        VectorXi D_quad;
        if (!runExtractor(V_quad, F_quad, D_quad)) {
            cerr << "Quad extraction failed." << endl;
            return 1;
        }
        const string out = outputFilename();
        if (!writeQuadOBJ(out, V_quad, F_quad, D_quad)) return 1;
        cout << "Saved quad mesh to " << out << endl;
        return 0;
    }

    // 3. Visualization and Saving
    igl::opengl::glfw::Viewer viewer;

    viewer.data().set_mesh(V, F);
    viewer.data().set_uv(V_uv, F_uv);
    viewer.data().show_texture = true;
    
    Matrix<unsigned char, Dynamic, Dynamic> R, G, B_tex;
    line_texture(R, G, B_tex);
    viewer.data().set_texture(R, G, B_tex);

    std::set<int> good_faces_set(good_faces.begin(), good_faces.end());

    cout << "\n=== Controls ===" << endl;
    cout << "  '1'  Original mesh (UV texture)     '2'  Deformed mesh (UV texture)" << endl;
    cout << endl;
    cout << "  '3'  INPUT frame field, as supplied (on "
         << (defined_on_faces ? "faces" : "vertices") << ")" << endl;
    cout << "  '4'  INTERPOLATED frame field (comiso::frame_field, per face)" << endl;
    cout << "  '5'  INTEGRATED (combed) cross field -- what MIQ integrated" << endl;
    cout << "  '6'  Seams and singularities" << endl;
    cout << "  '7'  DEFORMED mesh + its cross field" << endl;
    cout << endl;
    cout << "  'R'  Preview the extracted quad mesh    'S'  Extract and SAVE it" << endl;
    cout << "  'F'  Per-face interpolated bc1/bc2" << endl;
    cout << "  '+' / '-'  Coarser / finer quads (re-runs MIQ)" << endl;
    cout << "  '0'  Toggle the seam + singularity overlay on views 3-7" << endl;
    cout << endl;
    cout << "  Red = first direction (D1/FF1/X1), blue = second (D2/FF2/X2)." << endl;
    cout << "  Magenta lines = seams; green/purple dots = +1/4 and -1/4 singularities." << endl;
    cout << "  Add 'batch' on the command line to skip the viewer and just write the mesh." << endl;

    // Re-run MIQ at a new gradient size ('+' / '-'), keeping the same method.
    auto recomputeMIQ = [&](double new_gradient_size) {
        cout << "Recomputing MIQ with gradient_size=" << new_gradient_size
             << ", method=" << (use_direct_round ? "direct" : "integration") << "..." << endl;

        igl::copyleft::comiso::miq(
            V_deformed, F,
            X1_combed, X2_combed,
            MMatch, isSingularity, Seams,
            V_uv, F_uv,
            new_gradient_size,
            miq_stiffness,
            use_direct_round,
            15,
            5,
            true,
            false);
        cout << "Done." << endl;
        return true;
    };

    // Seams (and singularities) overlaid on the field views: a cross field is
    // ambiguous without them, since the seams are where the 4-RoSy branch
    // switches. Drawn on whichever mesh the caller set (indices match both).
    auto drawSeams = [&](igl::opengl::glfw::Viewer& v, const MatrixXd& VS) {
        if (!show_seams_overlay) return;
        // Seam edges lie exactly ON mesh edges, so drawn flat they z-fight with
        // the surface and disappear among the field crosses. Lift them a little
        // along the face normal (and the singular vertices along the vertex
        // normal) so they read clearly on top of the field.
        const double lift = avg_edge_len * 0.06;
        const int l_count = Seams.sum();
        if (l_count > 0) {
            MatrixXd P1(l_count, 3), P2(l_count, 3);
            int e = 0;
            for (int i = 0; i < Seams.rows(); i++)
                for (int j = 0; j < Seams.cols(); j++)
                    if (Seams(i, j) != 0) {
                        const RowVector3d off = lift * FN.row(i);
                        P1.row(e) = VS.row(F(i, j))               + off;
                        P2.row(e) = VS.row(F(i, (j + 1) % 3))     + off;
                        e++;
                    }
            // Bright magenta: distinct from the red/blue of every field view.
            v.data().add_edges(P1, P2, RowVector3d(1.0, 0.0, 0.8));
            v.data().line_width = 5.0f;
        }

        // Average the incident face normals to lift each singular vertex clear.
        MatrixXd VN;
        igl::per_vertex_normals(VS, F, VN);
        for (int i = 0; i < singularityIndex.size(); i++) {
            if (singularityIndex(i) != 1 && singularityIndex(i) != 3) continue;
            RowVector3d pn = VN.row(i);
            if (!pn.allFinite() || pn.norm() < 1e-9) pn.setZero();
            const RowVector3d p = VS.row(i) + 1.5 * lift * pn;
            v.data().add_points(p, singularityIndex(i) == 1 ? RowVector3d(0, 0.9, 0.2)
                                                           : RowVector3d(0.5, 0, 1.0));
        }
        v.data().point_size = 14.0f;
    };

    // Held in a named std::function so a view can re-invoke itself.
    std::function<bool(igl::opengl::glfw::Viewer&, unsigned char, int)> handleKey;
    handleKey = [&](igl::opengl::glfw::Viewer& v, unsigned char key, int mod) -> bool {
        if (key == 'S' || key == 's') {
            MatrixXd V_quad;
            MatrixXi F_quad;
            VectorXi D_quad;
            if (runExtractor(V_quad, F_quad, D_quad)) {
                const string output_filename = outputFilename();
                if (writeQuadOBJ(output_filename, V_quad, F_quad, D_quad)) {
                    cout << "Saved quad mesh to " << output_filename << endl;
                }
            }
            return true;
        }

        if (key == 'R' || key == 'r') {
            cout << "Display Mode: Extracted Quad Mesh Preview" << endl;
            MatrixXd V_quad;
            MatrixXi F_quad;
            VectorXi D_quad;
            if (runExtractor(V_quad, F_quad, D_quad)) {
                // Convert faces to triangles for visualization (fan triangulation,
                // so n-gons display correctly too)
                vector<RowVector3i> tris;
                for (int i = 0; i < F_quad.rows(); i++) {
                    const int deg = (D_quad.size() == F_quad.rows()) ? D_quad(i) : (int)F_quad.cols();
                    for (int k = 1; k + 1 < deg; k++)
                        tris.push_back(RowVector3i(F_quad(i, 0), F_quad(i, k), F_quad(i, k + 1)));
                }
                MatrixXi F_tri((int)tris.size(), 3);
                for (size_t t = 0; t < tris.size(); t++) F_tri.row((int)t) = tris[t];

                v.data().clear();
                v.data().set_mesh(V_quad, F_tri);
                v.data().show_texture = false;
                v.data().set_colors(RowVector3d(0.8, 0.8, 0.9));

                // Draw face edges to show the quad structure
                for (int i = 0; i < F_quad.rows(); i++) {
                    const int deg = (D_quad.size() == F_quad.rows()) ? D_quad(i) : (int)F_quad.cols();
                    for (int k = 0; k < deg; k++) {
                        v.data().add_edges(V_quad.row(F_quad(i, k)),
                                           V_quad.row(F_quad(i, (k + 1) % deg)),
                                           RowVector3d(0, 0, 0));
                    }
                }

                cout << "  Showing " << F_quad.rows() << " quads (" << V_quad.rows() << " vertices)" << endl;
            } else {
                cout << "  Failed to extract quad mesh!" << endl;
            }
            return true;
        }

        if (key == '+' || key == '=') {
            miq_gradient_size *= 1.5;
            if (recomputeMIQ(miq_gradient_size)) {
                v.data().set_uv(V_uv, F_uv);
            }
            return true;
        }

        if (key == '-' || key == '_') {
            miq_gradient_size /= 1.5;
            if (miq_gradient_size < 1.0) miq_gradient_size = 1.0;
            if (recomputeMIQ(miq_gradient_size)) {
                v.data().set_uv(V_uv, F_uv);
            }
            return true;
        }

        if (key == '1') {
            display_mode = 1;
            cout << "Display Mode: Original Mesh" << endl;
            v.data().set_mesh(V, F);
            v.data().set_uv(V_uv, F_uv);
            v.data().show_texture = true;
            v.data().clear_edges();
            if (show_cross_field) {
                drawCrossField(v, B, FF1, FF2, avg_edge_len * 0.5);
            }
            return true;
        }
        else if (key == '2') {
            display_mode = 2;
            cout << "Display Mode: Deformed Mesh" << endl;
            v.data().set_mesh(V_deformed, F);
            v.data().set_uv(V_uv, F_uv);
            v.data().show_texture = true;
            v.data().clear_edges();
            if (show_cross_field) {
                drawCrossField(v, B_deformed, FF1_def, FF2_def, avg_edge_len * 0.5);
            }
            return true;
        }
        else if (key == '0') {
            show_seams_overlay = !show_seams_overlay;
            cout << "Seam/singularity overlay: " << (show_seams_overlay ? "ON" : "OFF") << endl;
            if (display_mode >= 3 && display_mode <= 7)
                return handleKey(v, (unsigned char)('0' + display_mode), mod);
            return true;
        }
        else if (key == '3') {
            // 1) INPUT frame field, exactly as supplied by the user.
            display_mode = 3;
            cout << "Display Mode 3: INPUT frame field, as supplied (on "
                 << (defined_on_faces ? "faces" : "vertices") << ")" << endl;
            v.data().set_mesh(V, F);
            v.data().show_texture = false;
            v.data().clear_edges();

            // Shade the faces that carry a constraint.
            {
                MatrixXd C = MatrixXd::Constant(F.rows(), 3, 0.9);
                for (int fi = 0; fi < F.rows(); fi++)
                    if (faceHasDir[fi]) C.row(fi) << 0.80, 0.92, 0.80;
                v.data().set_colors(C);
            }

            const double scale = avg_edge_len * 0.4;
            int shown = 0;
            for (int ei = 0; ei < numElements; ei++) {
                if (!hasDir[ei]) continue;
                const RowVector3d pos = defined_on_faces ? B.row(ei) : V.row(ei);
                RowVector3d d1 = D1_elem.row(ei), d2 = D2_elem.row(ei);
                if (d1.norm() > 1e-9)
                    v.data().add_edges(pos - scale * d1.normalized(), pos + scale * d1.normalized(), RowVector3d(1, 0, 0));
                if (d2.norm() > 1e-9)
                    v.data().add_edges(pos - scale * d2.normalized(), pos + scale * d2.normalized(), RowVector3d(0, 0, 1));
                shown++;
            }
            cout << "  " << shown << " / " << numElements << " "
                 << (defined_on_faces ? "faces" : "vertices") << " carry a direction"
                 << "  (green tint = constrained faces)" << endl;
            drawSeams(v, V);
            return true;
        }
        else if (key == '4') {
            // 2) INTERPOLATED frame field: the solver's output FF1/FF2, per face.
            display_mode = 4;
            cout << "Display Mode 4: INTERPOLATED frame field (comiso::frame_field, per face)" << endl;
            v.data().set_mesh(V, F);
            v.data().show_texture = false;
            v.data().clear_edges();
            v.data().set_colors(RowVector3d(1, 1, 1));

            const double scale = avg_edge_len * 0.4;
            for (int fi = 0; fi < F.rows(); fi++) {
                const RowVector3d c = B.row(fi);
                RowVector3d f1 = FF1.row(fi), f2 = FF2.row(fi);
                if (f1.norm() > 1e-12) f1.normalize();
                if (f2.norm() > 1e-12) f2.normalize();
                v.data().add_edges(c - scale * f1, c + scale * f1, RowVector3d(1, 0, 0));
                v.data().add_edges(c - scale * f2, c + scale * f2, RowVector3d(0, 0, 1));
            }
            cout << "  " << F.rows() << " faces. Frames may be non-orthogonal / non-unit here." << endl;
            drawSeams(v, V);
            return true;
        }
        else if (key == '5') {
            // 3) INTEGRATED frame field: the combed cross field MIQ integrated,
            //    drawn on the ORIGINAL surface (re-projected into its tangent planes).
            display_mode = 5;
            cout << "Display Mode 5: INTEGRATED (combed) cross field -- what MIQ integrated" << endl;
            v.data().set_mesh(V, F);
            v.data().show_texture = false;
            v.data().clear_edges();
            v.data().set_colors(RowVector3d(1, 1, 1));

            const double scale = avg_edge_len * 0.4;
            for (int fi = 0; fi < F.rows(); fi++) {
                const RowVector3d c = B.row(fi);
                const Vector3d n = FN.row(fi);
                Vector3d a = X1_combed.row(fi).transpose();
                Vector3d b2 = X2_combed.row(fi).transpose();
                a  -= a.dot(n) * n;
                b2 -= b2.dot(n) * n;
                if (a.norm()  > 1e-12) a.normalize();
                if (b2.norm() > 1e-12) b2.normalize();
                v.data().add_edges(c - scale * a.transpose(),  c + scale * a.transpose(),  RowVector3d(1, 0, 0));
                v.data().add_edges(c - scale * b2.transpose(), c + scale * b2.transpose(), RowVector3d(0, 0, 1));
            }
            cout << "  Combed on the original surface (re-projected into each face plane)." << endl;
            drawSeams(v, V);
            return true;
        }
        else if (key == '6') {
            // 4) Seams and singularities, on the original surface.
            display_mode = 6;
            cout << "Display Mode 6: Seams and singularities" << endl;
            v.data().set_mesh(V, F);
            v.data().show_texture = false;
            v.data().clear_edges();
            v.data().set_colors(RowVector3d(1, 1, 1));

            const int l_count = Seams.sum();
            if (l_count > 0) {
                MatrixXd P1(l_count, 3), P2(l_count, 3);
                int e = 0;
                for (int i = 0; i < Seams.rows(); i++)
                    for (int j = 0; j < Seams.cols(); j++)
                        if (Seams(i, j) != 0) {
                            P1.row(e) = V.row(F(i, j));
                            P2.row(e) = V.row(F(i, (j + 1) % 3));
                            e++;
                        }
                v.data().add_edges(P1, P2, RowVector3d(1, 0, 0));
                v.data().line_width = 4.0f;
            }
            for (int i = 0; i < singularityIndex.size(); i++) {
                if (singularityIndex(i) == 1)
                    v.data().add_points(V.row(i), RowVector3d(0, 1, 0));
                else if (singularityIndex(i) == 3)
                    v.data().add_points(V.row(i), RowVector3d(1, 0, 0));
            }
            v.data().point_size = 13.0f;
            cout << "  Red edges = seams: " << l_count
                 << " | Green = +1/4 (" << pos_sing << "), Red = -1/4 (" << neg_sing << ")" << endl;
            return true;
        }
        else if (key == '7') {
            // 5) The DEFORMED mesh with its own cross field (the internal
            //    anisotropy embedding MIQ was actually run on).
            display_mode = 7;
            cout << "Display Mode 7: DEFORMED mesh + its cross field" << endl;
            v.data().set_mesh(V_deformed, F);
            v.data().show_texture = false;
            v.data().clear_edges();
            v.data().set_colors(RowVector3d(1, 1, 1));

            const double scale = avg_edge_len * 0.4;
            for (int fi = 0; fi < F.rows(); fi++) {
                const RowVector3d c = B_deformed.row(fi);
                RowVector3d x1 = X1_combed.row(fi), x2 = X2_combed.row(fi);
                if (x1.norm() > 1e-12) x1.normalize();
                if (x2.norm() > 1e-12) x2.normalize();
                v.data().add_edges(c - scale * x1, c + scale * x1, RowVector3d(1, 0, 0));
                v.data().add_edges(c - scale * x2, c + scale * x2, RowVector3d(0, 0, 1));
            }
            cout << "  This is the embedding the frame field was made orthogonal in." << endl;
            drawSeams(v, V_deformed);
            return true;
        }
        else if (key == 'F' || key == 'f') {
            display_mode = 6;
            cout << "Display Mode: bc1/bc2 per-face" << endl;
            v.data().set_mesh(V, F);
            v.data().set_uv(V_uv, F_uv);
            v.data().clear_edges();
            v.data().set_colors(RowVector3d(0.9, 0.9, 0.9));

            double scale = avg_edge_len * 0.4;
            for (int fi = 0; fi < F.rows(); fi++) {
                RowVector3d center = B.row(fi);
                RowVector3d d1 = bc1.row(fi);
                RowVector3d d2 = bc2.row(fi);
                v.data().add_edges(center - scale * d1, center + scale * d1, RowVector3d(1, 0, 0));
                v.data().add_edges(center - scale * d2, center + scale * d2, RowVector3d(0, 0, 1));
            }
            return true;
        }
        return false;
    };

    viewer.callback_key_pressed = handleKey;

    viewer.launch();
    return 0;
}