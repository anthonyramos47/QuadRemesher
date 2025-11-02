#include <igl/readOBJ.h>
#include <Eigen/Core>
#include <iostream>
#include <string>

// Directional includes
#include <igl/unproject_onto_mesh.h>
#include <directional/readOBJ.h>
#include <directional/CartesianField.h>
#include <directional/TriMesh.h>
#include <directional/IntrinsicVertexTangentBundle.h>
#include <directional/IntrinsicFaceTangentBundle.h>
#include <directional/read_raw_field.h>
#include <directional/power_field.h>
#include <directional/power_to_raw.h>
#include <directional/directional_viewer.h>
#include <directional/setup_integration.h>
#include <directional/integrate.h>
#include <directional/combing.h>
#include <directional/curl_matching.h>
#include <directional/writeOBJ.h>
#include <directional/write_raw_field.h>
#include <directional/cut_mesh_with_singularities.h>
#include <directional/directional_viewer.h>


int N = 4;
directional::TriMesh mesh, meshCut;
directional::IntrinsicVertexTangentBundle vtb;
directional::IntrinsicFaceTangentBundle ftb;
directional::CartesianField rawFaceField, powerFaceField, combedField;
directional::CartesianField rawVertexField, powerVertexField;
Eigen::MatrixXd cutUVFull, cutUVRot, cornerWholeUV;
directional::DirectionalViewer viewer;

typedef enum {FIELD, ROT_INTEGRATION, FULL_INTEGRATION} ViewingModes;
ViewingModes viewingMode=FIELD;

const std::string OUTPUT_PATH = "/home/anthony/KAUSTLocal/Qremesh/out";


Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> texture_R, texture_G, texture_B;
void setup_line_texture()
{
  unsigned size = 128;
  unsigned size2 = size/2;
  unsigned lineWidth = 5;
  texture_B.setConstant(size, size, 0);
  texture_G.setConstant(size, size, 0);
  texture_R.setConstant(size, size, 0);
  for (unsigned i=0; i<size; ++i)
    for (unsigned j=size2-lineWidth; j<=size2+lineWidth; ++j)
      texture_B(i,j) = texture_G(i,j) = texture_R(i,j) = 255;
  for (unsigned i=size2-lineWidth; i<=size2+lineWidth; ++i)
    for (unsigned j=0; j<size; ++j)
      texture_B(i,j) = texture_G(i,j) = texture_R(i,j) = 255;
}



void update_viewer()
{
  if (viewingMode==FIELD){
    viewer.set_active(true,0);
    viewer.set_active(false,1);
    viewer.toggle_texture(false,1);
  } else if ((viewingMode==ROT_INTEGRATION) || (viewingMode==FULL_INTEGRATION)){
    viewer.set_uv(viewingMode==ROT_INTEGRATION ? cutUVRot : cutUVFull,1);
    viewer.set_active(true,1);
    viewer.set_active(false,0);
    viewer.toggle_texture(true,1);
  }
}

// Handle keyboard input
bool key_down(igl::opengl::glfw::Viewer& viewer, int key, int modifiers)
{
  switch (key)
  {
      // Select vector
    case '1': viewingMode = FIELD; break;
    case '2': viewingMode = ROT_INTEGRATION; break;
    case '3': viewingMode = FULL_INTEGRATION; break;
    case 'W':
      Eigen::MatrixXd emptyMat;
      directional::writeOBJ(OUTPUT_PATH+"/mesh-rot-seamless.obj", meshCut, cutUVRot, meshCut.F);
      directional::writeOBJ(OUTPUT_PATH+"/mesh-full-seamless.obj", meshCut, cutUVFull, meshCut.F);
      break;
  }
  update_viewer();
  return true;
}


void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -i, --input <file>     Load mesh from OBJ file\n";
    std::cout << "  --cube                 Load standard cube mesh\n";
    std::cout << "  --sphere               Load standard sphere mesh\n";
    std::cout << "  -h, --help             Show this help message\n\n";
    std::cout << "Examples:\n";
    std::cout << "  " << programName << " -i data/mesh.obj\n";
    std::cout << "  " << programName << " --cube\n";
    std::cout << "  " << programName << " --sphere\n";
}

// Create a simple cube mesh
void createCube(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
    V.resize(8, 3);
    V << -1, -1, -1,
          1, -1, -1,
          1,  1, -1,
         -1,  1, -1,
         -1, -1,  1,
          1, -1,  1,
          1,  1,  1,
         -1,  1,  1;

    F.resize(12, 3);
    F << 0, 2, 1,  0, 3, 2,  // front
         4, 5, 6,  4, 6, 7,  // back
         0, 1, 5,  0, 5, 4,  // bottom
         3, 6, 2,  3, 7, 6,  // top
         0, 4, 7,  0, 7, 3,  // left
         1, 2, 6,  1, 6, 5;  // right
}

// Create a simple icosphere
void createSphere(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
    V.resize(12, 3);
    V <<  0.000, -1.000,  0.000,
          0.724, -0.447,  0.526,
         -0.276, -0.447,  0.851,
         -0.894, -0.447,  0.000,
         -0.276, -0.447, -0.851,
          0.724, -0.447, -0.526,
          0.276,  0.447,  0.851,
         -0.724,  0.447,  0.526,
         -0.724,  0.447, -0.526,
          0.276,  0.447, -0.851,
          0.894,  0.447,  0.000,
          0.000,  1.000,  0.000;

    F.resize(20, 3);
    F << 0, 1, 2,
         1, 0, 5,
         0, 2, 3,
         0, 3, 4,
         0, 4, 5,
         1, 10, 6,
         2, 6, 7,
         3, 7, 8,
         4, 8, 9,
         5, 9, 10,
         1, 6, 2,
         2, 7, 3,
         3, 8, 4,
         4, 9, 5,
         5, 10, 1,
         6, 11, 7,
         7, 11, 8,
         8, 11, 9,
         9, 11, 10,
         10, 11, 6;
}

int main(int argc, char* argv[]) {
    std::cout << "========================================\n";
    std::cout << "  FieldMesh - Directional Field Viewer\n";
    std::cout << "========================================\n\n";

    // Parse command line arguments
    std::string inputFile;
    bool useCube = false;
    bool useSphere = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "-i" || arg == "--input") {
            if (i + 1 < argc) {
                inputFile = argv[++i];
            } else {
                std::cerr << "Error: " << arg << " requires a file path\n";
                return 1;
            }
        }
        else if (arg == "--cube") {
            useCube = true;
        }
        else if (arg == "--sphere") {
            useSphere = true;
        }
        else {
            std::cerr << "Error: Unknown argument '" << arg << "'\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    // Check if we have input
    if (inputFile.empty() && !useCube && !useSphere) {
        std::cout << "No input specified, using default sphere.\n\n";
        useSphere = true;
    }

    // Load or create mesh
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    if (!inputFile.empty()) {
        std::cout << "Loading mesh from: " << inputFile << std::endl;
        if (!igl::readOBJ(inputFile, V, F)) {
            std::cerr << "Error: Failed to load mesh from " << inputFile << std::endl;
            return 1;
        }
        std::cout << "Loaded mesh with " << V.rows() << " vertices and "
                  << F.rows() << " faces\n";
    }
    else if (useCube) {
        std::cout << "Creating cube mesh\n";
        createCube(V, F);
        std::cout << "Created cube with " << V.rows() << " vertices and "
                  << F.rows() << " faces\n";
    }
    else if (useSphere) {
        std::cout << "Creating sphere mesh\n";
        createSphere(V, F);
        std::cout << "Created sphere with " << V.rows() << " vertices and "
                  << F.rows() << " faces\n";
    }

    std::cout << "\nMesh loaded successfully!\n";
    std::cout << "Vertices: " << V.rows() << "\n";
    std::cout << "Faces: " << F.rows() << "\n\n";

    // Initialize Directional data structures
    std::cout << "Initializing Directional structures...\n";

    // Set mesh
    mesh.set_mesh(V, F);

    directional::IntrinsicFaceTangentBundle ftb;
    ftb.init(mesh);
    vtb.init(mesh);

    std::cout << "Directional TriMesh and TangentBundle initialized!\n";
    std::cout << "  - Faces: " << mesh.F.rows() << "\n";
    std::cout << "  - Tangent spaces: " << ftb.sources.rows() << "\n\n";

    std::cout << "Launching Directional viewer...\n\n";

    Eigen::VectorXi constFaces, constVertices;
    Eigen::MatrixXd constVectors;
    constFaces.resize(1);
    constFaces<<0;
    constVectors.resize(1,3);
    constVectors<<mesh.V.row(mesh.F(0,2))-mesh.V.row(mesh.F(0,1));
    constVertices.resize(1);
    constVertices<<mesh.F(0,1);

    directional::power_field(vtb, constVertices, constVectors, Eigen::VectorXd::Constant(constVertices.size(),-1.0), N, powerVertexField);
    directional::power_field(ftb, constFaces, constVectors, Eigen::VectorXd::Constant(constFaces.size(),-1.0), N, powerFaceField);

    // computing power fields
    directional::power_to_raw(powerFaceField, N, rawFaceField,true);
    directional::power_to_raw(powerVertexField, N, rawVertexField,true);
    
    directional::principal_matching(rawFaceField);
    directional::principal_matching(rawVertexField);

    // Combing and cutting
    Eigen::VectorXd curlNorm;
    directional::curl_matching(rawFaceField, curlNorm);
    std::cout << "Curl norm max: " << curlNorm.maxCoeff() << std::endl;

    directional::IntegrationData intData(N);
    directional::setup_integration(rawFaceField, intData, meshCut, combedField);

    intData.verbose=true;
    intData.integralSeamless=false;
    
    std::cout<<"Solving for permutationally-seamless integration"<<std::endl;
    directional::integrate(combedField, intData, meshCut, cutUVRot ,cornerWholeUV);
    //Extracting the UV from [U,V,-U, -V];
    cutUVRot=cutUVRot.block(0,0,cutUVRot.rows(),2);
    std::cout<<"Done!"<<std::endl;

    intData.verbose=true;
    intData.integralSeamless=true;
    std::cout<<"Solving for integrally-seamless integration"<<std::endl;
    directional::integrate(combedField,  intData, meshCut, cutUVFull,cornerWholeUV);
    cutUVFull=cutUVFull.block(0,0,cutUVFull.rows(),2);
    std::cout<<"Done!"<<std::endl;

    viewer.set_mesh(mesh, 0);
    viewer.set_mesh(meshCut, 1);

    viewer.set_field(combedField, directional::DirectionalViewer::indexed_glyph_colors(combedField.extField, false));
    viewer.set_seams(combedField.matching);
    viewer.set_texture(texture_R,texture_G,texture_B,1);

      
    viewer.toggle_texture(false,0);
    viewer.toggle_field(true,0);
    viewer.toggle_seams(true,0);

    viewer.toggle_texture(true,1);
    viewer.toggle_field(false,1);
    viewer.toggle_seams(false,1);

  
    // viewer.set_mesh(mesh, 1);


    // viewer.set_field(rawFaceField,Eigen::MatrixXd(),0, 1.5, 0, 0.3);
    // viewer.set_field(rawVertexField,Eigen::MatrixXd(),1, 1.5, 0, 0.4);
    // viewer.toggle_field(viewingMode==FACE_FIELD,0);


    std::cout << "Viewer Controls:\n";
    std::cout << "  - Click and drag to rotate\n";
    std::cout << "  - Scroll to zoom\n";
    std::cout << "  - Right-click and drag to pan\n";
    std::cout << "  - Press ESC or close window to exit\n\n";

      
    update_viewer();
    
    viewer.callback_key_down = &key_down;
    viewer.launch();

    std::cout << "\nApplication closed.\n";

    return 0;
}
