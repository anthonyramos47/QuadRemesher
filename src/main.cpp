#include <igl/readOBJ.h>
#include <Eigen/Core>
#include <iostream>
#include <string>

// Directional includes
#include <igl/unproject_onto_mesh.h>
#include <igl/principal_curvature.h>
#include <directional/readOBJ.h>
#include <directional/CartesianField.h>
#include <directional/TriMesh.h>
#include <directional/IntrinsicVertexTangentBundle.h>
#include <directional/IntrinsicFaceTangentBundle.h>
#include <directional/read_raw_field.h>
#include <directional/power_field.h>
#include <directional/power_to_raw.h>
#include <directional/polyvector_to_raw.h>
#include <directional/polyvector_field.h>


#include <directional/combing.h>
#include <directional/curl_matching.h>
#include <directional/writeOBJ.h>
#include <directional/write_raw_field.h>
#include <directional/cut_mesh_with_singularities.h>
#include <directional/directional_viewer.h>
#include "polygonal_write_OFF.h"
#include <directional/mesh_function_isolines.h>
#include <directional/branched_isolines.h>
#include <directional/setup_mesh_function_isolines.h>
#include <directional/setup_integration.h>
#include <directional/integrate.h>


int N = 4;
directional::TriMesh mesh, meshCut;
directional::IntrinsicVertexTangentBundle vtb;
directional::IntrinsicFaceTangentBundle ftb;
directional::CartesianField rawFaceField, prosFaceField, powerFaceField, combedField;
Eigen::MatrixXd cutUVFull, cornerWholeUV;
directional::DirectionalViewer viewer;


typedef enum {FIELD, FIELD_2, FULL_INTEGRATION} ViewingModes;
ViewingModes viewingMode=FIELD;

const std::string OUTPUT_PATH = "../out/";
const std::string INPUT_PATH = "../data/";

void saveObj(const std::string& filename,
                    const Eigen::MatrixXd& V,
                    const Eigen::MatrixXi& F,
                    const Eigen::VectorXi& D)
{   

    // Open file
    std::ofstream file(OUTPUT_PATH + filename);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << OUTPUT_PATH + filename << " for writing.\n";  
        return;
    }
    // Write vertices
    for (int i = 0; i < V.rows(); ++i) {
        file << "v " << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << "\n";
    }
    // Write faces
    for (int i = 0; i < F.rows(); ++i) {
        file << "f";
        for (int j=0;j< D(i);j++){
            file << " " << (F(i, j) + 1); // OBJ format uses 1-based indexing
        }
        file << "\n";
    }
}


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
    viewer.toggle_field(true,0);

    viewer.set_active(false, 1);
    viewer.toggle_field(false,1);
    
  } 
  else if (viewingMode==FIELD_2){
    viewer.set_active(false,0);
    viewer.toggle_field(false,0);
    viewer.set_active(true, 1);
    viewer.toggle_field(true,1);
    
    

    
    //viewer.toggle_field(true,1);
  }
//   else if (viewingMode==FULL_INTEGRATION){
//     viewer.set_uv(cutUVFull,1);
//     viewer.set_active(true,1);
//     viewer.set_active(false,0);
//     viewer.toggle_texture(true,1);
//   } else if (viewingMode==PRINCIPAL_FIELD){
//     viewer.set_active(true,2);
//     viewer.set_active(false,0);
//     viewer.set_active(false,1);
//     viewer.toggle_texture(false,1); 
//   }
}

// void integrate_field()
// {
//     directional::IntegrationData intData;
//     intData.verbose=true;
//     intData.integralSeamless=true;
//     std::cout<<"Solving for integrally-seamless integration"<<std::endl;
//     directional::integrate(combedField,  intData, meshCut, cutUVFull,cornerWholeUV);
//     cutUVFull=cutUVFull.block(0,0,cutUVFull.rows(),2);
//     std::cout<<"Done!"<<std::endl;

// }


// Handle keyboard input
bool key_down(igl::opengl::glfw::Viewer& viewer, int key, int modifiers)
{
  switch (key)
  {
      // Select vector
    case '1': viewingMode = FIELD; break;
    case '2': viewingMode = FIELD_2; break;
    // case 'p': viewingMode = PRINCIPAL_FIELD; break;
    // case '3': 
    //     integrate_field(); 
    //     break;
    case 'W':
      Eigen::MatrixXd emptyMat;
      
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
                inputFile = INPUT_PATH + argv[++i];
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

    setup_line_texture();

    // Initialize Directional data structures
    std::cout << "Initializing Directional structures...\n";

    // Set mesh
    mesh.set_mesh(V, F);

    // Compute curvature directions via quadric fitting
    Eigen::MatrixXd PD1,PD2;
    Eigen::VectorXd PV1,PV2;
    igl::principal_curvature(V,F,PD1,PD2,PV1,PV2);    
    ftb.init(mesh);

    std::cout << "Directional TriMesh and TangentBundle initialized!\n";
    std::cout << "  - Faces: " << mesh.F.rows() << "\n";
    std::cout << "  - Tangent spaces: " << ftb.sources.rows() << "\n\n";

    // Convert principal curvature directions to face-based cross field
    std::cout << "Converting principal curvature directions to face field...\n";


    // // Initialize raw field with N=4 directions per face
    Eigen::MatrixXd faceCurvatureField(mesh.F.rows(), 3*N);

    // For each face, average the principal directions from its vertices
    for (int f = 0; f < mesh.F.rows(); f++) {
        // Get the three vertex indices of the face
        int v0 = mesh.F(f, 0);
        int v1 = mesh.F(f, 1);
        int v2 = mesh.F(f, 2);

        // Average the first principal direction from the three vertices
        Eigen::Vector3d avgPD1 = PD1.row(v0) + PD1.row(v1) + PD1.row(v2);
        avgPD1.normalize();

        // Average the second principal direction from the three vertices
        Eigen::Vector3d avgPD2 = PD2.row(v0) + PD2.row(v1) + PD2.row(v2);
        avgPD2.normalize();

        // For a 4-RoSy field, store: PD1, PD2, -PD1, -PD2
        faceCurvatureField.block<1,3>(f, 0) =  avgPD1.transpose();
        faceCurvatureField.block<1,3>(f, 3) =  avgPD2.transpose();
        faceCurvatureField.block<1,3>(f, 6) = -avgPD1.transpose();
        faceCurvatureField.block<1,3>(f, 9) = -avgPD2.transpose();
    }

    // Convert principal curvature directions to face-based cross field
    std::cout << "Setting curvature directions as powerfield\n" << std::endl;

    // Set up constraints for power field generation
    Eigen::VectorXi constFaces;
    Eigen::MatrixXd constVectors;
    constFaces.resize(mesh.F.rows());
    constFaces<<Eigen::VectorXi::LinSpaced(mesh.F.rows(),0,mesh.F.rows()-1);
    // Constrained vectors per face
    constVectors.resize(mesh.F.rows(),3*2); // 2 directions per face
    constVectors<<faceCurvatureField.block(0,0,mesh.F.rows(),3), faceCurvatureField.block(0,3,mesh.F.rows(),3);
    //constVectors<<faceCurvatureField.block(0,0,mesh.F.rows(),3); // Use first principal direction for power field
    // Create power field 
    directional::power_field(ftb, constFaces, constVectors, Eigen::VectorXd::Constant(constFaces.size(),-1.0), N, powerFaceField);    
    // Convert to raw field
    directional::power_to_raw(powerFaceField, N, prosFaceField, true);
    // Compute principal matching
    directional::principal_matching(prosFaceField);


    Eigen::MatrixXd testFaceField(mesh.F.rows(), 3*2);
    // per each face, set two arbitrary non-orthogonal directions
    for (int f = 0; f < mesh.F.rows(); f++) {
        // Get the three vertex indices of the face
        int v0 = mesh.F(f, 0);
        int v1 = mesh.F(f, 1);
        int v2 = mesh.F(f, 2);



        Eigen::Vector3d diru = mesh.V.row(v1) - mesh.V.row(v0) ;
        diru.normalize();
        Eigen::Vector3d dirv = mesh.V.row(v2) - mesh.V.row(v0)  ;
        dirv.normalize();   

        double theta = M_PI/6; // 30 degrees
    

        Eigen::Vector3d dir1 =   cos(theta) * diru + sin(theta) * dirv; 
        Eigen::Vector3d dir2 =   cos(theta) * diru - sin(theta) * dirv; 
        testFaceField.block<1,3>(f, 0) = dir1.transpose();
        testFaceField.block<1,3>(f, 3) = dir2.transpose();
    }


    std::cout << "Use non-orthogonal field\n" << std::endl;

    // Reset faces 
    directional::polyvector_field(ftb, constFaces, testFaceField, N, powerFaceField);
    // Convert to raw field
    directional::polyvector_to_raw(powerFaceField, rawFaceField);
    // Compute principal matching
    directional::principal_matching(rawFaceField);

    // // Initialize rawFaceField from the curvature directions
    // rawFaceField.init(ftb, directional::fieldTypeEnum::RAW_FIELD, N);
    // rawFaceField.set_extrinsic_field(faceCurvatureField);

    // std::cout << "Principal curvature field created with " << rawFaceField.extField.rows()
    //           << " faces and " << N << " directions per face\n\n";

    // std::cout << "Launching Directional viewer...\n\n";

    // // Principal matching
    // principalFaceField.init(ftb, directional::fieldTypeEnum::RAW_FIELD, N);
    // principalFaceField.set_extrinsic_field(faceCurvatureField); // Copy raw field
    // directional::principal_matching(principalFaceField);
    // directional::principal_matching(rawVertexField);

    // Combing and cutting
    // Eigen::VectorXd curlNorm;
    // directional::curl_matching(rawFaceField, curlNorm);
    // std::cout << "Curl norm max: " << curlNorm.maxCoeff() << std::endl;
    

    // // Setup integration data
    directional::IntegrationData intData(N);
    directional::setup_integration(rawFaceField, intData, meshCut, combedField);

    intData.verbose=true;
    intData.integralSeamless=true;
    intData.roundSeams=true;
        
    std::cout<<"Solving for permutationally-seamless integration"<<std::endl;
    directional::integrate(combedField,  intData, meshCut, cutUVFull,cornerWholeUV);
    cutUVFull=cutUVFull.block(0,0,cutUVFull.rows(),2);
    std::cout<<"Done!"<<std::endl;



    // //Extracting the UV from [U,V,-U, -V];
    // cutUVRot=cutUVRot.block(0,0,cutUVRot.rows(),2);
    // // std::cout<<"Done!"<<std::endl;


    // //setting up mesh data from itnegration data
    // directional::MeshFunctionIsolinesData mfiData;
    // directional::setup_mesh_function_isolines(meshCut, intData, mfiData);

    // Eigen::MatrixXd VRemeshed;
    // Eigen::MatrixXi FRemeshed;
    // Eigen::VectorXi DRemeshed;
    // // Meshing and saving
    // directional::mesh_function_isolines(mesh, mfiData, true, VRemeshed, DRemeshed, FRemeshed);

    
    // saveObj("Remeshed.obj", VRemeshed, FRemeshed, DRemeshed);
    // std::cout<<"Remeshed mesh saved to "<<OUTPUT_PATH+"/remeshed.obj"<<std::endl;

    // for (int i=0;i<FRemeshed.rows();i++){
    //   for (int j=0;j<DRemeshed(i);j++){
    //     std::cout << FRemeshed(i,j) << " ";
    //   }
    //     std::cout << std::endl;
    // }

    // for (int i=0;i<VRemeshed.rows();i++){
        
    //     std::cout << VRemeshed.row(i) << std::endl;
    // }   

    // hedra::polygonal_write_OFF(OUTPUT_PATH + "/Remeshed.off", VRemeshed, DRemeshed, FRemeshed);
    // std::cout<<"Remeshed mesh saved to "<<OUTPUT_PATH+"/remeshed.obj"<<std::endl;

    // intData.verbose=true;
    // intData.integralSeamless=true;
    // std::cout<<"Solving for integrally-seamless integration"<<std::endl;
    

    // Set meshes
    viewer.set_mesh(mesh, 0);
    viewer.set_mesh(meshCut, 1);
    // Set mesh cutted
    // viewer.set_mesh(meshCut, 1);

    // Set fields
    // Field 0
    viewer.set_field(prosFaceField, directional::DirectionalViewer::indexed_glyph_colors(prosFaceField.extField, false), 0);
    // Field 1
    viewer.set_field(rawFaceField, directional::DirectionalViewer::indexed_glyph_colors(rawFaceField.extField, false), 1);


    // // Field 2
    viewer.set_field(combedField, directional::DirectionalViewer::indexed_glyph_colors(combedField.extField, false), 1);
    viewer.set_seams(combedField.matching);
    viewer.set_texture(texture_R,texture_G,texture_B,1);

    // Initial viewer setup
    //viewer.toggle_texture(false,0);
    viewer.toggle_field(true, 0);
    
    viewer.toggle_seams(true,0);

    viewer.toggle_texture(true,1);
    viewer.toggle_seams(true,1);
    //viewer.toggle_field(false,1);
    //viewer.toggle_seams(false,1);

  


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
