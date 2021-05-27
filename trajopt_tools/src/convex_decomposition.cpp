#include <trajopt_tools/convex_decomposition.h>
#include <tesseract_collision/core/common.h>
#include <console_bridge/console.h>
#include <vhacd/VHACD.h>

using namespace VHACD;

namespace tesseract_collision
{
class ProgressCallback : public IVHACD::IUserCallback
{
public:
  ProgressCallback() = default;
  ~ProgressCallback() override = default;
  ProgressCallback(const ProgressCallback&) = default;
  ProgressCallback& operator=(const ProgressCallback&) = default;
  ProgressCallback(ProgressCallback&&) = default;
  ProgressCallback& operator=(ProgressCallback&&) = default;

  void Update(const double overallProgress,
              const double stageProgress,
              const double operationProgress,
              const char* const stage,
              const char* const operation) override
  {
    std::cout << std::setfill(' ') << std::setw(3) << std::lround(overallProgress + 0.5) << "% "
              << "[ " << stage << " " << std::setfill(' ') << std::setw(3) << lround(stageProgress + 0.5) << "% ] "
              << operation << " " << std::setfill(' ') << std::setw(3) << std::lround(operationProgress + 0.5) << "%"
              << std::endl;
  }
};

VHACDConvexDeomposition::VHACDConvexDeomposition(const VHACDParameters& params) : params_(params) {}

std::vector<tesseract_geometry::ConvexMesh::Ptr>
VHACDConvexDeomposition::run(const tesseract_common::VectorVector3d& vertices, const Eigen::VectorXi& faces) const
{
  params_.print();

  std::vector<float> points_local;
  points_local.reserve(vertices.size() * 3);
  for (const auto& v : vertices)
  {
    points_local.push_back(v.x());
    points_local.push_back(v.y());
    points_local.push_back(v.z());
  }

  std::vector<unsigned int> triangles_local;
  triangles_local.reserve(faces.size() / 4);
  for (Eigen::Index i = 0; i < faces.rows();)
  {
    int face_vertice_cnt = faces(i++);
    if (face_vertice_cnt != 3)
      throw std::runtime_error("Currently only supports triangle meshes");

    triangles_local.push_back(faces(i++));
    triangles_local.push_back(faces(i++));
    triangles_local.push_back(faces(i++));
  }

  // run V-HACD
  IVHACD* interfaceVHACD = CreateVHACD();

  ProgressCallback progress_callback;
  VHACD::IVHACD::Parameters p;
  p.m_concavity = params_.concavity;
  p.m_alpha = params_.alpha;
  p.m_beta = params_.beta;
  p.m_minVolumePerCH = params_.minVolumePerCH;
  p.m_resolution = params_.resolution;
  p.m_maxNumVerticesPerCH = params_.maxNumVerticesPerCH;
  p.m_planeDownsampling = params_.planeDownsampling;
  p.m_convexhullDownsampling = params_.convexhullDownsampling;
  p.m_pca = params_.pca;
  p.m_mode = params_.mode;
  p.m_convexhullApproximation = params_.convexhullApproximation;
  p.m_oclAcceleration = params_.oclAcceleration;
  p.m_maxConvexHulls = params_.maxConvexHulls;
  p.m_projectHullVertices = params_.projectHullVertices;
  p.m_callback = &progress_callback;

  bool res = interfaceVHACD->Compute(&points_local[0],
                                     static_cast<unsigned int>(points_local.size() / 3),
                                     (const uint32_t*)(&triangles_local[0]),
                                     static_cast<unsigned int>(triangles_local.size() / 3),
                                     p);

  std::vector<tesseract_geometry::ConvexMesh::Ptr> output;
  if (res)
  {
    unsigned int num_convex_hulls = interfaceVHACD->GetNConvexHulls();
    CONSOLE_BRIDGE_logError("Convex decomposition generated %lu convex hulls!", num_convex_hulls);
    IVHACD::ConvexHull ch;
    for (unsigned int p = 0; p < num_convex_hulls; ++p)
    {
      interfaceVHACD->GetConvexHull(p, ch);

      auto vhacd_vertices = std::make_shared<tesseract_common::VectorVector3d>();
      vhacd_vertices->reserve(ch.m_nPoints);
      for (std::size_t i = 0; i < ch.m_nPoints; ++i)
      {
        Eigen::Vector3d v(ch.m_points[3 * i], ch.m_points[(3 * i) + 1], ch.m_points[(3 * i) + 2]);
        vhacd_vertices->push_back(v);
      }

      //      auto vhacd_triangles = std::make_shared<Eigen::VectorXi>();
      //      vhacd_triangles->resize(ch.m_nTriangles * 3);
      //      for (std::size_t i = 0; i < ch.m_nTriangles * 3; ++i)
      //      {
      //        (*vhacd_triangles)(i) = ch.m_triangles[i];
      //      }
      //      CONSOLE_BRIDGE_logError("Converted Vertices and triangles");
      //      output.push_back(std::make_shared<tesseract_geometry::ConvexMesh>(vhacd_vertices, vhacd_triangles,
      //      ch.m_nTriangles));
      auto ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
      auto ch_faces = std::make_shared<Eigen::VectorXi>();
      int ch_num_faces = ch_num_faces = tesseract_collision::createConvexHull(*ch_vertices, *ch_faces, *vhacd_vertices);
      output.push_back(std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces));
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("Decomposition cancelled by user!");
  }

  interfaceVHACD->Clean();
  interfaceVHACD->Release();

  return output;
}

void VHACDParameters::print() const
{
  std::stringstream msg;
  msg << "+ Parameters" << std::endl;
  msg << "\t resolution                                  " << resolution << std::endl;
  msg << "\t Max number of convex-hulls                  " << maxConvexHulls << std::endl;
  msg << "\t max. concavity                              " << concavity << std::endl;
  msg << "\t plane down-sampling                         " << planeDownsampling << std::endl;
  msg << "\t convex-hull down-sampling                   " << convexhullDownsampling << std::endl;
  msg << "\t alpha                                       " << alpha << std::endl;
  msg << "\t beta                                        " << beta << std::endl;
  msg << "\t pca                                         " << pca << std::endl;
  msg << "\t mode                                        " << mode << std::endl;
  msg << "\t max. vertices per convex-hull               " << maxNumVerticesPerCH << std::endl;
  msg << "\t min. volume to add vertices to convex-hulls " << minVolumePerCH << std::endl;
  msg << "\t convex-hull approximation                   " << convexhullApproximation << std::endl;
  msg << "\t OpenCL acceleration                         " << oclAcceleration << std::endl;
  //  msg << "\t OpenCL platform ID                          " << oclPlatformID << std::endl;
  //  msg << "\t OpenCL device ID                            " << oclDeviceID << std::endl;

  std::cout << msg.str();
}

}  // namespace tesseract_collision