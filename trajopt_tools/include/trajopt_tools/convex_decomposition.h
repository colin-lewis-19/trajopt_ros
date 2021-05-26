#ifndef TESSERACT_COLLISION_CONVEX_DECOMPOSITION_H
#define TESSERACT_COLLISION_CONVEX_DECOMPOSITION_H

#include <vector>
#include <memory>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/convex_mesh.h>

namespace tesseract_collision
{
class ConvexDecomposition
{
public:
  ~ConvexDecomposition() = default;
  /**
   * @brief Run convex decomposition algorithm
   * @param vertices The vertices
   * @param faces A vector of triangle indicies. Every face starts with the number of vertices followed the the vertice
   * index
   * @return
   */
  virtual std::vector<tesseract_geometry::ConvexMesh::Ptr> run(const tesseract_common::VectorVector3d& vertices,
                                                               const Eigen::VectorXi& faces) const = 0;
};

struct VHACDParameters
{
  double concavity{ 0.001 };
  double alpha{ 0.05 };
  double beta{ 0.05 };
  double minVolumePerCH{ 0.0001 };
  uint32_t resolution{ 1000 };
  uint32_t maxNumVerticesPerCH{ 256 };
  uint32_t planeDownsampling{ 4 };
  uint32_t convexhullDownsampling{ 4 };
  uint32_t pca{ 0 };
  uint32_t mode{ 0 };  // 0: voxel-based (recommended), 1: tetrahedron-based
  uint32_t convexhullApproximation{ true };
  uint32_t oclAcceleration{ true };
  uint32_t maxConvexHulls{ 1024 };
  /**
   * @brief This will project the output convex hull vertices onto the original source mesh
   *  to increase the floating point accuracy of the results
   */
  bool projectHullVertices{ true };

  void print() const;
};

class VHACDConvexDeomposition : public ConvexDecomposition
{
public:
  VHACDConvexDeomposition() = default;
  VHACDConvexDeomposition(const VHACDParameters& params);

  std::vector<tesseract_geometry::ConvexMesh::Ptr> run(const tesseract_common::VectorVector3d& vertices,
                                                       const Eigen::VectorXi& faces) const override;

private:
  VHACDParameters params_;
};

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_CONVEX_DECOMPOSITION_H
